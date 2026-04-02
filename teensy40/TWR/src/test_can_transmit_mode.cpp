#include "supervisor.h"
#include "test_can_transmit_mode.h"
#include "CAN_helper.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <string.h>

#define CAN_TX_PROOF_SUMMARY 0

static constexpr uint32_t IQREQ_DT_HIST_BIN_US = 50u;
static constexpr uint32_t IQREQ_DT_HIST_MAX_US = 20000u;
static constexpr uint32_t IQREQ_DT_HIST_BINS =
    (IQREQ_DT_HIST_MAX_US / IQREQ_DT_HIST_BIN_US) + 1u;
static constexpr uint32_t IQREQ_DT_SPIKE_MARGIN_US = 500u;
static constexpr uint32_t IQREQ_SPIKE_RING_SIZE = 128u;
static constexpr uint32_t IQREQ_POSVEL_CORR_WINDOW_US = 4000u;

static bool first_entry = true;
static uint32_t start_time_us = 0;
static uint32_t last_tx_us = 0;
static uint32_t last_rc_report_us = 0;

struct CanTxProofStats {
  uint32_t tx_attempts = 0;
  uint32_t tx_ok = 0;
  uint32_t tx_fail = 0;
  uint32_t last_report_us = 0;
};
static CanTxProofStats g_can_tx_proof;

struct IqreqSpikeRing {
  uint32_t ts[IQREQ_SPIKE_RING_SIZE];
  uint16_t head = 0;
  uint16_t size = 0;
  uint32_t total = 0;
};
static IqreqSpikeRing g_iqreq_spikes_left;
static IqreqSpikeRing g_iqreq_spikes_right;

struct IqreqDtStats {
  uint32_t count = 0;
  uint32_t last_tx_us = 0;
  uint32_t min_dt_us = UINT32_MAX;
  uint32_t max_dt_us = 0;
  uint64_t sum_dt_us = 0;
  uint32_t bins[IQREQ_DT_HIST_BINS] = {0};
};
static IqreqDtStats g_iqreq_dt_left;
static IqreqDtStats g_iqreq_dt_right;

static inline void reset_iqreq_spike_ring(IqreqSpikeRing &ring) {
  ring.head = 0u;
  ring.size = 0u;
  ring.total = 0u;
  memset(ring.ts, 0, sizeof(ring.ts));
}

static inline void iqreq_spike_ring_add(IqreqSpikeRing &ring, uint32_t ts_us) {
  if (ring.size < IQREQ_SPIKE_RING_SIZE) {
    const uint16_t idx = (uint16_t)((ring.head + ring.size) % IQREQ_SPIKE_RING_SIZE);
    ring.ts[idx] = ts_us;
    ring.size++;
  } else {
    ring.ts[ring.head] = ts_us;
    ring.head = (uint16_t)((ring.head + 1u) % IQREQ_SPIKE_RING_SIZE);
  }
  if (ring.total < UINT32_MAX) ring.total++;
}

static uint16_t copy_iqreq_spike_times(const IqreqSpikeRing &ring, uint32_t *out, uint16_t max_out) {
  if (out == nullptr || max_out == 0u) return 0u;
  uint16_t n = ring.size;
  if (n > max_out) n = max_out;
  const uint16_t start = (uint16_t)((ring.head + ring.size - n) % IQREQ_SPIKE_RING_SIZE);
  for (uint16_t i = 0u; i < n; ++i) {
    out[i] = ring.ts[(uint16_t)((start + i) % IQREQ_SPIKE_RING_SIZE)];
  }
  return n;
}

static uint16_t count_correlated_spikes(const uint32_t *a,
                                        uint16_t na,
                                        const uint32_t *b,
                                        uint16_t nb,
                                        uint32_t window_us) {
  if (a == nullptr || b == nullptr || na == 0u || nb == 0u) return 0u;
  uint16_t i = 0u;
  uint16_t j = 0u;
  uint16_t matches = 0u;
  while (i < na && j < nb) {
    const uint32_t ta = a[i];
    const uint32_t tb = b[j];
    const uint32_t diff = (ta > tb) ? (ta - tb) : (tb - ta);
    if (diff <= window_us) {
      matches++;
      i++;
      j++;
      continue;
    }
    if (ta < tb) {
      i++;
    } else {
      j++;
    }
  }
  return matches;
}

static inline void reset_iqreq_dt_stats(IqreqDtStats &st) {
  st.count = 0u;
  st.last_tx_us = 0u;
  st.min_dt_us = UINT32_MAX;
  st.max_dt_us = 0u;
  st.sum_dt_us = 0u;
  memset(st.bins, 0, sizeof(st.bins));
}

static inline uint32_t note_iqreq_tx(IqreqDtStats &st, uint32_t now_us) {
  if (st.last_tx_us != 0u) {
    const uint32_t dt_us = (uint32_t)(now_us - st.last_tx_us);
    if (dt_us < st.min_dt_us) st.min_dt_us = dt_us;
    if (dt_us > st.max_dt_us) st.max_dt_us = dt_us;
    st.sum_dt_us += dt_us;
    st.count++;
    uint32_t idx = dt_us / IQREQ_DT_HIST_BIN_US;
    if (idx >= IQREQ_DT_HIST_BINS) idx = IQREQ_DT_HIST_BINS - 1u;
    st.bins[idx]++;
    st.last_tx_us = now_us;
    return dt_us;
  }
  st.last_tx_us = now_us;
  return 0u;
}

static uint32_t iqreq_dt_percentile_us(const IqreqDtStats &st, float q) {
  if (st.count == 0u) return 0u;
  if (q < 0.0f) q = 0.0f;
  if (q > 1.0f) q = 1.0f;
  uint32_t target = (uint32_t)(q * (float)st.count + 0.5f);
  if (target == 0u) target = 1u;
  if (target > st.count) target = st.count;

  uint32_t acc = 0u;
  for (uint32_t i = 0u; i < IQREQ_DT_HIST_BINS; ++i) {
    acc += st.bins[i];
    if (acc >= target) {
      if (i == IQREQ_DT_HIST_BINS - 1u) {
        return IQREQ_DT_HIST_MAX_US;
      }
      return (i * IQREQ_DT_HIST_BIN_US) + (IQREQ_DT_HIST_BIN_US / 2u);
    }
  }
  return IQREQ_DT_HIST_MAX_US;
}

static void print_end_of_run_summary(const Supervisor_typedef *sup,
                                     uint32_t now_us,
                                     uint32_t tx_period_us) {
  if (sup == nullptr) return;

  const uint32_t attempts = g_can_tx_proof.tx_attempts;
  const float fail_pct = (attempts > 0u)
                           ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)attempts)
                           : 0.0f;
  const CanRuntimeStats rt = canGetRuntimeStats();
  const uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
  const uint32_t posvel_age_us =
      (last_posvel_rx_us > 0u) ? (uint32_t)(now_us - last_posvel_rx_us) : UINT32_MAX;
  const float tx_hz = (tx_period_us > 0u) ? (1000000.0f / (float)tx_period_us) : 0.0f;
  const uint32_t iqreq_l_avg_us =
      (g_iqreq_dt_left.count > 0u) ? (uint32_t)(g_iqreq_dt_left.sum_dt_us / g_iqreq_dt_left.count) : 0u;
  const uint32_t iqreq_r_avg_us =
      (g_iqreq_dt_right.count > 0u) ? (uint32_t)(g_iqreq_dt_right.sum_dt_us / g_iqreq_dt_right.count) : 0u;
  const uint32_t iqreq_l_p50_us = iqreq_dt_percentile_us(g_iqreq_dt_left, 0.50f);
  const uint32_t iqreq_l_p95_us = iqreq_dt_percentile_us(g_iqreq_dt_left, 0.95f);
  const uint32_t iqreq_l_p99_us = iqreq_dt_percentile_us(g_iqreq_dt_left, 0.99f);
  const uint32_t iqreq_r_p50_us = iqreq_dt_percentile_us(g_iqreq_dt_right, 0.50f);
  const uint32_t iqreq_r_p95_us = iqreq_dt_percentile_us(g_iqreq_dt_right, 0.95f);
  const uint32_t iqreq_r_p99_us = iqreq_dt_percentile_us(g_iqreq_dt_right, 0.99f);
  const uint32_t iqreq_dt_spike_threshold_us = tx_period_us + IQREQ_DT_SPIKE_MARGIN_US;

  uint32_t iq_l_spike_times[IQREQ_SPIKE_RING_SIZE];
  uint32_t iq_r_spike_times[IQREQ_SPIKE_RING_SIZE];
  uint32_t pos_l_spike_times[POSVEL_GAP_SPIKE_RING_SIZE];
  uint32_t pos_r_spike_times[POSVEL_GAP_SPIKE_RING_SIZE];
  const uint16_t iq_l_spike_n =
      copy_iqreq_spike_times(g_iqreq_spikes_left, iq_l_spike_times, IQREQ_SPIKE_RING_SIZE);
  const uint16_t iq_r_spike_n =
      copy_iqreq_spike_times(g_iqreq_spikes_right, iq_r_spike_times, IQREQ_SPIKE_RING_SIZE);

  Serial.printf(
      "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f,\"mode\":%d,\"tx_enable\":%d,\"tx_period_us\":%lu,\"tx_hz\":%.2f,\"rc_drive\":%d,\"rc_throttle_ch\":%u,\"rc_steer_ch\":%u,\"rc_deadband\":%.3f,\"rc_max_torque_nm\":%.3f,\"posvel_age_us\":%lu,\"can1_rx_reads\":%lu,\"can2_rx_reads\":%lu,\"rx_overflow\":%lu,"
      "\"iqreq_l_dt_count\":%lu,\"iqreq_l_dt_avg_us\":%lu,\"iqreq_l_dt_min_us\":%lu,\"iqreq_l_dt_p50_us\":%lu,\"iqreq_l_dt_p95_us\":%lu,\"iqreq_l_dt_p99_us\":%lu,\"iqreq_l_dt_max_us\":%lu,"
      "\"iqreq_r_dt_count\":%lu,\"iqreq_r_dt_avg_us\":%lu,\"iqreq_r_dt_min_us\":%lu,\"iqreq_r_dt_p50_us\":%lu,\"iqreq_r_dt_p95_us\":%lu,\"iqreq_r_dt_p99_us\":%lu,\"iqreq_r_dt_max_us\":%lu,"
      "\"iqreq_dt_spike_threshold_us\":%lu,\"iqreq_l_dt_spike_count\":%lu,\"iqreq_r_dt_spike_count\":%lu}\r\n",
      (unsigned long)g_can_tx_proof.tx_attempts,
      (unsigned long)g_can_tx_proof.tx_ok,
      (unsigned long)g_can_tx_proof.tx_fail,
      fail_pct,
      (int)sup->mode,
      sup->user_tx_enable ? 1 : 0,
      (unsigned long)tx_period_us,
      tx_hz,
      sup->user_rc_drive_enable ? 1 : 0,
      (unsigned int)(sup->user_rc_throttle_ch + 1u),
      (unsigned int)(sup->user_rc_steer_ch + 1u),
      sup->user_rc_deadband,
      sup->user_rc_max_torque_nm,
      (unsigned long)posvel_age_us,
      (unsigned long)rt.can1_rx_reads,
      (unsigned long)rt.can2_rx_reads,
      (unsigned long)rt.rx_overflow,
      (unsigned long)g_iqreq_dt_left.count,
      (unsigned long)iqreq_l_avg_us,
      (unsigned long)((g_iqreq_dt_left.count > 0u) ? g_iqreq_dt_left.min_dt_us : 0u),
      (unsigned long)iqreq_l_p50_us,
      (unsigned long)iqreq_l_p95_us,
      (unsigned long)iqreq_l_p99_us,
      (unsigned long)((g_iqreq_dt_left.count > 0u) ? g_iqreq_dt_left.max_dt_us : 0u),
      (unsigned long)g_iqreq_dt_right.count,
      (unsigned long)iqreq_r_avg_us,
      (unsigned long)((g_iqreq_dt_right.count > 0u) ? g_iqreq_dt_right.min_dt_us : 0u),
      (unsigned long)iqreq_r_p50_us,
      (unsigned long)iqreq_r_p95_us,
      (unsigned long)iqreq_r_p99_us,
      (unsigned long)((g_iqreq_dt_right.count > 0u) ? g_iqreq_dt_right.max_dt_us : 0u),
      (unsigned long)iqreq_dt_spike_threshold_us,
      (unsigned long)g_iqreq_spikes_left.total,
      (unsigned long)g_iqreq_spikes_right.total);

  PosvelRxStats left{};
  PosvelRxStats right{};
  PosvelRxStats can1_bus{};
  PosvelRxStats can2_bus{};

  const uint8_t left_id = (sup->esc_count > 0u) ? sup->esc[0].config.node_id : 0u;
  const uint8_t right_id = (sup->esc_count > 1u) ? sup->esc[1].config.node_id : 0u;
  const bool have_left = (left_id > 0u) ? canGetPosvelRxStats(left_id, left) : false;
  const bool have_right = (right_id > 0u) ? canGetPosvelRxStats(right_id, right) : false;
  const bool have_can1_bus = canGetBusPosvelRxStats(1u, can1_bus);
  const bool have_can2_bus = canGetBusPosvelRxStats(2u, can2_bus);
  const uint32_t posvel_gap_spike_threshold_us = canGetPosvelGapSpikeThresholdUs();
  const uint32_t left_posvel_gap_spike_count =
      (left_id > 0u) ? canGetPosvelGapSpikeCount(left_id) : 0u;
  const uint32_t right_posvel_gap_spike_count =
      (right_id > 0u) ? canGetPosvelGapSpikeCount(right_id) : 0u;
  const uint16_t pos_l_spike_n =
      (left_id > 0u) ? canCopyPosvelGapSpikeTimes(left_id, pos_l_spike_times, POSVEL_GAP_SPIKE_RING_SIZE) : 0u;
  const uint16_t pos_r_spike_n =
      (right_id > 0u) ? canCopyPosvelGapSpikeTimes(right_id, pos_r_spike_times, POSVEL_GAP_SPIKE_RING_SIZE) : 0u;
  const uint16_t left_spike_corr_matches =
      count_correlated_spikes(iq_l_spike_times, iq_l_spike_n, pos_l_spike_times, pos_l_spike_n, IQREQ_POSVEL_CORR_WINDOW_US);
  const uint16_t right_spike_corr_matches =
      count_correlated_spikes(iq_r_spike_times, iq_r_spike_n, pos_r_spike_times, pos_r_spike_n, IQREQ_POSVEL_CORR_WINDOW_US);

  const uint32_t left_age_us =
      (have_left && left.last_rx_us > 0u) ? (uint32_t)(now_us - left.last_rx_us) : UINT32_MAX;
  const uint32_t right_age_us =
      (have_right && right.last_rx_us > 0u) ? (uint32_t)(now_us - right.last_rx_us) : UINT32_MAX;
  const uint32_t left_avg_gap_us =
      (have_left && left.gap_count > 0u) ? (uint32_t)(left.sum_gap_us / left.gap_count) : 0u;
  const uint32_t right_avg_gap_us =
      (have_right && right.gap_count > 0u) ? (uint32_t)(right.sum_gap_us / right.gap_count) : 0u;
  const uint32_t left_min_gap_us =
      (have_left && left.gap_count > 0u) ? left.min_gap_us : 0u;
  const uint32_t right_min_gap_us =
      (have_right && right.gap_count > 0u) ? right.min_gap_us : 0u;
  const uint32_t left_p50_gap_us =
      (have_left && left.gap_count > 0u) ? left.p50_gap_us : 0u;
  const uint32_t right_p50_gap_us =
      (have_right && right.gap_count > 0u) ? right.p50_gap_us : 0u;
  const uint32_t left_p95_gap_us =
      (have_left && left.gap_count > 0u) ? left.p95_gap_us : 0u;
  const uint32_t right_p95_gap_us =
      (have_right && right.gap_count > 0u) ? right.p95_gap_us : 0u;
  const uint32_t left_p99_gap_us =
      (have_left && left.gap_count > 0u) ? left.p99_gap_us : 0u;
  const uint32_t right_p99_gap_us =
      (have_right && right.gap_count > 0u) ? right.p99_gap_us : 0u;
  const uint32_t can1_age_us =
      (have_can1_bus && can1_bus.last_rx_us > 0u) ? (uint32_t)(now_us - can1_bus.last_rx_us) : UINT32_MAX;
  const uint32_t can2_age_us =
      (have_can2_bus && can2_bus.last_rx_us > 0u) ? (uint32_t)(now_us - can2_bus.last_rx_us) : UINT32_MAX;
  const uint32_t can1_avg_gap_us =
      (have_can1_bus && can1_bus.gap_count > 0u) ? (uint32_t)(can1_bus.sum_gap_us / can1_bus.gap_count) : 0u;
  const uint32_t can2_avg_gap_us =
      (have_can2_bus && can2_bus.gap_count > 0u) ? (uint32_t)(can2_bus.sum_gap_us / can2_bus.gap_count) : 0u;
  const uint32_t can1_p50_gap_us =
      (have_can1_bus && can1_bus.gap_count > 0u) ? can1_bus.p50_gap_us : 0u;
  const uint32_t can2_p50_gap_us =
      (have_can2_bus && can2_bus.gap_count > 0u) ? can2_bus.p50_gap_us : 0u;
  const uint32_t can1_p95_gap_us =
      (have_can1_bus && can1_bus.gap_count > 0u) ? can1_bus.p95_gap_us : 0u;
  const uint32_t can2_p95_gap_us =
      (have_can2_bus && can2_bus.gap_count > 0u) ? can2_bus.p95_gap_us : 0u;
  const uint32_t can1_p99_gap_us =
      (have_can1_bus && can1_bus.gap_count > 0u) ? can1_bus.p99_gap_us : 0u;
  const uint32_t can2_p99_gap_us =
      (have_can2_bus && can2_bus.gap_count > 0u) ? can2_bus.p99_gap_us : 0u;
  const int32_t lr_offset_us =
      (have_left && have_right && left.last_rx_us > 0u && right.last_rx_us > 0u)
          ? (int32_t)(right.last_rx_us - left.last_rx_us)
          : INT32_MAX;
  const uint32_t lr_offset_abs_us =
      (lr_offset_us == INT32_MAX)
          ? UINT32_MAX
          : (uint32_t)((lr_offset_us < 0) ? -lr_offset_us : lr_offset_us);
  const int32_t lr_count_delta =
      (have_left && have_right)
          ? (int32_t)right.count - (int32_t)left.count
          : INT32_MAX;
  const uint32_t left_alive_false_count =
      (sup->esc_count > 0u) ? sup->esc_alive_false_count[0] : 0u;
  const uint32_t right_alive_false_count =
      (sup->esc_count > 1u) ? sup->esc_alive_false_count[1] : 0u;

  Serial.printf(
      "{\"cmd\":\"CAN_POSVEL_RX\",\"t\":%lu,"
      "\"left_id\":%u,\"left_count\":%lu,\"left_age_us\":%lu,\r\n"
      "\"left_avg_gap_us\":%lu,\"left_min_gap_us\":%lu,\"left_p50_gap_us\":%lu,\"left_p95_gap_us\":%lu,\"left_p99_gap_us\":%lu,\"left_max_gap_us\":%lu,\r\n\"left_est_missed\":%lu,"
      "\"left_alive_false_count\":%lu,"
      "\"right_id\":%u,\"right_count\":%lu,\"right_age_us\":%lu,\r\n"
      "\"right_avg_gap_us\":%lu,\"right_min_gap_us\":%lu,\"right_p50_gap_us\":%lu,\"right_p95_gap_us\":%lu,\"right_p99_gap_us\":%lu,\"right_max_gap_us\":%lu,\r\n\"right_est_missed\":%lu,"
      "\"right_alive_false_count\":%lu,"
      "\"can1_posvel_count\":%lu,\"can1_posvel_age_us\":%lu,\"can1_posvel_avg_gap_us\":%lu,\"can1_posvel_p50_gap_us\":%lu,\"can1_posvel_p95_gap_us\":%lu,\"can1_posvel_p99_gap_us\":%lu,"
      "\"can2_posvel_count\":%lu,\"can2_posvel_age_us\":%lu,\"can2_posvel_avg_gap_us\":%lu,\"can2_posvel_p50_gap_us\":%lu,\"can2_posvel_p95_gap_us\":%lu,\"can2_posvel_p99_gap_us\":%lu,"
      "\"posvel_gap_spike_threshold_us\":%lu,\"left_posvel_gap_spike_count\":%lu,\"right_posvel_gap_spike_count\":%lu,"
      "\"iqreq_posvel_corr_window_us\":%lu,\"left_iqreq_posvel_corr_matches\":%u,\"right_iqreq_posvel_corr_matches\":%u,"
      "\"can1_rx_reads\":%lu,\"can2_rx_reads\":%lu,\"rx_overflow\":%lu,"
      "\"lr_offset_us\":%ld,\"lr_offset_abs_us\":%lu,\"lr_count_delta\":%ld}\r\n",
      (unsigned long)now_us,
      left_id,
      (unsigned long)(have_left ? left.count : 0u),
      (unsigned long)left_age_us,
      (unsigned long)left_avg_gap_us,
      (unsigned long)left_min_gap_us,
      (unsigned long)left_p50_gap_us,
      (unsigned long)left_p95_gap_us,
      (unsigned long)left_p99_gap_us,
      (unsigned long)(have_left ? left.max_gap_us : 0u),
      (unsigned long)(have_left ? left.est_missed : 0u),
      (unsigned long)left_alive_false_count,
      right_id,
      (unsigned long)(have_right ? right.count : 0u),
      (unsigned long)right_age_us,
      (unsigned long)right_avg_gap_us,
      (unsigned long)right_min_gap_us,
      (unsigned long)right_p50_gap_us,
      (unsigned long)right_p95_gap_us,
      (unsigned long)right_p99_gap_us,
      (unsigned long)(have_right ? right.max_gap_us : 0u),
      (unsigned long)(have_right ? right.est_missed : 0u),
      (unsigned long)right_alive_false_count,
      (unsigned long)(have_can1_bus ? can1_bus.count : 0u),
      (unsigned long)can1_age_us,
      (unsigned long)can1_avg_gap_us,
      (unsigned long)can1_p50_gap_us,
      (unsigned long)can1_p95_gap_us,
      (unsigned long)can1_p99_gap_us,
      (unsigned long)(have_can2_bus ? can2_bus.count : 0u),
      (unsigned long)can2_age_us,
      (unsigned long)can2_avg_gap_us,
      (unsigned long)can2_p50_gap_us,
      (unsigned long)can2_p95_gap_us,
      (unsigned long)can2_p99_gap_us,
      (unsigned long)posvel_gap_spike_threshold_us,
      (unsigned long)left_posvel_gap_spike_count,
      (unsigned long)right_posvel_gap_spike_count,
      (unsigned long)IQREQ_POSVEL_CORR_WINDOW_US,
      (unsigned int)left_spike_corr_matches,
      (unsigned int)right_spike_corr_matches,
      (unsigned long)rt.can1_rx_reads,
      (unsigned long)rt.can2_rx_reads,
      (unsigned long)rt.rx_overflow,
      (long)lr_offset_us,
      (unsigned long)lr_offset_abs_us,
      (long)lr_count_delta);
}

static bool esc_is_alive(const Supervisor_typedef *sup, uint8_t esc_num) {
  return (sup != nullptr) && (esc_num < sup->esc_count) && sup->esc[esc_num].state.alive;
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float apply_deadband(float x, float deadband) {
  const float d = clampf(deadband, 0.0f, 0.49f);
  if (fabsf(x) <= d) return 0.0f;
  const float sign = (x >= 0.0f) ? 1.0f : -1.0f;
  return sign * ((fabsf(x) - d) / (1.0f - d));
}

static inline float rc_channel_norm(const Supervisor_typedef *sup, uint8_t idx, bool *valid_out) {
  if (valid_out) *valid_out = false;
  if (sup == nullptr) return 0.0f;
  if (idx >= sup->rc_count) return 0.0f;
  if (!sup->rc[idx].valid) return 0.0f;
  if (valid_out) *valid_out = true;
  return clampf(sup->rc[idx].norm, -1.0f, 1.0f);
}

static bool ESC_torque_cmd(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                           FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2,
                           uint8_t esc_num,
                           float torque) {
  if (!sup) return false;
  if (esc_num >= sup->esc_count) return false;

  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[esc_num].config.node_id);
  msg.len = 8;
  msg.flags.extended = 1;
  canPackFloat(torque, msg.buf);
  canPackFloat(0.0f, msg.buf + 4);
  const uint8_t tx_bus = can_tx_bus_for_node(sup->esc[esc_num].config.node_id);
  const bool ok1 = (tx_bus == CAN_TX_BUS_CAN1 || tx_bus == CAN_TX_BUS_BOTH)
                     ? can1.write(msg)
                     : false;
  const bool ok2 = (tx_bus == CAN_TX_BUS_CAN2 || tx_bus == CAN_TX_BUS_BOTH)
                     ? can2.write(msg)
                     : false;
  return ok1 || ok2;
}

void test_can_transmit_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                            FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  if (!sup) return;
  const bool aliveL = esc_is_alive(sup, 0);
  const bool aliveR = esc_is_alive(sup, 1);

  if (!aliveL && !aliveR) {
    if (sup->esc_count > 0u) {
      ESC_torque_cmd(sup, can1, can2, 0, 0.0f);
    }
    if (sup->esc_count > 1u) {
      ESC_torque_cmd(sup, can1, can2, 1, 0.0f);
    }

    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_tx_us = 0;
    return;
  }

  if (first_entry) {
    first_entry = false;
    start_time_us = micros();
    last_tx_us = 0;
    g_can_tx_proof = CanTxProofStats{};
    reset_iqreq_dt_stats(g_iqreq_dt_left);
    reset_iqreq_dt_stats(g_iqreq_dt_right);
    reset_iqreq_spike_ring(g_iqreq_spikes_left);
    reset_iqreq_spike_ring(g_iqreq_spikes_right);
    g_can_tx_proof.last_report_us = start_time_us;
    last_rc_report_us = 0u;
  }

  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - start_time_us;
  const uint32_t tx_period_us = (sup->user_tx_period_us > 0u) ? sup->user_tx_period_us : 1000u;
  const uint32_t iqreq_dt_spike_threshold_us = tx_period_us + IQREQ_DT_SPIKE_MARGIN_US;
  const bool tx_due = (last_tx_us == 0u) || ((uint32_t)(now_us - last_tx_us) >= tx_period_us);

  bool rc_throttle_valid = false;
  bool rc_steer_valid = false;
  const float rc_throttle_raw = rc_channel_norm(sup, sup->user_rc_throttle_ch, &rc_throttle_valid);
  const float rc_steer_raw = rc_channel_norm(sup, sup->user_rc_steer_ch, &rc_steer_valid);
  const float rc_throttle = apply_deadband(rc_throttle_raw, sup->user_rc_deadband);
  const float rc_steer = apply_deadband(rc_steer_raw, sup->user_rc_deadband);
  const bool rc_ok = rc_throttle_valid && rc_steer_valid;

  float tau_l_cmd = sup->user_test_tau_left;
  float tau_r_cmd = sup->user_test_tau_right;
  if (sup->user_rc_drive_enable && rc_ok) {
    const float max_tau = clampf(sup->user_rc_max_torque_nm, 0.0f, 2.0f);
    const float base = max_tau * rc_throttle;
    const float steer = max_tau * rc_steer;
    tau_l_cmd = clampf(base + steer, -max_tau, max_tau);
    tau_r_cmd = clampf(-base + steer, -max_tau, max_tau);
  } else if (sup->user_rc_drive_enable && !rc_ok) {
    tau_l_cmd = 0.0f;
    tau_r_cmd = 0.0f;
  }

  if (sup->user_tx_enable && tx_due) {
    last_tx_us = now_us;
    if (aliveL) {
      const uint32_t dt_l = note_iqreq_tx(g_iqreq_dt_left, now_us);
      if (dt_l >= iqreq_dt_spike_threshold_us) {
        iqreq_spike_ring_add(g_iqreq_spikes_left, now_us);
      }
      const bool okL = ESC_torque_cmd(sup, can1, can2, 0, tau_l_cmd);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okL ? 1u : 0u;
      g_can_tx_proof.tx_fail += okL ? 0u : 1u;
    }
    if (aliveR) {
      const uint32_t dt_r = note_iqreq_tx(g_iqreq_dt_right, now_us);
      if (dt_r >= iqreq_dt_spike_threshold_us) {
        iqreq_spike_ring_add(g_iqreq_spikes_right, now_us);
      }
      const bool okR = ESC_torque_cmd(sup, can1, can2, 1, tau_r_cmd);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okR ? 1u : 0u;
      g_can_tx_proof.tx_fail += okR ? 0u : 1u;
    }
  }

  if (sup->user_rc_drive_enable &&
      ((last_rc_report_us == 0u) || ((uint32_t)(now_us - last_rc_report_us) >= 100000u))) {
    last_rc_report_us = now_us;
    const uint8_t ch_t = sup->user_rc_throttle_ch;
    const uint8_t ch_s = sup->user_rc_steer_ch;
    const uint16_t raw_t = (ch_t < sup->rc_count) ? sup->rc[ch_t].raw_us : 0u;
    const uint16_t raw_s = (ch_s < sup->rc_count) ? sup->rc[ch_s].raw_us : 0u;
    Serial.printf(
        "{\"cmd\":\"RC_TEST\",\"t\":%lu,\"tx_enable\":%d,\"tx_hz\":%.2f,"
        "\"throttle_ch\":%u,\"throttle_valid\":%d,\"throttle_raw_us\":%u,\"throttle_norm\":%.3f,"
        "\"steer_ch\":%u,\"steer_valid\":%d,\"steer_raw_us\":%u,\"steer_norm\":%.3f,"
        "\"tau_l\":%.3f,\"tau_r\":%.3f}\r\n",
        (unsigned long)now_us,
        sup->user_tx_enable ? 1 : 0,
        (tx_period_us > 0u) ? (1000000.0f / (float)tx_period_us) : 0.0f,
        (unsigned int)(ch_t + 1u), rc_throttle_valid ? 1 : 0, (unsigned int)raw_t, rc_throttle,
        (unsigned int)(ch_s + 1u), rc_steer_valid ? 1 : 0, (unsigned int)raw_s, rc_steer,
        tau_l_cmd, tau_r_cmd);
  }

#if CAN_TX_PROOF_SUMMARY
  if ((uint32_t)(now_us - g_can_tx_proof.last_report_us) >= 1000000u) {
    const uint32_t attempts = g_can_tx_proof.tx_attempts;
    const float fail_pct = (attempts > 0u)
                             ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)attempts)
                             : 0.0f;
    const CanRuntimeStats rt = canGetRuntimeStats();
    const uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
    const uint32_t posvel_age_us =
        (last_posvel_rx_us > 0u) ? (uint32_t)(now_us - last_posvel_rx_us) : UINT32_MAX;
    const float tx_hz = (tx_period_us > 0u) ? (1000000.0f / (float)tx_period_us) : 0.0f;
    Serial.printf(
        "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f,\"mode\":%d,\"tx_enable\":%d,\"tx_period_us\":%lu,\"tx_hz\":%.2f,\"posvel_age_us\":%lu,\"can1_rx_reads\":%lu,\"can2_rx_reads\":%lu,\"rx_overflow\":%lu}\r\n",
        (unsigned long)g_can_tx_proof.tx_attempts,
        (unsigned long)g_can_tx_proof.tx_ok,
        (unsigned long)g_can_tx_proof.tx_fail,
        fail_pct,
        (int)sup->mode,
        sup->user_tx_enable ? 1 : 0,
        (unsigned long)tx_period_us,
        tx_hz,
        (unsigned long)posvel_age_us,
        (unsigned long)rt.can1_rx_reads,
        (unsigned long)rt.can2_rx_reads,
        (unsigned long)rt.rx_overflow);
    g_can_tx_proof.last_report_us += 1000000u;
  }
#endif

  if (sup->user_total_us > 0 && elapsed_us > sup->user_total_us) {
    if (sup->esc_count > 0u) {
      const bool okStopL = ESC_torque_cmd(sup, can1, can2, 0, 0.0f);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okStopL ? 1u : 0u;
      g_can_tx_proof.tx_fail += okStopL ? 0u : 1u;
    }
    if (sup->esc_count > 1u) {
      const bool okStopR = ESC_torque_cmd(sup, can1, can2, 1, 0.0f);
      g_can_tx_proof.tx_attempts += 1u;
      g_can_tx_proof.tx_ok += okStopR ? 1u : 0u;
      g_can_tx_proof.tx_fail += okStopR ? 0u : 1u;
    }
    print_end_of_run_summary(sup, now_us, tx_period_us);
    Serial.printf(
      "{\"cmd\":\"CAN_TX_DONE\",\"elapsed_us\":%lu,\"limit_us\":%lu,\"tx_fail\":%lu}\r\n",
      (unsigned long)elapsed_us,
      (unsigned long)sup->user_total_us,
      (unsigned long)g_can_tx_proof.tx_fail);

    // End this run deterministically and wait for the next explicit "run" command.
    sup->mode = SUP_MODE_IDLE;
    sup->user_total_us = 0;
    first_entry = true;
    last_tx_us = 0;
  }
}
