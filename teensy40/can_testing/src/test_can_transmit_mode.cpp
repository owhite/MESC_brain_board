#include "supervisor.h"
#include "test_can_transmit_mode.h"
#include "CAN_helper.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>

static bool g_first_entry = true;
static uint32_t g_start_time_us = 0u;
static uint32_t g_last_tx_us = 0u;
static uint32_t g_tx_attempts = 0u;
static uint32_t g_tx_ok = 0u;
static uint32_t g_tx_fail = 0u;

// CAN reliability improvement:
// Report callback-ingress vs main-loop sequence continuity to localize loss.
static void print_min_ingress_seq_summary(const Supervisor_typedef *sup) {
  if (sup == nullptr) return;

  const uint8_t left_id = (sup->esc_count > 0u) ? sup->esc[0].config.node_id : 0u;
  const uint8_t right_id = (sup->esc_count > 1u) ? sup->esc[1].config.node_id : 0u;

  PosvelSeqStats left_cb{};
  PosvelSeqStats right_cb{};
  PosvelSeqStats left_main{};
  PosvelSeqStats right_main{};

  const bool have_left_cb = (left_id > 0u) ? canGetPosvelIngressSeqStats(left_id, left_cb) : false;
  const bool have_right_cb = (right_id > 0u) ? canGetPosvelIngressSeqStats(right_id, right_cb) : false;
  const bool have_left_main = (left_id > 0u) ? canGetPosvelSeqStats(left_id, left_main) : false;
  const bool have_right_main = (right_id > 0u) ? canGetPosvelSeqStats(right_id, right_main) : false;

  Serial.printf(
      "{\"cmd\":\"CAN_INGRESS_SEQ\","
      "\"left_id\":%u,\"left_cb_valid\":%lu,\"left_cb_missed\":%lu,\"left_cb_dup\":%lu,\"left_cb_ooo\":%lu,"
      "\"left_main_valid\":%lu,\"left_main_missed\":%lu,\"left_main_dup\":%lu,\"left_main_ooo\":%lu,"
      "\"right_id\":%u,\"right_cb_valid\":%lu,\"right_cb_missed\":%lu,\"right_cb_dup\":%lu,\"right_cb_ooo\":%lu,"
      "\"right_main_valid\":%lu,\"right_main_missed\":%lu,\"right_main_dup\":%lu,\"right_main_ooo\":%lu}\r\n",
      left_id,
      (unsigned long)(have_left_cb ? left_cb.valid_count : 0u),
      (unsigned long)(have_left_cb ? left_cb.missed_total : 0u),
      (unsigned long)(have_left_cb ? left_cb.duplicate_total : 0u),
      (unsigned long)(have_left_cb ? left_cb.out_of_order_total : 0u),
      (unsigned long)(have_left_main ? left_main.valid_count : 0u),
      (unsigned long)(have_left_main ? left_main.missed_total : 0u),
      (unsigned long)(have_left_main ? left_main.duplicate_total : 0u),
      (unsigned long)(have_left_main ? left_main.out_of_order_total : 0u),
      right_id,
      (unsigned long)(have_right_cb ? right_cb.valid_count : 0u),
      (unsigned long)(have_right_cb ? right_cb.missed_total : 0u),
      (unsigned long)(have_right_cb ? right_cb.duplicate_total : 0u),
      (unsigned long)(have_right_cb ? right_cb.out_of_order_total : 0u),
      (unsigned long)(have_right_main ? right_main.valid_count : 0u),
      (unsigned long)(have_right_main ? right_main.missed_total : 0u),
      (unsigned long)(have_right_main ? right_main.duplicate_total : 0u),
      (unsigned long)(have_right_main ? right_main.out_of_order_total : 0u));
}

static void print_min_f405_ovr_summary(const Supervisor_typedef *sup, uint32_t now_us) {
  if (sup == nullptr) return;

  const uint8_t left_id = (sup->esc_count > 0u) ? sup->esc[0].config.node_id : 0u;
  const uint8_t right_id = (sup->esc_count > 1u) ? sup->esc[1].config.node_id : 0u;
  F405OverrunStats left_ovr{};
  F405OverrunStats right_ovr{};
  const bool have_left = (left_id > 0u) ? canGetF405OverrunStats(left_id, left_ovr) : false;
  const bool have_right = (right_id > 0u) ? canGetF405OverrunStats(right_id, right_ovr) : false;
  const uint32_t left_age_us =
      (have_left && left_ovr.last_rx_us > 0u) ? (uint32_t)(now_us - left_ovr.last_rx_us) : UINT32_MAX;
  const uint32_t right_age_us =
      (have_right && right_ovr.last_rx_us > 0u) ? (uint32_t)(now_us - right_ovr.last_rx_us) : UINT32_MAX;

  Serial.printf(
      "{\"cmd\":\"CAN_F405_OVR\",\"left_id\":%u,\"left_fov0\":%lu,\"left_fov1\":%lu,\"left_fov_age_us\":%lu,"
      "\"right_id\":%u,\"right_fov0\":%lu,\"right_fov1\":%lu,\"right_fov_age_us\":%lu}\r\n",
      left_id,
      (unsigned long)(have_left ? left_ovr.fifo0_overrun_count : 0u),
      (unsigned long)(have_left ? left_ovr.fifo1_overrun_count : 0u),
      (unsigned long)left_age_us,
      right_id,
      (unsigned long)(have_right ? right_ovr.fifo0_overrun_count : 0u),
      (unsigned long)(have_right ? right_ovr.fifo1_overrun_count : 0u),
      (unsigned long)right_age_us);
}

static void print_min_f405_iqreq_summary(const Supervisor_typedef *sup, uint32_t now_us) {
  if (sup == nullptr) return;

  const uint8_t left_id = (sup->esc_count > 0u) ? sup->esc[0].config.node_id : 0u;
  const uint8_t right_id = (sup->esc_count > 1u) ? sup->esc[1].config.node_id : 0u;
  F405IqreqStats left{};
  F405IqreqStats right{};
  const bool have_left = (left_id > 0u) ? canGetF405IqreqStats(left_id, left) : false;
  const bool have_right = (right_id > 0u) ? canGetF405IqreqStats(right_id, right) : false;
  const uint32_t left_age_us =
      (have_left && left.last_rx_us > 0u) ? (uint32_t)(now_us - left.last_rx_us) : UINT32_MAX;
  const uint32_t right_age_us =
      (have_right && right.last_rx_us > 0u) ? (uint32_t)(now_us - right.last_rx_us) : UINT32_MAX;

  Serial.printf(
      "{\"cmd\":\"CAN_F405_IQREQ\",\"left_id\":%u,\"left_valid\":%lu,\"left_missed\":%lu,\"left_age_us\":%lu,"
      "\"right_id\":%u,\"right_valid\":%lu,\"right_missed\":%lu,\"right_age_us\":%lu}\r\n",
      left_id,
      (unsigned long)(have_left ? left.iqreq_seq_valid_count : 0u),
      (unsigned long)(have_left ? left.iqreq_seq_missed_total : 0u),
      (unsigned long)left_age_us,
      right_id,
      (unsigned long)(have_right ? right.iqreq_seq_valid_count : 0u),
      (unsigned long)(have_right ? right.iqreq_seq_missed_total : 0u),
      (unsigned long)right_age_us);
}

static void print_min_rx_summary(const Supervisor_typedef *sup, uint32_t now_us) {
  if (sup == nullptr) return;

  const uint8_t left_id = (sup->esc_count > 0u) ? sup->esc[0].config.node_id : 0u;
  const uint8_t right_id = (sup->esc_count > 1u) ? sup->esc[1].config.node_id : 0u;

  PosvelRxStats left{};
  PosvelRxStats right{};
  PosvelSeqStats left_seq{};
  PosvelSeqStats right_seq{};
  const bool have_left = (left_id > 0u) ? canGetPosvelRxStats(left_id, left) : false;
  const bool have_right = (right_id > 0u) ? canGetPosvelRxStats(right_id, right) : false;
  const bool have_left_seq = (left_id > 0u) ? canGetPosvelSeqStats(left_id, left_seq) : false;
  const bool have_right_seq = (right_id > 0u) ? canGetPosvelSeqStats(right_id, right_seq) : false;
  const CanRuntimeStats rt = canGetRuntimeStats();

  const uint32_t left_age_us =
      (have_left && left.last_rx_us > 0u) ? (uint32_t)(now_us - left.last_rx_us) : UINT32_MAX;
  const uint32_t right_age_us =
      (have_right && right.last_rx_us > 0u) ? (uint32_t)(now_us - right.last_rx_us) : UINT32_MAX;
  const uint32_t left_alive_false_count =
      (sup->esc_count > 0u) ? sup->esc_alive_false_count[0] : 0u;
  const uint32_t right_alive_false_count =
      (sup->esc_count > 1u) ? sup->esc_alive_false_count[1] : 0u;

  Serial.printf(
      "{\"cmd\":\"CAN_RX_SUM\","
      "\"left_id\":%u,\"left_count\":%lu,\"left_age_us\":%lu,\"left_est_missed\":%lu,\"left_seq_valid\":%lu,\"left_seq_missed\":%lu,\"left_seq_dup\":%lu,\"left_seq_ooo\":%lu,\"left_seq_burst_miss_max\":%lu,\"left_alive_false_count\":%lu,"
      "\"right_id\":%u,\"right_count\":%lu,\"right_age_us\":%lu,\"right_est_missed\":%lu,\"right_seq_valid\":%lu,\"right_seq_missed\":%lu,\"right_seq_dup\":%lu,\"right_seq_ooo\":%lu,\"right_seq_burst_miss_max\":%lu,\"right_alive_false_count\":%lu,"
      "\"can1_rx_reads\":%lu,\"can2_rx_reads\":%lu,\"rx_overflow\":%lu}\r\n",
      left_id,
      (unsigned long)(have_left ? left.count : 0u),
      (unsigned long)left_age_us,
      (unsigned long)(have_left ? left.est_missed : 0u),
      (unsigned long)(have_left_seq ? left_seq.valid_count : 0u),
      (unsigned long)(have_left_seq ? left_seq.missed_total : 0u),
      (unsigned long)(have_left_seq ? left_seq.duplicate_total : 0u),
      (unsigned long)(have_left_seq ? left_seq.out_of_order_total : 0u),
      (unsigned long)(have_left_seq ? left_seq.burst_miss_max : 0u),
      (unsigned long)left_alive_false_count,
      right_id,
      (unsigned long)(have_right ? right.count : 0u),
      (unsigned long)right_age_us,
      (unsigned long)(have_right ? right.est_missed : 0u),
      (unsigned long)(have_right_seq ? right_seq.valid_count : 0u),
      (unsigned long)(have_right_seq ? right_seq.missed_total : 0u),
      (unsigned long)(have_right_seq ? right_seq.duplicate_total : 0u),
      (unsigned long)(have_right_seq ? right_seq.out_of_order_total : 0u),
      (unsigned long)(have_right_seq ? right_seq.burst_miss_max : 0u),
      (unsigned long)right_alive_false_count,
      (unsigned long)rt.can1_rx_reads,
      (unsigned long)rt.can2_rx_reads,
      (unsigned long)rt.rx_overflow);
}

static void print_min_can_events_summary() {
  CanEventDispatchStats can1{};
  CanEventDispatchStats can2{};
  const bool have_can1 = canGetEventsDispatchStats(1u, can1);
  const bool have_can2 = canGetEventsDispatchStats(2u, can2);

  Serial.printf(
      "{\"cmd\":\"CAN_EVENTS_SUM\","
      "\"can1_calls\":%lu,\"can1_dt_min_us\":%lu,\"can1_dt_avg_us\":%lu,\"can1_dt_max_us\":%lu,\"can1_over_1000_us\":%lu,\"can1_over_2000_us\":%lu,"
      "\"can2_calls\":%lu,\"can2_dt_min_us\":%lu,\"can2_dt_avg_us\":%lu,\"can2_dt_max_us\":%lu,\"can2_over_1000_us\":%lu,\"can2_over_2000_us\":%lu}\r\n",
      (unsigned long)(have_can1 ? can1.call_count : 0u),
      (unsigned long)(have_can1 ? can1.min_dt_us : 0u),
      (unsigned long)(have_can1 ? can1.avg_dt_us : 0u),
      (unsigned long)(have_can1 ? can1.max_dt_us : 0u),
      (unsigned long)(have_can1 ? can1.over_1000_us : 0u),
      (unsigned long)(have_can1 ? can1.over_2000_us : 0u),
      (unsigned long)(have_can2 ? can2.call_count : 0u),
      (unsigned long)(have_can2 ? can2.min_dt_us : 0u),
      (unsigned long)(have_can2 ? can2.avg_dt_us : 0u),
      (unsigned long)(have_can2 ? can2.max_dt_us : 0u),
      (unsigned long)(have_can2 ? can2.over_1000_us : 0u),
      (unsigned long)(have_can2 ? can2.over_2000_us : 0u));
}

static bool esc_is_alive(const Supervisor_typedef *sup, uint8_t esc_num) {
  return (sup != nullptr) && (esc_num < sup->esc_count) && sup->esc[esc_num].state.alive;
}

static bool ESC_torque_cmd(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                           FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2,
                           uint8_t esc_num,
                           float torque) {
  if (sup == nullptr) return false;
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

void test_can_transmit_mode_request_restart(void) {
  g_first_entry = true;
  g_start_time_us = 0u;
  g_last_tx_us = 0u;
  g_tx_attempts = 0u;
  g_tx_ok = 0u;
  g_tx_fail = 0u;
}

void test_can_transmit_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                            FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  if (!sup) return;

  const bool aliveL = esc_is_alive(sup, 0);
  const bool aliveR = esc_is_alive(sup, 1);

  if (!aliveL && !aliveR) {
    sup->mode = SUP_MODE_IDLE;
    Serial.printf(
        "{\"cmd\":\"STATE_CHANGE\",\"reason\":\"run_aborted_no_alive_esc\",\"mode_from\":\"SUP_MODE_TEST_CAN\",\"mode_to\":\"SUP_MODE_IDLE\"}\r\n");
    g_first_entry = true;
    g_start_time_us = 0u;
    g_last_tx_us = 0u;
    return;
  }

  if (g_first_entry) {
    g_first_entry = false;
    g_start_time_us = micros();
    g_last_tx_us = 0u;
    g_tx_attempts = 0u;
    g_tx_ok = 0u;
    g_tx_fail = 0u;
    Serial.printf(
        "{\"cmd\":\"TEST_CAN_STEP1_ENTER\",\"run_us\":%lu,\"run_s\":%lu,\"tx_enable\":%d,\"tx_period_us\":%lu}\r\n",
        (unsigned long)sup->user_total_us,
        (unsigned long)(sup->user_total_us / 1000000u),
        sup->user_tx_enable ? 1 : 0,
        (unsigned long)sup->user_tx_period_us);
  }

  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - g_start_time_us;
  const uint32_t tx_period_us = (sup->user_tx_period_us > 0u) ? sup->user_tx_period_us : 1000u;
  const bool tx_due = (g_last_tx_us == 0u) || ((uint32_t)(now_us - g_last_tx_us) >= tx_period_us);

  if (sup->user_tx_enable && tx_due) {
    g_last_tx_us = now_us;
    if (aliveL && sup->esc_count > 0u) {
      const bool okL = ESC_torque_cmd(sup, can1, can2, 0, sup->user_test_tau_left);
      g_tx_attempts++;
      g_tx_ok += okL ? 1u : 0u;
      g_tx_fail += okL ? 0u : 1u;
    }
    if (aliveR && sup->esc_count > 1u) {
      const bool okR = ESC_torque_cmd(sup, can1, can2, 1, sup->user_test_tau_right);
      g_tx_attempts++;
      g_tx_ok += okR ? 1u : 0u;
      g_tx_fail += okR ? 0u : 1u;
    }
  }

  if (sup->user_total_us > 0u && elapsed_us >= sup->user_total_us) {
    if (sup->esc_count > 0u) {
      const bool okStopL = ESC_torque_cmd(sup, can1, can2, 0, 0.0f);
      g_tx_attempts++;
      g_tx_ok += okStopL ? 1u : 0u;
      g_tx_fail += okStopL ? 0u : 1u;
    }
    if (sup->esc_count > 1u) {
      const bool okStopR = ESC_torque_cmd(sup, can1, can2, 1, 0.0f);
      g_tx_attempts++;
      g_tx_ok += okStopR ? 1u : 0u;
      g_tx_fail += okStopR ? 0u : 1u;
    }

    // CAN reliability improvement:
    // Emit compact end-of-run diagnostics needed for A/B and soak comparisons.
    print_min_f405_ovr_summary(sup, now_us);
    print_min_f405_iqreq_summary(sup, now_us);
    print_min_ingress_seq_summary(sup);
    print_min_rx_summary(sup, now_us);
    print_min_can_events_summary();
    Serial.printf(
        "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"tx_enable\":%d,\"tx_period_us\":%lu}\r\n",
        (unsigned long)g_tx_attempts,
        (unsigned long)g_tx_ok,
        (unsigned long)g_tx_fail,
        sup->user_tx_enable ? 1 : 0,
        (unsigned long)tx_period_us);
    Serial.printf(
        "{\"cmd\":\"TEST_CAN_STEP1_DONE\",\"elapsed_us\":%lu,\"limit_us\":%lu,\"tx_fail\":%lu}\r\n",
        (unsigned long)elapsed_us,
        (unsigned long)sup->user_total_us,
        (unsigned long)g_tx_fail);

    sup->mode = SUP_MODE_IDLE;
    Serial.printf(
        "{\"cmd\":\"STATE_CHANGE\",\"reason\":\"run_complete_step1\",\"mode_from\":\"SUP_MODE_TEST_CAN\",\"mode_to\":\"SUP_MODE_IDLE\",\"elapsed_us\":%lu}\r\n",
        (unsigned long)elapsed_us);

    sup->user_total_us = 0u;
    g_first_entry = true;
    g_start_time_us = 0u;
    g_last_tx_us = 0u;
  }
}
