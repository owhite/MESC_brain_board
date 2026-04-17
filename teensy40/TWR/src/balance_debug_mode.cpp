#include "balance_debug_mode.h"

#include "CAN_helper.h"
#include "main.h"

#include <Arduino.h>
#include <math.h>

#ifndef SEND_TORQUE
#define SEND_TORQUE 1
#endif

// Baseline V1: simple full-state LQR at fixed 500 Hz command cadence.
// State ordering: [theta, theta_dot, x_wheel, x_dot]^T
// Top-down shaped retune:
// - more wheel-position centering authority (Kx)
// - less high-frequency rate aggression (K_theta_dot, K_x_dot)
static constexpr float K_DISC[4] = {
    5.30f,   // K_theta
    0.42f,   // K_theta_dot
    1.00f,   // K_x      -> position-hold (wheel center return)
    0.35f    // K_x_dot  -> damping on platform translation
};
// Diagnostic isolation mode:
// true  -> disable x-position hold channel to validate tare/IMU equilibrium.
// false -> full 4-state balance + position-hold behavior.
static constexpr bool BALANCE_TARE_VALIDATION_MODE = false;
// Slow integral action on wheel-position error for bias rejection.
// This helps hold a fixed wheel position in the presence of small constant
// offsets (IMU bias, motor mismatch, asymmetrical friction).
static constexpr float POS_HOLD_KI = 0.35f;         // Nm per (m*s)
static constexpr float POS_HOLD_I_CLAMP_NM = 0.35f; // clamp integral contribution
// Enable position-hold terms only when we're close enough to upright.
// This prevents x-channel terms from fighting aggressive tilt recovery.
static constexpr float X_HOLD_ENABLE_THETA_RAD = 0.05f;
static constexpr float X_HOLD_ENABLE_THETA_DOT_RAD_S = 0.35f;
static constexpr float X_HOLD_ENABLE_X_DOT_M_S = 0.03f;
// When disabled, decay the x integrator to avoid persistent bias build-up.
static constexpr float X_HOLD_INT_LEAK_PER_STEP = 0.995f;
// Disable automatic x_ref chasing by default so hold mode keeps the wheels
// parked near the original start location.
static constexpr bool X_REF_RECENTER_ENABLE = false;
// Slow re-center of x_ref when truly stable (hold mode only), to avoid
// long-lived lean bias from tiny tare/model offsets.
static constexpr float X_REF_RECENTER_THETA_RAD = 0.04f;
static constexpr float X_REF_RECENTER_THETA_DOT_RAD_S = 0.30f;
static constexpr float X_REF_RECENTER_XDOT_M_S = 0.05f;
static constexpr uint32_t X_REF_RECENTER_STABLE_US = 700000u;
static constexpr float X_REF_RECENTER_TAU_S = 2.0f;

// Keep aligned with model assumptions.
static constexpr float WHEEL_RADIUS_M = 0.05278f;
static constexpr float LEFT_WHEEL_SIGN = 1.0f;
static constexpr float RIGHT_WHEEL_SIGN = -1.0f;

static constexpr float TORQUE_CLAMP_NM = 2.0f;
static constexpr float SAFETY_SCALE = 1.0f;
static constexpr float THETA_FAIL_RAD = 0.6f;
static constexpr float VEL_FAIL_RAD_S = 30.0f;
// Temporary drift/vibration test mode:
// - true: send constant torque after tare (no LQR command)
// - false: normal balance control law
static constexpr bool BALANCE_FIXED_TORQUE_TEST = false;
// Set to +1.0f or -1.0f to choose spin direction for the vibration test.
static constexpr float BALANCE_FIXED_TORQUE_NM = 1.0f;
static constexpr uint32_t IMU_TIMEOUT_US = 20000u;
static constexpr uint32_t ESC_TIMEOUT_US = 20000u;
static constexpr uint16_t THETA_EQ_SAMPLES = 200u;
// CAN reliability improvement baseline:
// Keep balance-mode IQREQ cadence aligned with stable 500 Hz command profile.
static constexpr uint32_t BALANCE_TX_PERIOD_US = 2000u; // 500 Hz
static constexpr uint32_t BALANCE_LOG_SECONDS = 10u;
static constexpr uint32_t BALANCE_LOG_HZ = 250u;
static constexpr uint32_t BALANCE_LOG_CAPACITY = BALANCE_LOG_SECONDS * BALANCE_LOG_HZ;
static constexpr float TARE_IMU_RATE_MAX_RAD_S = 0.12f;
static constexpr float TARE_IMU_RAW_RATE_MAX_RAD_S = 0.20f;
static constexpr float TARE_ACCEL_ERR_MAX_G = 0.03f;
static constexpr float TARE_ESC_VEL_MAX_RAD_S = 0.6f;
static constexpr uint16_t TARE_SETTLE_SAMPLES = 400u; // 400 ms at 1 kHz
static constexpr float TARE_RATE_BIAS_MAX_RAD_S = 0.35f;
static constexpr uint32_t TARE_WAIT_PRINT_US = 500000u;

static constexpr float ZERO_CROSS_HYST_RAD = 0.01f;
static constexpr uint32_t ZERO_CROSS_MIN_INTERVAL_US = 120000u;
static constexpr uint32_t ZERO_CROSS_ARM_DELAY_US = 2000000u;
static constexpr float RC_CMD_LPF_HZ = 6.0f;
static constexpr float RC_THROTTLE_THETA_REF_MAX_RAD = 0.10f;
static constexpr float RC_STEER_TORQUE_MAX_NM = 0.35f;
// Virtual axle synchronization terms to reduce heading arc/drift.
// Isolation test: keep disabled while investigating slow equilibrium drift.
static constexpr float K_YAW_P = 0.0f; // Torque per radian of heading error
static constexpr float K_YAW_D = 0.0f; // Damping for yaw-rate error

// ESC POS telemetry plausibility guard (expects wrapped radians in [0, 2*pi)).
static constexpr float POS_RAW_MIN_RAD = -0.5f;
static constexpr float POS_RAW_MAX_RAD = (2.0f * M_PI) + 0.5f;
static constexpr float POS_STEP_MAX_RAD = 0.5f;

static bool g_first_entry = true;
static uint32_t g_start_us = 0u;
static uint32_t g_last_tx_us = 0u;
static uint32_t g_tx_attempts = 0u;
static uint32_t g_tx_ok = 0u;
static uint32_t g_tx_fail = 0u;
static uint32_t g_last_tare_wait_print_us = 0u;
static int8_t g_theta_sign_state = 0;
static uint32_t g_last_zero_cross_us = 0u;
static uint32_t g_zero_cross_arm_us = 0u;

static float g_theta_eq = 0.0f;
static float g_theta_eq_accum = 0.0f;
static float g_theta_dot_bias = 0.0f;
static float g_theta_dot_bias_accum = 0.0f;
static uint16_t g_theta_eq_count = 0u;
static uint16_t g_tare_stable_count = 0u;

static bool g_unwrap_init = false;
static float g_prev_l = 0.0f;
static float g_prev_r = 0.0f;
static float g_unwrap_l = 0.0f;
static float g_unwrap_r = 0.0f;
static float g_vel_filt_l = 0.0f;
static float g_vel_filt_r = 0.0f;
static float g_yaw_ref = 0.0f;

static float g_u_cmd_hold = 0.0f;
static float g_x_ref_m = 0.0f;
static float g_x_int_m_s = 0.0f;
static bool g_x_ref_init = false;
static uint32_t g_x_recenter_stable_since_us = 0u;
static float g_last_d_l_rad = 0.0f;
static float g_last_d_r_rad = 0.0f;
static bool g_pos_range_ok = true;
static bool g_pos_step_ok = true;
static float g_rc_throttle_cmd_filt = 0.0f;
static float g_rc_steer_cmd_filt = 0.0f;

struct BalanceDiagEntry {
  uint32_t t_us;
  uint32_t elapsed_us;
  uint32_t dt_us;
  uint32_t tx_period_us;
  float pitch_raw_rad;
  float pitch_rate_raw;
  float theta_eq_rad;
  float theta_tared_rad;
  float theta_dot;
  float theta_dot_bias;
  float imu_acc_mag_g;
  uint8_t imu_accel_valid;
  float imu_int_fb_y;
  float x_m;
  float x_dot;
  float x_err_m;
  float u_theta;
  float u_x;
  float u_i;
  float u_unsat;
  float u_cmd;
  float pos_l_raw;
  float pos_r_raw;
  float vel_l_raw;
  float vel_r_raw;
  float rc_throttle_cmd;
  float rc_steer_cmd;
  float theta_ref_rad;
  float tau_l_cmd;
  float tau_r_cmd;
  uint16_t tare_eq_count;
  uint16_t tare_stable_count;
};

static BalanceDiagEntry g_diag_ring[BALANCE_LOG_CAPACITY];
static uint16_t g_diag_head = 0u;
static uint16_t g_diag_size = 0u;
static uint32_t g_diag_total = 0u;
static bool g_diag_dumped = false;
static uint32_t g_trace_id = 0u;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float apply_deadband(float x, float deadband) {
  const float d = clampf(deadband, 0.0f, 0.49f);
  if (fabsf(x) <= d) return 0.0f;
  const float sign = (x >= 0.0f) ? 1.0f : -1.0f;
  return sign * ((fabsf(x) - d) / (1.0f - d));
}

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

static void print_balance_can_summary(const Supervisor_typedef *sup,
                                      uint32_t now_us,
                                      uint32_t tx_period_us,
                                      bool tx_enabled) {
  print_min_ingress_seq_summary(sup);
  print_min_rx_summary(sup, now_us);
  print_min_can_events_summary();
  Serial.printf(
      "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"tx_enable\":%d,\"tx_period_us\":%lu}\r\n",
      (unsigned long)g_tx_attempts,
      (unsigned long)g_tx_ok,
      (unsigned long)g_tx_fail,
      tx_enabled ? 1 : 0,
      (unsigned long)tx_period_us);
}

static inline void diag_reset() {
  g_diag_head = 0u;
  g_diag_size = 0u;
  g_diag_total = 0u;
  g_diag_dumped = false;
}

static inline void diag_push(const BalanceDiagEntry &e) {
  g_diag_ring[g_diag_head] = e;
  g_diag_head = (uint16_t)((g_diag_head + 1u) % BALANCE_LOG_CAPACITY);
  if (g_diag_size < BALANCE_LOG_CAPACITY) {
    g_diag_size++;
  }
  if (g_diag_total < UINT32_MAX) {
    g_diag_total++;
  }
}

static void diag_dump(const char *reason) {
  if (g_diag_dumped) return;
  g_trace_id++;

  const char *r = (reason != nullptr) ? reason : "run_end";
  const uint32_t dropped = (g_diag_total > g_diag_size) ? (g_diag_total - g_diag_size) : 0u;
  (void)r;
  (void)dropped;
  // Temporarily disabled to focus serial output on CAN health summaries.
  // Serial.printf(
  //     "{\"cmd\":\"BALANCE_TRACE_BEGIN\",\"trace_id\":%lu,\"reason\":\"%s\",\"samples\":%u,\"capacity\":%u,\"total_seen\":%lu,\"dropped\":%lu}\r\n",
  //     (unsigned long)g_trace_id,
  //     r,
  //     (unsigned int)g_diag_size,
  //     (unsigned int)BALANCE_LOG_CAPACITY,
  //     (unsigned long)g_diag_total,
  //     (unsigned long)dropped);

  // Per-sample BALANCE_TRACE rows are intentionally disabled for now.
  // Temporarily disabled to focus serial output on CAN health summaries.
  // Serial.printf("{\"cmd\":\"BALANCE_TRACE_END\",\"trace_id\":%lu,\"reason\":\"%s\",\"samples\":%u}\r\n",
  //               (unsigned long)g_trace_id,
  //               r,
  //               (unsigned int)g_diag_size);
  g_diag_dumped = true;
}

static inline void reset_balance_state() {
  g_first_entry = true;
  g_start_us = 0u;
  g_last_tx_us = 0u;
  g_tx_attempts = 0u;
  g_tx_ok = 0u;
  g_tx_fail = 0u;
  g_last_tare_wait_print_us = 0u;

  g_theta_eq = 0.0f;
  g_theta_sign_state = 0;
  g_last_zero_cross_us = 0u;
  g_zero_cross_arm_us = 0u;
  g_theta_eq_accum = 0.0f;
  g_theta_dot_bias = 0.0f;
  g_theta_dot_bias_accum = 0.0f;
  g_theta_eq_count = 0u;
  g_tare_stable_count = 0u;

  g_unwrap_init = false;
  g_prev_l = 0.0f;
  g_prev_r = 0.0f;
  g_unwrap_l = 0.0f;
  g_unwrap_r = 0.0f;
  g_vel_filt_l = 0.0f;
  g_vel_filt_r = 0.0f;
  g_yaw_ref = 0.0f;

  g_u_cmd_hold = 0.0f;
  g_x_ref_m = 0.0f;
  g_x_int_m_s = 0.0f;
  g_x_ref_init = false;
  g_x_recenter_stable_since_us = 0u;
  g_last_d_l_rad = 0.0f;
  g_last_d_r_rad = 0.0f;
  g_pos_range_ok = true;
  g_pos_step_ok = true;
  g_rc_throttle_cmd_filt = 0.0f;
  g_rc_steer_cmd_filt = 0.0f;
  diag_reset();
}

static bool send_iqreq_checked(float torque_nm,
                               uint8_t node_id,
                               FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                               FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  const float torque_cmd_nm = (SEND_TORQUE != 0) ? torque_nm : 0.0f;
  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, node_id);
  msg.len = 8;
  msg.flags.extended = 1;
  canPackFloat(torque_cmd_nm, msg.buf);
  canPackFloat(0.0f, msg.buf + 4);

  const uint8_t tx_bus = can_tx_bus_for_node(node_id);
  const bool ok1 = (tx_bus == CAN_TX_BUS_CAN1 || tx_bus == CAN_TX_BUS_BOTH)
                     ? can1.write(msg)
                     : false;
  const bool ok2 = (tx_bus == CAN_TX_BUS_CAN2 || tx_bus == CAN_TX_BUS_BOTH)
                     ? can2.write(msg)
                     : false;
  return ok1 || ok2;
}

static inline void send_zero_and_idle(Supervisor_typedef *sup,
                                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                                      FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2,
                                      uint32_t now_us,
                                      uint32_t tx_period_us,
                                      bool tx_enabled) {
  const uint16_t esc_n = (sup->esc_count >= 2u) ? 2u : sup->esc_count;
  for (uint16_t i = 0u; i < esc_n; ++i) {
    const bool ok = send_iqreq_checked(0.0f, sup->esc[i].config.node_id, can1, can2);
    g_tx_attempts++;
    g_tx_ok += ok ? 1u : 0u;
    g_tx_fail += ok ? 0u : 1u;
  }
  print_balance_can_summary(sup, now_us, tx_period_us, tx_enabled);
  sup->mode = SUP_MODE_IDLE;
  reset_balance_state();
}

static void update_wheel_unwrap(float pos_l_raw, float pos_r_raw,
                                float vel_l, float vel_r,
                                float dt_s,
                                float *x_wheel_m,
                                float *x_dot_m_s) {
  if (!g_unwrap_init) {
    g_prev_l = pos_l_raw;
    g_prev_r = pos_r_raw;
    g_unwrap_l = 0.0f;
    g_unwrap_r = 0.0f;
    g_vel_filt_l = LEFT_WHEEL_SIGN * vel_l;
    g_vel_filt_r = RIGHT_WHEEL_SIGN * vel_r;
    g_unwrap_init = true;
  }

  float d_l = pos_l_raw - g_prev_l;
  float d_r = pos_r_raw - g_prev_r;
  if (d_l > M_PI) d_l -= 2.0f * M_PI;
  if (d_l < -M_PI) d_l += 2.0f * M_PI;
  if (d_r > M_PI) d_r -= 2.0f * M_PI;
  if (d_r < -M_PI) d_r += 2.0f * M_PI;

  d_l *= LEFT_WHEEL_SIGN;
  d_r *= RIGHT_WHEEL_SIGN;
  g_last_d_l_rad = d_l;
  g_last_d_r_rad = d_r;

  g_pos_range_ok = (pos_l_raw >= POS_RAW_MIN_RAD && pos_l_raw <= POS_RAW_MAX_RAD &&
                    pos_r_raw >= POS_RAW_MIN_RAD && pos_r_raw <= POS_RAW_MAX_RAD);
  g_pos_step_ok = (fabsf(d_l) <= POS_STEP_MAX_RAD && fabsf(d_r) <= POS_STEP_MAX_RAD);

  g_unwrap_l += d_l;
  g_unwrap_r += d_r;
  g_prev_l = pos_l_raw;
  g_prev_r = pos_r_raw;

  const float vel_l_common = LEFT_WHEEL_SIGN * vel_l;
  const float vel_r_common = RIGHT_WHEEL_SIGN * vel_r;

  // Light LPF (~20 Hz) matching earlier working framework.
  const float fc_hz = 20.0f;
  const float rc = 1.0f / (2.0f * PI * fc_hz);
  const float alpha = dt_s / (dt_s + rc);
  g_vel_filt_l += alpha * (vel_l_common - g_vel_filt_l);
  g_vel_filt_r += alpha * (vel_r_common - g_vel_filt_r);

  const float x_wheel_rad = 0.5f * (g_unwrap_l + g_unwrap_r);
  const float x_dot_rad_s = 0.5f * (g_vel_filt_l + g_vel_filt_r);
  *x_wheel_m = x_wheel_rad * WHEEL_RADIUS_M;
  *x_dot_m_s = x_dot_rad_s * WHEEL_RADIUS_M;
}

void balance_debug_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                      FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  if (sup == nullptr) return;
  if (sup->esc_count < 2u) {
    send_zero_and_idle(sup, can1, can2, micros(), BALANCE_TX_PERIOD_US, sup->user_tx_enable);
    return;
  }

  const uint32_t now_us = micros();
  const float dt_s_raw = (float)sup->timing.dt_us * 1.0e-6f;
  const float dt_s = (dt_s_raw >= 0.0005f && dt_s_raw <= 0.01f) ? dt_s_raw : 0.001f;

  if (g_first_entry) {
    g_first_entry = false;
    g_start_us = now_us;
    g_last_tx_us = 0u;
    g_last_tare_wait_print_us = 0u;
    g_theta_eq = 0.0f;
    g_theta_sign_state = 0;
    g_last_zero_cross_us = 0u;
    g_zero_cross_arm_us = 0u;
    g_theta_eq_accum = 0.0f;
    g_theta_dot_bias = 0.0f;
    g_theta_dot_bias_accum = 0.0f;
    g_theta_eq_count = 0u;
    g_tare_stable_count = 0u;
    g_unwrap_init = false;
    g_yaw_ref = 0.0f;
    g_u_cmd_hold = 0.0f;
    g_x_ref_m = 0.0f;
    g_x_int_m_s = 0.0f;
    g_x_ref_init = false;
    g_x_recenter_stable_since_us = 0u;
    diag_reset();
    g_tx_attempts = 0u;
    g_tx_ok = 0u;
    g_tx_fail = 0u;

    Serial.printf("{\"cmd\":\"BALANCE_START\",\"send_torque\":%d,\"fixed_torque_test\":%d,\"fixed_torque_nm\":%.3f}\r\n",
                  (int)SEND_TORQUE,
                  BALANCE_FIXED_TORQUE_TEST ? 1 : 0,
                  BALANCE_FIXED_TORQUE_NM);
  }

  const bool imu_fresh = sup->imu.valid &&
                         ((uint32_t)(now_us - sup->imu.last_update_us) <= IMU_TIMEOUT_US);
  const bool esc_l_fresh = sup->esc[0].state.alive &&
                           ((uint32_t)(now_us - sup->esc[0].status.last_update_us) <= ESC_TIMEOUT_US);
  const bool esc_r_fresh = sup->esc[1].state.alive &&
                           ((uint32_t)(now_us - sup->esc[1].status.last_update_us) <= ESC_TIMEOUT_US);
  if (!imu_fresh || !esc_l_fresh || !esc_r_fresh) {
    Serial.printf("{\"cmd\":\"BALANCE_ABORT\",\"reason\":\"freshness\",\"imu_fresh\":%d,\"esc_l_fresh\":%d,\"esc_r_fresh\":%d}\r\n",
                  imu_fresh ? 1 : 0, esc_l_fresh ? 1 : 0, esc_r_fresh ? 1 : 0);
    diag_dump("freshness");
    const uint32_t tx_period_us_fallback =
        (sup->user_tx_period_us > 0u) ? sup->user_tx_period_us : BALANCE_TX_PERIOD_US;
    send_zero_and_idle(sup, can1, can2, now_us, tx_period_us_fallback, sup->user_tx_enable);
    return;
  }

  const uint32_t tx_period_us =
      (sup->user_tx_period_us > 0u) ? sup->user_tx_period_us : BALANCE_TX_PERIOD_US;
  const bool tx_enabled = sup->user_tx_enable && (SEND_TORQUE != 0);
  const bool tx_due = (g_last_tx_us == 0u) ||
                      ((uint32_t)(now_us - g_last_tx_us) >= tx_period_us);

  if (g_theta_eq_count < THETA_EQ_SAMPLES) {
    const float acc_err_g = fabsf(sup->imu.accel_mag_g - 1.0f);
    const bool tare_stable = (sup->imu.accel_valid == 1u) &&
                             (acc_err_g <= TARE_ACCEL_ERR_MAX_G) &&
                             (fabsf(sup->imu.pitch_rate_raw) <= TARE_IMU_RAW_RATE_MAX_RAD_S) &&
                             (fabsf(sup->imu.pitch_rate) <= TARE_IMU_RATE_MAX_RAD_S) &&
                             (fabsf(sup->esc[0].state.vel_rad_s) <= TARE_ESC_VEL_MAX_RAD_S) &&
                             (fabsf(sup->esc[1].state.vel_rad_s) <= TARE_ESC_VEL_MAX_RAD_S);
    const bool settle_done = (g_tare_stable_count >= TARE_SETTLE_SAMPLES);

    if (!tare_stable) {
      g_tare_stable_count = 0u;
      g_theta_eq_count = 0u;
      g_theta_eq_accum = 0.0f;
      g_theta_dot_bias_accum = 0.0f;
    } else if (!settle_done) {
      g_tare_stable_count++;
    } else {
      g_theta_eq_accum += sup->imu.pitch_rad;
      g_theta_dot_bias_accum += sup->imu.pitch_rate;
      g_theta_eq_count++;

      if (g_theta_eq_count == THETA_EQ_SAMPLES) {
        g_theta_eq = g_theta_eq_accum / (float)THETA_EQ_SAMPLES;
        g_theta_dot_bias = g_theta_dot_bias_accum / (float)THETA_EQ_SAMPLES;

        if (fabsf(g_theta_dot_bias) > TARE_RATE_BIAS_MAX_RAD_S) {
          Serial.printf(
              "{\"cmd\":\"BALANCE_TARE_RETRY\",\"reason\":\"rate_bias\",\"theta_dot_bias_rad_s\":%.6f}\r\n",
              g_theta_dot_bias);
          g_theta_eq_count = 0u;
          g_theta_eq_accum = 0.0f;
          g_theta_dot_bias_accum = 0.0f;
          g_tare_stable_count = 0u;
        } else {
          g_unwrap_init = false;
          g_last_tx_us = 0u;
          g_theta_sign_state = 0;
          g_last_zero_cross_us = now_us;
          g_zero_cross_arm_us = now_us + ZERO_CROSS_ARM_DELAY_US;
          g_yaw_ref = g_unwrap_l - g_unwrap_r;
          Serial.printf("{\"cmd\":\"BALANCE_TARE_DONE\",\"theta_eq_rad\":%.6f,\"theta_dot_bias_rad_s\":%.6f}\r\n",
                        g_theta_eq, g_theta_dot_bias);
        }
      }
    }

    if ((g_last_tare_wait_print_us == 0u) ||
        ((uint32_t)(now_us - g_last_tare_wait_print_us) >= TARE_WAIT_PRINT_US)) {
      g_last_tare_wait_print_us = now_us;
      Serial.printf(
          "{\"cmd\":\"BALANCE_TARE_WAIT\",\"tare_stable\":%d,\"settle_count\":%u,\"settle_target\":%u,\"eq_count\":%u,\"eq_target\":%u,\"imu_pitch_rate\":%.6f,\"imu_pitch_rate_raw\":%.6f,\"imu_acc_mag_g\":%.6f,\"imu_acc_err_g\":%.6f,\"vel_l\":%.6f,\"vel_r\":%.6f}\r\n",
          tare_stable ? 1 : 0,
          (unsigned int)g_tare_stable_count,
          (unsigned int)TARE_SETTLE_SAMPLES,
          (unsigned int)g_theta_eq_count,
          (unsigned int)THETA_EQ_SAMPLES,
          sup->imu.pitch_rate,
          sup->imu.pitch_rate_raw,
          sup->imu.accel_mag_g,
          acc_err_g,
          sup->esc[0].state.vel_rad_s,
          sup->esc[1].state.vel_rad_s);
    }

    if (tx_due) {
      g_last_tx_us = now_us;
      const bool ok_l = send_iqreq_checked(0.0f, sup->esc[0].config.node_id, can1, can2);
      const bool ok_r = send_iqreq_checked(0.0f, sup->esc[1].config.node_id, can1, can2);
      g_tx_attempts += 2u;
      g_tx_ok += (ok_l ? 1u : 0u) + (ok_r ? 1u : 0u);
      g_tx_fail += (ok_l ? 0u : 1u) + (ok_r ? 0u : 1u);
      g_u_cmd_hold = 0.0f;
    }
    return;
  }

  const float theta = sup->imu.pitch_rad - g_theta_eq;
  const float theta_dot = sup->imu.pitch_rate - g_theta_dot_bias;
  const bool hold_mode = (sup->mode == SUP_MODE_BALANCE_HOLD);

  float rc_throttle_cmd = 0.0f;
  float rc_steer_cmd = 0.0f;
  bool rc_throttle_valid = false;
  bool rc_steer_valid = false;
  if (!hold_mode) {
    if (sup->user_rc_throttle_ch < sup->rc_count && sup->rc[sup->user_rc_throttle_ch].valid) {
      rc_throttle_valid = true;
      rc_throttle_cmd = clampf(sup->rc[sup->user_rc_throttle_ch].norm, -1.0f, 1.0f);
      if (sup->user_rc_throttle_invert) rc_throttle_cmd = -rc_throttle_cmd;
      rc_throttle_cmd = apply_deadband(rc_throttle_cmd, sup->user_rc_deadband);
    }
    if (sup->user_rc_steer_ch < sup->rc_count && sup->rc[sup->user_rc_steer_ch].valid) {
      rc_steer_valid = true;
      rc_steer_cmd = clampf(sup->rc[sup->user_rc_steer_ch].norm, -1.0f, 1.0f);
      if (sup->user_rc_steer_invert) rc_steer_cmd = -rc_steer_cmd;
      rc_steer_cmd = apply_deadband(rc_steer_cmd, sup->user_rc_deadband);
    }
    const float rc_alpha = dt_s / (dt_s + (1.0f / (2.0f * PI * RC_CMD_LPF_HZ)));
    g_rc_throttle_cmd_filt += rc_alpha * (rc_throttle_cmd - g_rc_throttle_cmd_filt);
    g_rc_steer_cmd_filt += rc_alpha * (rc_steer_cmd - g_rc_steer_cmd_filt);
    if (!rc_throttle_valid) g_rc_throttle_cmd_filt *= 0.95f;
    if (!rc_steer_valid) g_rc_steer_cmd_filt *= 0.95f;
  } else {
    // Hold mode explicitly ignores RC motion shaping.
    g_rc_throttle_cmd_filt = 0.0f;
    g_rc_steer_cmd_filt = 0.0f;
  }
  const float theta_ref = hold_mode ? 0.0f : (RC_THROTTLE_THETA_REF_MAX_RAD * g_rc_throttle_cmd_filt);
  const float theta_err = theta - theta_ref;

  int8_t theta_sign = 0;
  if (theta_err > ZERO_CROSS_HYST_RAD) theta_sign = 1;
  else if (theta_err < -ZERO_CROSS_HYST_RAD) theta_sign = -1;

  if (theta_sign != 0) {
    if (g_theta_sign_state != 0 && theta_sign != g_theta_sign_state) {
      if (now_us >= g_zero_cross_arm_us &&
          (uint32_t)(now_us - g_last_zero_cross_us) >= ZERO_CROSS_MIN_INTERVAL_US) {
        balance_zero_cross_tweet();
        g_last_zero_cross_us = now_us;
      }
    }
    g_theta_sign_state = theta_sign;
  }
  const float pos_l = sup->esc[0].state.pos_rad;
  const float pos_r = sup->esc[1].state.pos_rad;
  const float vel_l = sup->esc[0].state.vel_rad_s;
  const float vel_r = sup->esc[1].state.vel_rad_s;

  float x_wheel_m = 0.0f;
  float x_dot_m_s = 0.0f;
  update_wheel_unwrap(pos_l, pos_r, vel_l, vel_r, dt_s, &x_wheel_m, &x_dot_m_s);
  float yaw_error = (g_unwrap_l - g_unwrap_r) - g_yaw_ref; // Relative to tare heading
  float yaw_rate_error = g_vel_filt_l - g_vel_filt_r; // Difference in wheel speed
  float u_sync = (K_YAW_P * yaw_error) + (K_YAW_D * yaw_rate_error);
  if (!g_x_ref_init) {
    // Lock position reference at balance start so "hold position" means
    // return to this wheel-center location after disturbances.
    g_x_ref_m = x_wheel_m;
    g_x_ref_init = true;
  }
  // Position-hold error: positive when wheel center is behind reference.
  // Using (ref - measured) here sets the x-channel control direction
  // explicitly for restoring motion toward the locked reference.
  const float x_pos_err_m = g_x_ref_m - x_wheel_m;
  const float x_dot_err_m_s = -x_dot_m_s;

  // In hold mode, allow a very slow x_ref recenter only after prolonged
  // upright stability. This trims steady bias without fighting balance.
  if (X_REF_RECENTER_ENABLE && hold_mode) {
    const bool recenter_stable =
        (fabsf(theta_err) <= X_REF_RECENTER_THETA_RAD) &&
        (fabsf(theta_dot) <= X_REF_RECENTER_THETA_DOT_RAD_S) &&
        (fabsf(x_dot_m_s) <= X_REF_RECENTER_XDOT_M_S);
    if (recenter_stable) {
      if (g_x_recenter_stable_since_us == 0u) {
        g_x_recenter_stable_since_us = now_us;
      } else if ((uint32_t)(now_us - g_x_recenter_stable_since_us) >= X_REF_RECENTER_STABLE_US) {
        const float alpha = dt_s / (X_REF_RECENTER_TAU_S + dt_s);
        g_x_ref_m += alpha * (x_wheel_m - g_x_ref_m);
      }
    } else {
      g_x_recenter_stable_since_us = 0u;
    }
  } else {
    g_x_recenter_stable_since_us = 0u;
  }

  if (!BALANCE_FIXED_TORQUE_TEST && (!g_pos_range_ok || !g_pos_step_ok)) {
    Serial.printf(
        "{\"cmd\":\"BALANCE_ABORT\",\"reason\":\"pos_units\",\"pos_l_raw\":%.6f,\"pos_r_raw\":%.6f,\"d_l_rad\":%.6f,\"d_r_rad\":%.6f,\"range_ok\":%d,\"step_ok\":%d}\r\n",
        pos_l, pos_r, g_last_d_l_rad, g_last_d_r_rad, g_pos_range_ok ? 1 : 0, g_pos_step_ok ? 1 : 0);
    diag_dump("pos_units");
    send_zero_and_idle(sup, can1, can2, now_us, tx_period_us, tx_enabled);
    return;
  }

  if (!BALANCE_FIXED_TORQUE_TEST &&
      (fabsf(theta) > THETA_FAIL_RAD || fabsf(vel_l) > VEL_FAIL_RAD_S || fabsf(vel_r) > VEL_FAIL_RAD_S)) {
    Serial.printf("{\"cmd\":\"BALANCE_ABORT\",\"reason\":\"limits\",\"theta\":%.4f,\"vel_l\":%.3f,\"vel_r\":%.3f}\r\n",
                  theta, vel_l, vel_r);
    diag_dump("limits");
    send_zero_and_idle(sup, can1, can2, now_us, tx_period_us, tx_enabled);
    return;
  }

  // Full-state control effort:
  // - theta/theta_dot terms stabilize body tilt.
  // - x_err/x_dot_err terms provide position-hold so the wheel pair returns
  //   toward its centered location instead of drifting while "upright".
  // - slow integral on x_pos_err rejects small steady-state bias torques.
  // Tilt-priority gate: disable x channel when tilt recovery is active.
  const bool x_hold_enabled =
      (fabsf(theta_err) <= X_HOLD_ENABLE_THETA_RAD) &&
      (fabsf(theta_dot) <= X_HOLD_ENABLE_THETA_DOT_RAD_S) &&
      (fabsf(x_dot_m_s) <= X_HOLD_ENABLE_X_DOT_M_S);

  const float k_x = BALANCE_TARE_VALIDATION_MODE ? 0.0f : K_DISC[2];
  const float k_x_dot = BALANCE_TARE_VALIDATION_MODE ? 0.0f : K_DISC[3];
  const float pos_hold_ki = BALANCE_TARE_VALIDATION_MODE ? 0.0f : POS_HOLD_KI;

  if (pos_hold_ki > 0.0f) {
    if (x_hold_enabled) {
      g_x_int_m_s += x_pos_err_m * dt_s;
      const float x_int_lim = POS_HOLD_I_CLAMP_NM / pos_hold_ki;
      g_x_int_m_s = clampf(g_x_int_m_s, -x_int_lim, x_int_lim);
    } else {
      g_x_int_m_s *= X_HOLD_INT_LEAK_PER_STEP;
    }
  } else {
    g_x_int_m_s = 0.0f;
  }

  const float u_x_i = x_hold_enabled
                          ? clampf(pos_hold_ki * g_x_int_m_s,
                                   -POS_HOLD_I_CLAMP_NM, POS_HOLD_I_CLAMP_NM)
                          : 0.0f;
  const float u_theta = -(K_DISC[0] * theta_err) * SAFETY_SCALE;
  const float u_theta_dot = -(K_DISC[1] * theta_dot) * SAFETY_SCALE;
  const float u_x = x_hold_enabled ? (-(k_x * x_pos_err_m) * SAFETY_SCALE) : 0.0f;
  const float u_x_dot = x_hold_enabled ? (-(k_x_dot * x_dot_err_m_s) * SAFETY_SCALE) : 0.0f;
  const float u_i = -(u_x_i) * SAFETY_SCALE;
  const float u_balance = u_theta + u_theta_dot + u_x + u_x_dot + u_i;
  const float u_model = BALANCE_FIXED_TORQUE_TEST
                            ? BALANCE_FIXED_TORQUE_NM
                            : u_balance;
  const float u_steer = hold_mode ? 0.0f : (RC_STEER_TORQUE_MAX_NM * g_rc_steer_cmd_filt);

  if (tx_due) {
    float u_cmd = clampf(u_model, -TORQUE_CLAMP_NM, TORQUE_CLAMP_NM);
    float tau_l_cmd = clampf(-u_cmd + u_steer - u_sync, -TORQUE_CLAMP_NM, TORQUE_CLAMP_NM);
    float tau_r_cmd = clampf( u_cmd + u_steer + u_sync, -TORQUE_CLAMP_NM, TORQUE_CLAMP_NM);
    g_u_cmd_hold = u_cmd;
    g_last_tx_us = now_us;
    if (tx_enabled) {
      const bool ok_l = send_iqreq_checked(tau_l_cmd, sup->esc[0].config.node_id, can1, can2);
      const bool ok_r = send_iqreq_checked(tau_r_cmd, sup->esc[1].config.node_id, can1, can2);
      g_tx_attempts += 2u;
      g_tx_ok += (ok_l ? 1u : 0u) + (ok_r ? 1u : 0u);
      g_tx_fail += (ok_l ? 0u : 1u) + (ok_r ? 0u : 1u);
    } else {
      tau_l_cmd = 0.0f;
      tau_r_cmd = 0.0f;
      const bool ok_l = send_iqreq_checked(0.0f, sup->esc[0].config.node_id, can1, can2);
      const bool ok_r = send_iqreq_checked(0.0f, sup->esc[1].config.node_id, can1, can2);
      g_tx_attempts += 2u;
      g_tx_ok += (ok_l ? 1u : 0u) + (ok_r ? 1u : 0u);
      g_tx_fail += (ok_l ? 0u : 1u) + (ok_r ? 0u : 1u);
    }

    BalanceDiagEntry e{};
    e.t_us = now_us;
    e.elapsed_us = now_us - g_start_us;
    e.dt_us = sup->timing.dt_us;
    e.tx_period_us = tx_period_us;
    e.pitch_raw_rad = sup->imu.pitch_rad;
    e.pitch_rate_raw = sup->imu.pitch_rate_raw;
    e.theta_eq_rad = g_theta_eq;
    e.theta_tared_rad = theta;
    e.theta_dot = theta_dot;
    e.theta_dot_bias = g_theta_dot_bias;
    e.imu_acc_mag_g = sup->imu.accel_mag_g;
    e.imu_accel_valid = sup->imu.accel_valid;
    e.imu_int_fb_y = sup->imu.mahony_int_fb_y;
    e.x_m = x_wheel_m;
    e.x_dot = x_dot_m_s;
    e.x_err_m = x_pos_err_m;
    e.u_theta = u_theta;
    e.u_x = u_x;
    e.u_i = u_i;
    e.u_unsat = u_model;
    e.u_cmd = g_u_cmd_hold;
    e.pos_l_raw = pos_l;
    e.pos_r_raw = pos_r;
    e.vel_l_raw = vel_l;
    e.vel_r_raw = vel_r;
    e.rc_throttle_cmd = g_rc_throttle_cmd_filt;
    e.rc_steer_cmd = g_rc_steer_cmd_filt;
    e.theta_ref_rad = theta_ref;
    e.tau_l_cmd = tau_l_cmd;
    e.tau_r_cmd = tau_r_cmd;
    e.tare_eq_count = g_theta_eq_count;
    e.tare_stable_count = g_tare_stable_count;
    diag_push(e);
  }

  if (sup->user_total_us > 0u) {
    const uint32_t elapsed_us = now_us - g_start_us;
    if (elapsed_us >= sup->user_total_us) {
      Serial.printf("{\"cmd\":\"BALANCE_DONE\",\"elapsed_us\":%lu,\"limit_us\":%lu}\r\n",
                    (unsigned long)elapsed_us,
                    (unsigned long)sup->user_total_us);
      diag_dump("time_limit");
      send_zero_and_idle(sup, can1, can2, now_us, tx_period_us, tx_enabled);
      return;
    }
  }
}

void balance_debug_dump_on_mode_exit(const char *reason) {
  if (g_first_entry) return;
  diag_dump((reason != nullptr) ? reason : "mode_exit");
  reset_balance_state();
}
