#include "balance_TWR_mode.h"

#include "CAN_helper.h"
#include "main.h"
#include "telemetry_link.h"

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
// CAN reliability improvement baseline:
// Keep balance-mode IQREQ cadence aligned with stable 500 Hz command profile.
static constexpr uint32_t BALANCE_TX_PERIOD_US = 2000u; // 500 Hz

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
// "Stable on kickstand" proxy bit for telemetry consumers.
static constexpr float KICKSTAND_STABLE_THETA_ERR_RAD = 0.03f;
static constexpr float KICKSTAND_STABLE_THETA_DOT_RAD_S = 0.15f;
static constexpr float KICKSTAND_STABLE_WHEEL_VEL_RAD_S = 1.0f;

// ESC POS telemetry plausibility guard (expects wrapped radians in [0, 2*pi)).
static constexpr float POS_RAW_MIN_RAD = -0.5f;
static constexpr float POS_RAW_MAX_RAD = (2.0f * M_PI) + 0.5f;
static constexpr float POS_STEP_MAX_RAD = 0.5f;

static bool g_first_entry = true;
static uint32_t g_start_us = 0u;
static uint32_t g_last_tx_us = 0u;
static int8_t g_theta_sign_state = 0;
static uint32_t g_last_zero_cross_us = 0u;
static uint32_t g_zero_cross_arm_us = 0u;

static float g_theta_eq = 0.0f;
static float g_theta_dot_bias = 0.0f;
static bool g_theta_ref_preloaded = false;
static bool g_is_tared = false;

static bool g_unwrap_init = false;
static float g_prev_l = 0.0f;
static float g_prev_r = 0.0f;
static float g_unwrap_l = 0.0f;
static float g_unwrap_r = 0.0f;
static float g_vel_filt_l = 0.0f;
static float g_vel_filt_r = 0.0f;
static float g_yaw_ref = 0.0f;

static float g_x_ref_m = 0.0f;
static float g_x_int_m_s = 0.0f;
static bool g_x_ref_init = false;
static uint32_t g_x_recenter_stable_since_us = 0u;
static bool g_pos_range_ok = true;
static bool g_pos_step_ok = true;
static float g_rc_throttle_cmd_filt = 0.0f;
static float g_rc_steer_cmd_filt = 0.0f;

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

static inline void reset_balance_state() {
  g_first_entry = true;
  g_start_us = 0u;
  g_last_tx_us = 0u;

  g_theta_eq = 0.0f;
  g_theta_sign_state = 0;
  g_last_zero_cross_us = 0u;
  g_zero_cross_arm_us = 0u;
  g_theta_dot_bias = 0.0f;
  g_theta_ref_preloaded = false;
  g_is_tared = false;

  g_unwrap_init = false;
  g_prev_l = 0.0f;
  g_prev_r = 0.0f;
  g_unwrap_l = 0.0f;
  g_unwrap_r = 0.0f;
  g_vel_filt_l = 0.0f;
  g_vel_filt_r = 0.0f;
  g_yaw_ref = 0.0f;

  g_x_ref_m = 0.0f;
  g_x_int_m_s = 0.0f;
  g_x_ref_init = false;
  g_x_recenter_stable_since_us = 0u;
  g_pos_range_ok = true;
  g_pos_step_ok = true;
  g_rc_throttle_cmd_filt = 0.0f;
  g_rc_steer_cmd_filt = 0.0f;
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
  (void)now_us;
  (void)tx_period_us;
  (void)tx_enabled;
  const uint16_t esc_n = (sup->esc_count >= 2u) ? 2u : sup->esc_count;
  for (uint16_t i = 0u; i < esc_n; ++i) {
    (void)send_iqreq_checked(0.0f, sup->esc[i].config.node_id, can1, can2);
  }
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

void balance_TWR_set_theta_reference(float theta_eq_rad, float theta_dot_bias_rad_s) {
  g_theta_eq = theta_eq_rad;
  g_theta_dot_bias = theta_dot_bias_rad_s;
  g_theta_ref_preloaded = true;
  g_is_tared = true;
}

void balance_TWR_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                      FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  if (sup == nullptr) return;
  const uint32_t now_us = micros();
  float dt_s = 0.0f;
  float theta = 0.0f;
  float theta_dot = 0.0f;
  float theta_ref = 0.0f;
  float theta_err = 0.0f;
  float x_wheel_m = 0.0f;
  float x_dot_m_s = 0.0f;
  float x_pos_err_m = 0.0f;
  float tau_l_cmd = 0.0f;
  float tau_r_cmd = 0.0f;
  float u_sync = 0.0f;
  float u_i = 0.0f;
  bool x_hold_enabled = false;
  bool esc_comms_ok = false;
  bool is_stable_on_kickstand = false;
  uint32_t telem_state_bits = 0u;
  uint16_t telem_status = 0u;
  uint16_t telem_event = TELEMETRY_EVENT_NONE;
  auto refresh_telem_state_bits = [&]() {
    telem_state_bits = 0u;
    if (x_hold_enabled) telem_state_bits |= TELEMETRY_STATE_X_HOLD_ENABLED;
    if (g_is_tared) telem_state_bits |= TELEMETRY_STATE_IS_TARED;
    if (esc_comms_ok) telem_state_bits |= TELEMETRY_STATE_ESC_COMMS_OK;
    if (is_stable_on_kickstand) telem_state_bits |= TELEMETRY_STATE_STABLE_ON_KICKSTAND;
  };
  auto publish_telem = [&]() {
    refresh_telem_state_bits();
    BalanceTelemetrySample sample = {};
    sample.timestamp_us = now_us;
    sample.state_bits = telem_state_bits;
    sample.status_flags = telem_status;
    sample.event_code = telem_event;
    sample.dt_s = dt_s;
    sample.theta = theta;
    sample.theta_dot = theta_dot;
    sample.theta_err = theta_err;
    sample.x_wheel_m = x_wheel_m;
    sample.x_dot_m_s = x_dot_m_s;
    sample.x_pos_err_m = x_pos_err_m;
    sample.tau_l_cmd = tau_l_cmd;
    sample.tau_r_cmd = tau_r_cmd;
    sample.u_sync = u_sync;
    sample.u_i = u_i;
    telemetry_publish_balance_tick(sample);
  };

  if (sup->esc_count < 2u) {
    telem_status |= TELEMETRY_STATUS_FAIL_SAFE;
    telem_event = TELEMETRY_EVENT_STALE_ESC_L;
    publish_telem();
    send_zero_and_idle(sup, can1, can2, now_us, BALANCE_TX_PERIOD_US, sup->user_tx_enable);
    return;
  }

  const float dt_s_raw = (float)sup->timing.dt_us * 1.0e-6f;
  dt_s = (dt_s_raw >= 0.0005f && dt_s_raw <= 0.01f) ? dt_s_raw : 0.001f;

  if (g_first_entry) {
    g_first_entry = false;
    g_start_us = now_us;
    g_last_tx_us = 0u;
    g_theta_sign_state = 0;
    g_last_zero_cross_us = 0u;
    g_zero_cross_arm_us = 0u;
    if (!g_theta_ref_preloaded) {
      // Fallback if balance mode is entered without explicit calibration.
      g_theta_eq = sup->imu.pitch_rad;
      g_theta_dot_bias = sup->imu.pitch_rate;
      g_is_tared = false;
    }
    g_theta_ref_preloaded = false;
    g_unwrap_init = false;
    g_yaw_ref = 0.0f;
    g_x_ref_m = 0.0f;
    g_x_int_m_s = 0.0f;
    g_x_ref_init = false;
    g_x_recenter_stable_since_us = 0u;
  }

  const bool imu_fresh = sup->imu.valid &&
                         ((uint32_t)(now_us - sup->imu.last_update_us) <= IMU_TIMEOUT_US);
  const bool esc_l_fresh = sup->esc[0].state.alive &&
                           ((uint32_t)(now_us - sup->esc[0].status.last_update_us) <= ESC_TIMEOUT_US);
  const bool esc_r_fresh = sup->esc[1].state.alive &&
                           ((uint32_t)(now_us - sup->esc[1].status.last_update_us) <= ESC_TIMEOUT_US);
  esc_comms_ok = esc_l_fresh && esc_r_fresh;
  if (imu_fresh) telem_status |= TELEMETRY_STATUS_IMU_FRESH;
  if (esc_l_fresh) telem_status |= TELEMETRY_STATUS_ESC_L_FRESH;
  if (esc_r_fresh) telem_status |= TELEMETRY_STATUS_ESC_R_FRESH;
  if (!imu_fresh || !esc_l_fresh || !esc_r_fresh) {
    telem_status |= TELEMETRY_STATUS_FAIL_SAFE;
    if (!imu_fresh) {
      telem_event = TELEMETRY_EVENT_STALE_IMU;
    } else if (!esc_l_fresh) {
      telem_event = TELEMETRY_EVENT_STALE_ESC_L;
    } else {
      telem_event = TELEMETRY_EVENT_STALE_ESC_R;
    }
    publish_telem();
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
  if (tx_enabled) telem_status |= TELEMETRY_STATUS_TX_ENABLED;
  if (tx_due) telem_status |= TELEMETRY_STATUS_TX_DUE;

  theta = sup->imu.pitch_rad - g_theta_eq;
  theta_dot = sup->imu.pitch_rate - g_theta_dot_bias;
  const bool hold_mode = (sup->mode == SUP_MODE_BALANCE_HOLD);
  if (hold_mode) telem_status |= TELEMETRY_STATUS_HOLD_MODE;

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
  theta_ref = hold_mode ? 0.0f : (RC_THROTTLE_THETA_REF_MAX_RAD * g_rc_throttle_cmd_filt);
  theta_err = theta - theta_ref;

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

  update_wheel_unwrap(pos_l, pos_r, vel_l, vel_r, dt_s, &x_wheel_m, &x_dot_m_s);
  if (g_pos_range_ok) telem_status |= TELEMETRY_STATUS_POS_RANGE_OK;
  if (g_pos_step_ok) telem_status |= TELEMETRY_STATUS_POS_STEP_OK;
  float yaw_error = (g_unwrap_l - g_unwrap_r) - g_yaw_ref; // Relative to tare heading
  float yaw_rate_error = g_vel_filt_l - g_vel_filt_r; // Difference in wheel speed
  u_sync = (K_YAW_P * yaw_error) + (K_YAW_D * yaw_rate_error);
  if (!g_x_ref_init) {
    // Lock position reference at balance start so "hold position" means
    // return to this wheel-center location after disturbances.
    g_x_ref_m = x_wheel_m;
    g_x_ref_init = true;
  }
  // Position-hold error: positive when wheel center is behind reference.
  // Using (ref - measured) here sets the x-channel control direction
  // explicitly for restoring motion toward the locked reference.
  x_pos_err_m = g_x_ref_m - x_wheel_m;
  const float x_dot_err_m_s = -x_dot_m_s;
  is_stable_on_kickstand = hold_mode &&
                           (fabsf(theta_err) <= KICKSTAND_STABLE_THETA_ERR_RAD) &&
                           (fabsf(theta_dot) <= KICKSTAND_STABLE_THETA_DOT_RAD_S) &&
                           (fabsf(vel_l) <= KICKSTAND_STABLE_WHEEL_VEL_RAD_S) &&
                           (fabsf(vel_r) <= KICKSTAND_STABLE_WHEEL_VEL_RAD_S);

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
    telem_status |= TELEMETRY_STATUS_FAIL_SAFE;
    telem_event = g_pos_range_ok ? TELEMETRY_EVENT_POS_STEP_FAIL : TELEMETRY_EVENT_POS_RANGE_FAIL;
    publish_telem();
    send_zero_and_idle(sup, can1, can2, now_us, tx_period_us, tx_enabled);
    return;
  }


  if (!BALANCE_FIXED_TORQUE_TEST &&
      (fabsf(theta) > THETA_FAIL_RAD || fabsf(vel_l) > VEL_FAIL_RAD_S || fabsf(vel_r) > VEL_FAIL_RAD_S)) {
    telem_status |= TELEMETRY_STATUS_FAIL_SAFE;
    telem_event = (fabsf(theta) > THETA_FAIL_RAD) ? TELEMETRY_EVENT_THETA_FAIL : TELEMETRY_EVENT_VEL_FAIL;
    publish_telem();
    send_zero_and_idle(sup, can1, can2, now_us, tx_period_us, tx_enabled);
    return;
  }
 
  
  // Full-state control effort:
  // - theta/theta_dot terms stabilize body tilt.
  // - x_err/x_dot_err terms provide position-hold so the wheel pair returns
  //   toward its centered location instead of drifting while "upright".
  // - slow integral on x_pos_err rejects small steady-state bias torques.
  // Tilt-priority gate: disable x channel when tilt recovery is active.
  x_hold_enabled =
      (fabsf(theta_err) <= X_HOLD_ENABLE_THETA_RAD) &&
      (fabsf(theta_dot) <= X_HOLD_ENABLE_THETA_DOT_RAD_S) &&
      (fabsf(x_dot_m_s) <= X_HOLD_ENABLE_X_DOT_M_S);
  if (x_hold_enabled) telem_status |= TELEMETRY_STATUS_X_HOLD_ENABLED;

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
  u_i = -(u_x_i) * SAFETY_SCALE;
  const float u_balance = u_theta + u_theta_dot + u_x + u_x_dot + u_i;
  const float u_model = BALANCE_FIXED_TORQUE_TEST
                            ? BALANCE_FIXED_TORQUE_NM
                            : u_balance;
  const float u_steer = hold_mode ? 0.0f : (RC_STEER_TORQUE_MAX_NM * g_rc_steer_cmd_filt);
  const float u_cmd = clampf(u_model, -TORQUE_CLAMP_NM, TORQUE_CLAMP_NM);
  tau_l_cmd = clampf(-u_cmd + u_steer - u_sync, -TORQUE_CLAMP_NM, TORQUE_CLAMP_NM);
  tau_r_cmd = clampf( u_cmd + u_steer + u_sync, -TORQUE_CLAMP_NM, TORQUE_CLAMP_NM);

  if (tx_due) {
    g_last_tx_us = now_us;
    if (tx_enabled) {
      (void)send_iqreq_checked(tau_l_cmd, sup->esc[0].config.node_id, can1, can2);
      (void)send_iqreq_checked(tau_r_cmd, sup->esc[1].config.node_id, can1, can2);
    } else {
      tau_l_cmd = 0.0f;
      tau_r_cmd = 0.0f;
      (void)send_iqreq_checked(0.0f, sup->esc[0].config.node_id, can1, can2);
      (void)send_iqreq_checked(0.0f, sup->esc[1].config.node_id, can1, can2);
    }
  }

  if (sup->user_total_us > 0u) {
    const uint32_t elapsed_us = now_us - g_start_us;
    if (elapsed_us >= sup->user_total_us) {
      publish_telem();
      send_zero_and_idle(sup, can1, can2, now_us, tx_period_us, tx_enabled);
      return;
    }
  }

  publish_telem();
}

void balance_TWR_dump_on_mode_exit(const char *reason) {
  (void)reason;
  if (g_first_entry) return;
  reset_balance_state();
}
