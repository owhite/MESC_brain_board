#include "balance_TWR_mode.h"

#include "CAN_helper.h"
#include "main.h"

#include <Arduino.h>
#include <math.h>

#ifndef SEND_TORQUE
#define SEND_TORQUE 1
#endif

// Baseline V1: simple full-state LQR at fixed 250 Hz command cadence.
// State ordering: [theta, theta_dot, x_wheel, x_dot]^T
// Top-down shaped retune:
// - more wheel-position centering authority (Kx)
// - less high-frequency rate aggression (K_theta_dot, K_x_dot)
static constexpr float K_DISC[4] = {
    5.30f,   // K_theta
    0.42f,   // K_theta_dot
    0.0f,    // K_x (isolation test)
    0.0f     // K_x_dot (isolation test)
};

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
static constexpr uint32_t BALANCE_TX_PERIOD_US = 4000u; // 250 Hz
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

// ESC POS telemetry plausibility guard (expects wrapped radians in [0, 2*pi)).
static constexpr float POS_RAW_MIN_RAD = -0.5f;
static constexpr float POS_RAW_MAX_RAD = (2.0f * M_PI) + 0.5f;
static constexpr float POS_STEP_MAX_RAD = 0.5f;

static bool g_first_entry = true;
static uint32_t g_start_us = 0u;
static uint32_t g_last_tx_us = 0u;
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

static float g_u_cmd_hold = 0.0f;
static float g_last_d_l_rad = 0.0f;
static float g_last_d_r_rad = 0.0f;
static bool g_pos_range_ok = true;
static bool g_pos_step_ok = true;

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
  float u_unsat;
  float u_cmd;
  float pos_l_raw;
  float pos_r_raw;
  float vel_l_raw;
  float vel_r_raw;
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
  Serial.printf(
      "{\"cmd\":\"BALANCE_TRACE_BEGIN\",\"trace_id\":%lu,\"reason\":\"%s\",\"samples\":%u,\"capacity\":%u,\"total_seen\":%lu,\"dropped\":%lu}\r\n",
      (unsigned long)g_trace_id,
      r,
      (unsigned int)g_diag_size,
      (unsigned int)BALANCE_LOG_CAPACITY,
      (unsigned long)g_diag_total,
      (unsigned long)dropped);

  if (g_diag_size > 0u) {
    const uint16_t start = (uint16_t)((g_diag_head + BALANCE_LOG_CAPACITY - g_diag_size) % BALANCE_LOG_CAPACITY);
    for (uint16_t i = 0u; i < g_diag_size; ++i) {
      const BalanceDiagEntry &e = g_diag_ring[(uint16_t)((start + i) % BALANCE_LOG_CAPACITY)];
      Serial.printf(
          "{\"cmd\":\"BALANCE_TRACE\",\"trace_id\":%lu,\"i\":%u,\"t\":%lu,\"elapsed_us\":%lu,\"pitch_raw_rad\":%.6f,\"pitch_rate_raw\":%.6f,\"theta_eq_rad\":%.6f,\"theta_tared_rad\":%.6f,\"theta_dot\":%.6f,\"theta_dot_bias\":%.6f,\"imu_acc_mag_g\":%.6f,\"imu_accel_valid\":%u,\"imu_int_fb_y\":%.6f,\"x_m\":%.6f,\"x_dot\":%.6f,\"u_unsat\":%.6f,\"u\":%.6f,\"pos_l_raw\":%.6f,\"pos_r_raw\":%.6f,\"vel_l_raw\":%.6f,\"vel_r_raw\":%.6f,\"tare_eq_count\":%u,\"tare_stable_count\":%u,\"dt_us\":%lu,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
          (unsigned long)g_trace_id,
          (unsigned int)i,
          (unsigned long)e.t_us,
          (unsigned long)e.elapsed_us,
          e.pitch_raw_rad,
          e.pitch_rate_raw,
          e.theta_eq_rad,
          e.theta_tared_rad,
          e.theta_dot,
          e.theta_dot_bias,
          e.imu_acc_mag_g,
          (unsigned int)e.imu_accel_valid,
          e.imu_int_fb_y,
          e.x_m,
          e.x_dot,
          e.u_unsat,
          e.u_cmd,
          e.pos_l_raw,
          e.pos_r_raw,
          e.vel_l_raw,
          e.vel_r_raw,
          (unsigned int)e.tare_eq_count,
          (unsigned int)e.tare_stable_count,
          (unsigned long)e.dt_us,
          (unsigned long)e.tx_period_us,
          (e.tx_period_us > 0u) ? (1000000.0f / (float)e.tx_period_us) : 0.0f);
    }
  }
  Serial.printf("{\"cmd\":\"BALANCE_TRACE_END\",\"trace_id\":%lu,\"reason\":\"%s\",\"samples\":%u}\r\n",
                (unsigned long)g_trace_id,
                r,
                (unsigned int)g_diag_size);
  g_diag_dumped = true;
}

static inline void send_iqreq(float torque_nm,
                              uint8_t node_id,
                              FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                              FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, node_id);
  msg.len = 8;
  msg.flags.extended = 1;
  canPackFloat(torque_nm, msg.buf);
  canPackFloat(0.0f, msg.buf + 4);

  const uint8_t tx_bus = can_tx_bus_for_node(node_id);
  if (tx_bus == CAN_TX_BUS_CAN1 || tx_bus == CAN_TX_BUS_BOTH) {
    can1.write(msg);
  }
  if (tx_bus == CAN_TX_BUS_CAN2 || tx_bus == CAN_TX_BUS_BOTH) {
    can2.write(msg);
  }
}

static inline void reset_balance_state() {
  g_first_entry = true;
  g_start_us = 0u;
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
  g_prev_l = 0.0f;
  g_prev_r = 0.0f;
  g_unwrap_l = 0.0f;
  g_unwrap_r = 0.0f;
  g_vel_filt_l = 0.0f;
  g_vel_filt_r = 0.0f;

  g_u_cmd_hold = 0.0f;
  g_last_d_l_rad = 0.0f;
  g_last_d_r_rad = 0.0f;
  g_pos_range_ok = true;
  g_pos_step_ok = true;
  diag_reset();
}

static inline void send_zero_and_idle(Supervisor_typedef *sup,
                                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                                      FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  const uint16_t esc_n = (sup->esc_count >= 2u) ? 2u : sup->esc_count;
  for (uint16_t i = 0u; i < esc_n; ++i) {
    send_iqreq(0.0f, sup->esc[i].config.node_id, can1, can2);
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

void balance_TWR_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                      FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  if (sup == nullptr) return;
  if (sup->esc_count < 2u) {
    send_zero_and_idle(sup, can1, can2);
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
  g_u_cmd_hold = 0.0f;
  diag_reset();
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
    send_zero_and_idle(sup, can1, can2);
    return;
  }

  const uint32_t tx_period_us = BALANCE_TX_PERIOD_US;
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
      send_iqreq(0.0f, sup->esc[0].config.node_id, can1, can2);
      send_iqreq(0.0f, sup->esc[1].config.node_id, can1, can2);
      g_u_cmd_hold = 0.0f;
    }
    return;
  }

  const float theta = sup->imu.pitch_rad - g_theta_eq;
  const float theta_dot = sup->imu.pitch_rate - g_theta_dot_bias;

  int8_t theta_sign = 0;
  if (theta > ZERO_CROSS_HYST_RAD) theta_sign = 1;
  else if (theta < -ZERO_CROSS_HYST_RAD) theta_sign = -1;

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

  if (!BALANCE_FIXED_TORQUE_TEST && (!g_pos_range_ok || !g_pos_step_ok)) {
    Serial.printf(
        "{\"cmd\":\"BALANCE_ABORT\",\"reason\":\"pos_units\",\"pos_l_raw\":%.6f,\"pos_r_raw\":%.6f,\"d_l_rad\":%.6f,\"d_r_rad\":%.6f,\"range_ok\":%d,\"step_ok\":%d}\r\n",
        pos_l, pos_r, g_last_d_l_rad, g_last_d_r_rad, g_pos_range_ok ? 1 : 0, g_pos_step_ok ? 1 : 0);
    diag_dump("pos_units");
    send_zero_and_idle(sup, can1, can2);
    return;
  }

  if (!BALANCE_FIXED_TORQUE_TEST &&
      (fabsf(theta) > THETA_FAIL_RAD || fabsf(vel_l) > VEL_FAIL_RAD_S || fabsf(vel_r) > VEL_FAIL_RAD_S)) {
    Serial.printf("{\"cmd\":\"BALANCE_ABORT\",\"reason\":\"limits\",\"theta\":%.4f,\"vel_l\":%.3f,\"vel_r\":%.3f}\r\n",
                  theta, vel_l, vel_r);
    diag_dump("limits");
    send_zero_and_idle(sup, can1, can2);
    return;
  }

  const float u_model = BALANCE_FIXED_TORQUE_TEST
                            ? BALANCE_FIXED_TORQUE_NM
                            : (-(K_DISC[0] * theta +
                                 K_DISC[1] * theta_dot +
                                 K_DISC[2] * x_wheel_m +
                                 K_DISC[3] * x_dot_m_s) * SAFETY_SCALE);

  if (tx_due) {
    float u_cmd = clampf(u_model, -TORQUE_CLAMP_NM, TORQUE_CLAMP_NM);
    g_u_cmd_hold = u_cmd;
    g_last_tx_us = now_us;
#if SEND_TORQUE
    send_iqreq(-u_cmd, sup->esc[0].config.node_id, can1, can2);
    send_iqreq(u_cmd, sup->esc[1].config.node_id, can1, can2);
#else
    send_iqreq(0.0f, sup->esc[0].config.node_id, can1, can2);
    send_iqreq(0.0f, sup->esc[1].config.node_id, can1, can2);
#endif

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
    e.u_unsat = u_model;
    e.u_cmd = g_u_cmd_hold;
    e.pos_l_raw = pos_l;
    e.pos_r_raw = pos_r;
    e.vel_l_raw = vel_l;
    e.vel_r_raw = vel_r;
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
      send_zero_and_idle(sup, can1, can2);
      return;
    }
  }
}

void balance_TWR_dump_on_mode_exit(const char *reason) {
  if (g_first_entry) return;
  diag_dump((reason != nullptr) ? reason : "mode_exit");
  reset_balance_state();
}
