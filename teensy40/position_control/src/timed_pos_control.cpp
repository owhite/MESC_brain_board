#include "supervisor.h"
#include "timed_pos_control.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

#define SEND_TORQUE
#define SEND_TELEMETRY
#define CAN_TX_PROOF_SUMMARY 1

// Nearly ALL of this code was written with a combination of chatGPT prompts and codex assistance
// with iterative refinement based on testing and debugging on the actual hardware rig.
// 
// Control-theory constants: mode limits, gains, friction model, shaping, and safety thresholds.
// --- Target/mode selection ---
static constexpr uint8_t POSITION_ESC_INDEX = 0u;   // ESC slot index used by this mode (0 -> node 11 in this rig).
static constexpr int32_t SWEEP_DEG_LIMIT = 10000;   // Allowed input range for `sweep <deg>` command.

// --- Outer-loop position controller gains/limits ---
static constexpr float POS_KP_DEFAULT = 0.32f;      // Position gain [Nm/rad].
static constexpr float POS_KD_DEFAULT = 0.012f;     // Damping gain [Nm/(rad/s)].
static constexpr float POS_TORQUE_CLAMP_NM = 0.90f; // Absolute torque limit sent to ESC [Nm].
static constexpr float POS_TORQUE_SLEW_NM_PER_S = 45.0f; // Max torque rate-of-change [Nm/s].
static constexpr float POS_TORQUE_SIGN = -1.0f;     // Mechanical sign alignment between controller and motor wiring.
static constexpr float POS_DONE_BAND_RAD = 3.0f * (PI / 180.0f); // In-band completion threshold [rad] (3 deg).

// --- Friction/backlash helpers ---
static constexpr float STICT_TORQUE_NM = 0.32f;     // Static-friction compensation near zero speed [Nm].
static constexpr float COULOMB_TORQUE_NM = 0.10f;   // Coulomb friction compensation during motion [Nm].
static constexpr float STATIC_VEL_THRESH_RAD_S = 0.20f; // Speed threshold to classify "near static" [rad/s].
static constexpr float STATIC_ERR_THRESH_RAD = 0.03f;   // Min position error before applying stiction term [rad].
static constexpr float BREAKAWAY_ERR_RAD = 0.20f;   // Error threshold for breakaway assist [rad].
static constexpr float BREAKAWAY_TAU_MIN_NM = 0.45f; // Minimum commanded torque magnitude during breakaway [Nm].

// --- Velocity estimate from position samples ---
static constexpr float VEL_FROM_POS_ALPHA = 0.25f;  // LPF blend for vel estimate from delta-pos (0..1).

// --- CAN timing/health thresholds ---
static constexpr uint32_t CMD_TX_MIN_DT_US = 2000u; // Minimum interval between torque TX frames [us] (~500 Hz max).
static constexpr uint32_t POSVEL_TIMEOUT_US = 10000u; // Stale-feedback timeout before fault [us].

// --- Feedback sanity limits ---
static constexpr float POS_RAW_ABS_MAX_RAD = 100.0f;    // Reject absurd absolute position magnitudes [rad].
static constexpr float VEL_RAW_ABS_MAX_RAD_S = 500.0f;  // Reject absurd velocity magnitudes [rad/s].
static constexpr float UNWRAP_BASE_LIMIT_RAD = 40.0f;   // Base unwrap bound [rad].
static constexpr float UNWRAP_TARGET_MARGIN_RAD = 12.0f; // Extra unwrap headroom above requested move [rad].
static constexpr uint8_t INVALID_FB_MAX_CONSEC = 3u;    // Consecutive invalid feedback samples before fault.
static constexpr float POS_JUMP_BASE_RAD = 0.30f;       // Base per-sample delta-pos jump allowance [rad].
static constexpr float POS_JUMP_SPEED_RAD_S = 350.0f;   // Additional allowed jump scale by dt [rad/s].
static constexpr uint8_t POS_JUMP_MAX_CONSEC = 8u;      // Consecutive implausible jumps before fault.

static int report_counter = 0;
static bool first_entry = true;
static uint32_t start_time_us = 0;
static uint32_t last_posvel_rx_used_us = 0;
static uint32_t last_posvel_us = 0;
static float prev_pos_raw = 0.0f;
static float pos_unwrap_rad = 0.0f;
static float pos_start_raw_rad = 0.0f;
static float vel_from_pos_rad_s = 0.0f;
static float target_unwrap_rad = 0.0f;
static float unwrap_limit_rad = 0.0f;
static float tau_cmd_last = 0.0f;
static uint32_t last_torque_tx_us = 0u;
static int32_t target_deg_cmd = 0;
static uint8_t invalid_fb_consec = 0;
static uint8_t pos_jump_consec = 0u;
static uint32_t pos_jump_total = 0u;

struct CanTxProofStats {
  uint32_t tx_attempts = 0;
  uint32_t tx_ok = 0;
  uint32_t tx_fail = 0;
  uint32_t last_report_us = 0;
};
static CanTxProofStats g_can_tx_proof;

static inline float sgnf(float x) {
  if (x > 0.0f) return 1.0f;
  if (x < 0.0f) return -1.0f;
  return 0.0f;
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float wrap_pm_pi(float x) {
  while (x >= PI) x -= 2.0f * PI;
  while (x < -PI) x += 2.0f * PI;
  return x;
}

static bool ESC_torque_cmd(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can,
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
#ifdef SEND_TORQUE
  return can.write(msg);
#else
  (void)can;
  return false;
#endif
}

void timed_pos_control_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
  if (!sup) return;
  if (POSITION_ESC_INDEX >= sup->esc_count) {
    sup->mode = SUP_MODE_IDLE;
    return;
  }

  if (!sup->esc[POSITION_ESC_INDEX].state.alive) {
    ESC_torque_cmd(sup, can, POSITION_ESC_INDEX, 0.0f);
    Serial.println("timed_pos_control exit: ESC not alive -> idle");
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_posvel_rx_used_us = 0;
    return;
  }

  if (first_entry) {
    first_entry = false;
    start_time_us = micros();
    last_posvel_rx_used_us = 0;
    last_posvel_us = 0;
    // Target generation: interpret sweep as a relative move from current position.
    target_deg_cmd = (int32_t)clampf((float)sup->user_timed_pos_sweep_deg,
                                     -(float)SWEEP_DEG_LIMIT,
                                     (float)SWEEP_DEG_LIMIT);
    pos_start_raw_rad = sup->esc[POSITION_ESC_INDEX].state.pos_rad;
    prev_pos_raw = pos_start_raw_rad;
    pos_unwrap_rad = pos_start_raw_rad;
    target_unwrap_rad = pos_start_raw_rad + (((float)target_deg_cmd) * (PI / 180.0f));
    const float target_span = fabsf(target_unwrap_rad - pos_start_raw_rad);
    unwrap_limit_rad = fmaxf(UNWRAP_BASE_LIMIT_RAD, target_span + UNWRAP_TARGET_MARGIN_RAD);
    vel_from_pos_rad_s = 0.0f;
    tau_cmd_last = 0.0f;
    last_torque_tx_us = start_time_us;
    g_can_tx_proof = CanTxProofStats{};
    g_can_tx_proof.last_report_us = start_time_us;
    report_counter = 0;
    invalid_fb_consec = 0u;
    pos_jump_consec = 0u;
    pos_jump_total = 0u;
    Serial.printf("{\"cmd\":\"TIMED_POS_START\",\"deg\":%ld,\"target_rad\":%.6f}\r\n",
                  (long)target_deg_cmd,
                  target_unwrap_rad);
  }

  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - start_time_us;
  const uint32_t pos_us = sup->esc[POSITION_ESC_INDEX].status.last_update_us;
  const float pos_raw_now = sup->esc[POSITION_ESC_INDEX].state.pos_rad;
  const float vel_rad_s = sup->esc[POSITION_ESC_INDEX].state.vel_rad_s;

  // Safety/fault theory: reject clearly invalid sensor values before using them in control.
  const bool invalid_feedback_now =
      (!isfinite(pos_raw_now)) || (!isfinite(vel_rad_s)) ||
      (fabsf(pos_raw_now) > POS_RAW_ABS_MAX_RAD) ||
      (fabsf(vel_rad_s) > VEL_RAW_ABS_MAX_RAD_S);
  if (invalid_feedback_now) {
    if (invalid_fb_consec < 255u) invalid_fb_consec++;
  } else {
    invalid_fb_consec = 0u;
  }
  if (invalid_fb_consec >= INVALID_FB_MAX_CONSEC) {
    ESC_torque_cmd(sup, can, POSITION_ESC_INDEX, 0.0f);
    Serial.printf(
      "{\"cmd\":\"TIMED_POS_FAULT\",\"reason\":\"invalid_feedback\",\"pos_raw\":%.6f,"
        "\"vel_raw\":%.6f,\"n_bad\":%u}\r\n",
        pos_raw_now,
        vel_rad_s,
        (unsigned)invalid_fb_consec);
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_posvel_rx_used_us = 0;
    return;
  }

  const bool new_pos = (pos_us != 0u) && (pos_us != last_posvel_rx_used_us);
  if (new_pos) {
    // Unwrap + velocity estimation: integrate wrapped angle deltas and estimate velocity from delta-position.
    const float pos_raw = pos_raw_now;
    const float d = wrap_pm_pi(pos_raw - prev_pos_raw);
    bool accept_delta = true;
    if (last_posvel_us != 0u && pos_us > last_posvel_us) {
      const float dt_pos_s = (float)(pos_us - last_posvel_us) * 1e-6f;
      if (dt_pos_s > 1e-6f) {
        const float d_lim = POS_JUMP_BASE_RAD + (POS_JUMP_SPEED_RAD_S * dt_pos_s);
        if (fabsf(d) > d_lim) {
          accept_delta = false;
        } else {
          const float vel_inst = d / dt_pos_s;
          vel_from_pos_rad_s =
              (VEL_FROM_POS_ALPHA * vel_inst) + ((1.0f - VEL_FROM_POS_ALPHA) * vel_from_pos_rad_s);
        }
      }
    }
    if (accept_delta) {
      pos_unwrap_rad += d;
      pos_jump_consec = 0u;
    } else {
      if (pos_jump_consec < 255u) pos_jump_consec++;
      pos_jump_total++;
    }
    last_posvel_us = pos_us;
    prev_pos_raw = pos_raw;
    last_posvel_rx_used_us = pos_us;
  }

  if (pos_jump_consec >= POS_JUMP_MAX_CONSEC) {
    // Safety/fault theory: reject persistent implausible per-sample position jumps.
    ESC_torque_cmd(sup, can, POSITION_ESC_INDEX, 0.0f);
    Serial.printf(
      "{\"cmd\":\"TIMED_POS_FAULT\",\"reason\":\"pos_jump\",\"jump_consec\":%u,\"jump_total\":%lu,\"pos_raw\":%.6f}\r\n",
        (unsigned)pos_jump_consec,
        (unsigned long)pos_jump_total,
        pos_raw_now);
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    return;
  }

  const uint32_t pos_age_us = (pos_us > 0u) ? (uint32_t)(now_us - pos_us) : UINT32_MAX;
  if (pos_age_us > POSVEL_TIMEOUT_US) {
    // Safety/fault theory: stale telemetry timeout forces zero torque and exits mode.
    ESC_torque_cmd(sup, can, POSITION_ESC_INDEX, 0.0f);
    Serial.printf("{\"cmd\":\"TIMED_POS_FAULT\",\"reason\":\"stale_posvel\",\"age_us\":%lu}\r\n",
                  (unsigned long)pos_age_us);
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    return;
  }

  if (fabsf(pos_unwrap_rad) > unwrap_limit_rad) {
    // Safety/fault theory: unwrap bound prevents runaway integration from driving unsafe commands.
    ESC_torque_cmd(sup, can, POSITION_ESC_INDEX, 0.0f);
    Serial.printf(
      "{\"cmd\":\"TIMED_POS_FAULT\",\"reason\":\"unwrap_limit\",\"pos_unwrap_rad\":%.6f,"
        "\"target_rad\":%.6f,\"unwrap_limit_rad\":%.6f}\r\n",
        pos_unwrap_rad,
        target_unwrap_rad,
        unwrap_limit_rad);
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    return;
  }

  const float pos_err_rad = target_unwrap_rad - pos_unwrap_rad;

  const float kp = (sup->user_Kp_term > 0.0f) ? sup->user_Kp_term : POS_KP_DEFAULT;
  const float kd = (sup->user_Kd_term > 0.0f) ? sup->user_Kd_term : POS_KD_DEFAULT;

  const float vel_ctrl_rad_s = vel_from_pos_rad_s;
  // Core control law: tau = Kp*(target - position) - Kd*velocity.
  float tau_cmd = (kp * pos_err_rad) - (kd * vel_ctrl_rad_s);
  // Friction/backlash compensation: static, Coulomb, and breakaway assist terms.
  if (fabsf(vel_ctrl_rad_s) < STATIC_VEL_THRESH_RAD_S && fabsf(pos_err_rad) > STATIC_ERR_THRESH_RAD) {
    tau_cmd += STICT_TORQUE_NM * sgnf(pos_err_rad);
  } else if (fabsf(vel_ctrl_rad_s) >= STATIC_VEL_THRESH_RAD_S) {
    tau_cmd += COULOMB_TORQUE_NM * sgnf(vel_ctrl_rad_s);
  }
  if (fabsf(pos_err_rad) > BREAKAWAY_ERR_RAD && fabsf(vel_ctrl_rad_s) < STATIC_VEL_THRESH_RAD_S) {
    const float tau_floor = BREAKAWAY_TAU_MIN_NM * sgnf(pos_err_rad);
    if (fabsf(tau_cmd) < fabsf(tau_floor)) tau_cmd = tau_floor;
  }

  // Actuator shaping: slew-rate limit, clamp, and sign mapping before CAN transmission.
  const float loop_dt_s = (sup->timing.dt_us > 0u) ? ((float)sup->timing.dt_us * 1e-6f) : 0.001f;
  const float tau_step = POS_TORQUE_SLEW_NM_PER_S * loop_dt_s;
  if ((tau_cmd * tau_cmd_last) < 0.0f) {
    // Allow quicker direction changes: don't force long ramp through wrong-sign torque.
    tau_cmd_last = 0.0f;
  }
  tau_cmd = clampf(tau_cmd, tau_cmd_last - tau_step, tau_cmd_last + tau_step);
  tau_cmd = clampf(tau_cmd, -POS_TORQUE_CLAMP_NM, POS_TORQUE_CLAMP_NM);
  tau_cmd_last = tau_cmd;
  const float tau_send = POS_TORQUE_SIGN * tau_cmd;

#ifdef SEND_TORQUE
  // Command transmission rate limit: cap outgoing IQREQ command rate on the CAN bus.
  if ((uint32_t)(now_us - last_torque_tx_us) >= CMD_TX_MIN_DT_US) {
    const bool ok = ESC_torque_cmd(sup, can, POSITION_ESC_INDEX, tau_send);
    g_can_tx_proof.tx_attempts += 1u;
    g_can_tx_proof.tx_ok += ok ? 1u : 0u;
    g_can_tx_proof.tx_fail += ok ? 0u : 1u;
    last_torque_tx_us = now_us;
  }
#endif

#if CAN_TX_PROOF_SUMMARY
  if ((uint32_t)(now_us - g_can_tx_proof.last_report_us) >= 1000000u) {
    const uint32_t attempts = g_can_tx_proof.tx_attempts;
    const float fail_pct = (attempts > 0u)
                             ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)attempts)
                             : 0.0f;
    Serial.printf(
        "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f,\"mode\":%d,\"posvel_age_us\":%lu}\r\n",
        (unsigned long)g_can_tx_proof.tx_attempts,
        (unsigned long)g_can_tx_proof.tx_ok,
        (unsigned long)g_can_tx_proof.tx_fail,
        fail_pct,
        (int)sup->mode,
        (unsigned long)pos_age_us);
    g_can_tx_proof.last_report_us += 1000000u;
  }
#endif

#ifdef SEND_TELEMETRY
  if (++report_counter >= TELEMETRY_DECIMATE) {
    report_counter = 0;
    const uint32_t loop_dt_us = sup->timing.dt_us;
    const float loop_hz = (loop_dt_us > 0u) ? (1000000.0f / (float)loop_dt_us) : 0.0f;
    // Completion criterion: in-band flag indicates error within done threshold.
    const bool in_band = fabsf(pos_err_rad) <= POS_DONE_BAND_RAD;
    Serial.printf(
      "{\"cmd\":\"TIMED_POS_CTRL\",\"t\":%lu,\"deg_cmd\":%ld,\"target_rad\":%.6f,"
        "\"pos_unwrap_rad\":%.6f,\"pos_raw\":%.6f,\"vel_raw\":%.6f,"
        "\"vel_ctrl\":%.6f,\"err_rad\":%.6f,\"tau_cmd\":%.4f,\"tau_send\":%.4f,"
        "\"jump_consec\":%u,\"jump_total\":%lu,\"in_band\":%d,"
        "\"new_pos\":%d,\"pos_age_us\":%lu,\"loop_dt_us\":%lu,\"loop_hz\":%.2f,\"ovr\":%lu}\r\n",
        (unsigned long)now_us,
        (long)target_deg_cmd,
        target_unwrap_rad,
        pos_unwrap_rad,
        sup->esc[POSITION_ESC_INDEX].state.pos_rad,
        vel_rad_s,
        vel_ctrl_rad_s,
        pos_err_rad,
        tau_cmd,
        tau_send,
        (unsigned)pos_jump_consec,
        (unsigned long)pos_jump_total,
        in_band ? 1 : 0,
        new_pos ? 1 : 0,
        (unsigned long)pos_age_us,
        (unsigned long)loop_dt_us,
        loop_hz,
        sup->timing.overruns);
  }
#endif

  if (sup->user_timed_pos_total_us > 0 && elapsed_us > sup->user_timed_pos_total_us) {
    // End-of-run result: stop torque and emit final error/health summary.
    const bool okStop = ESC_torque_cmd(sup, can, POSITION_ESC_INDEX, 0.0f);
    g_can_tx_proof.tx_attempts += 1u;
    g_can_tx_proof.tx_ok += okStop ? 1u : 0u;
    g_can_tx_proof.tx_fail += okStop ? 0u : 1u;

    Serial.printf(
      "{\"cmd\":\"TIMED_POS_DONE\",\"elapsed_us\":%lu,\"limit_us\":%lu,\"deg_cmd\":%ld,"
        "\"final_pos_unwrap_rad\":%.6f,\"final_err_rad\":%.6f,\"tx_fail\":%lu}\r\n",
        (unsigned long)elapsed_us,
        (unsigned long)sup->user_timed_pos_total_us,
        (long)target_deg_cmd,
        pos_unwrap_rad,
        pos_err_rad,
        (unsigned long)g_can_tx_proof.tx_fail);
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    last_posvel_rx_used_us = 0;
  }
}
