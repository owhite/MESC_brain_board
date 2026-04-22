#include "supervisor.h"
#include <string.h>
#include <math.h>
#include "main.h"
#include "test_can_transmit_mode.h"
#include "balance_TWR_mode.h"
#include "balance_debug_mode.h"
#include "telemetry_link.h"

// ---------------- Global Flags ----------------
// These are set by the control ISR to signal the main loop.
volatile uint32_t g_control_pending_ticks = 0;
volatile uint32_t g_control_now_us = 0;
static constexpr uint32_t ESC_ALIVE_TIMEOUT_US = 200000u;

static inline bool is_balance_mode(SupervisorMode m) {
  return (m == SUP_MODE_BALANCE_HOLD) ||
         (m == SUP_MODE_BALANCE_TWR) ||
         (m == SUP_MODE_BALANCE_DEBUG);
}

static const char* mode_to_str(SupervisorMode mode) {
  switch (mode) {
    case SUP_MODE_IDLE: return "SUP_MODE_IDLE";
    case SUP_MODE_CALIBRATE: return "SUP_MODE_CALIBRATE";
    case SUP_VERIFY_ANGLE: return "SUP_VERIFY_ANGLE";
    case SUP_MODE_BALANCE_HOLD: return "SUP_MODE_BALANCE_HOLD";
    case SUP_MODE_BALANCE_TWR: return "SUP_MODE_BALANCE_TWR";
    case SUP_MODE_BALANCE_DEBUG: return "SUP_MODE_BALANCE_DEBUG";
    case SUP_MODE_TEST_CAN: return "SUP_MODE_TEST_CAN";
    default: return "SUP_MODE_UNKNOWN";
  }
}

// Note there are multiple ISRs, some for the controlLoop(),
// and some for RC transmitter input capture.

// ---------------- Local Supervisor Reference ----------------
// Global reference for use by RC ISRs.
static Supervisor_typedef *g_sup = nullptr;
static uint8_t g_rc_pins[RC_INPUT_MAX_PINS];
static volatile uint32_t g_rise_time[RC_INPUT_MAX_PINS];

// ---------------- RC Input Capture ISRs ----------------
// Each ISR handles one RC input pin. On rising edge, records time;
// on falling edge, computes pulse width and stores it.
static void rc_isr0() { uint8_t i=0; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr1() { uint8_t i=1; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr2() { uint8_t i=2; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr3() { uint8_t i=3; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}

// Lookup table of RC ISRs by channel index.
static void (*rc_isrs[RC_INPUT_MAX_PINS])() = {rc_isr0, rc_isr1, rc_isr2, rc_isr3};

// ---------------- Control Loop ISR ----------------
// Fires at CONTROL_PERIOD_US and sets a flag for the main loop
// to run the deterministic controlLoop().
void controlLoop_isr(void) {
  g_control_now_us = micros();
  if (g_control_pending_ticks < UINT32_MAX) {
    g_control_pending_ticks++;
  }
}

// --- Helper: compute shortest angular difference (target - actual) in [-π, +π]
float angle_diff(float target, float actual) {
  float diff = fmodf(target - actual + M_PI, 2.0f * M_PI);
  if (diff < 0) diff += 2.0f * M_PI;
  return diff - M_PI;
}

// ---------------- Supervisor Initialization ----------------
// Sets up ESCs, RC inputs, and resets timing/telemetry stats.
// Called once at startup from main().
void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count) {

  if (!sup) return;
  g_sup = sup;

  *sup = Supervisor_typedef{};

  // ---- Clear ESC lookup table ----
  for (uint16_t i = 0; i < ESC_LOOKUP_SIZE; ++i) {
    esc_lookup[i] = nullptr;
  }

  // ---- ESC setup ----
  if (esc_count > SUPERVISOR_MAX_ESCS) esc_count = SUPERVISOR_MAX_ESCS;
  sup->esc_count = esc_count;

  uint32_t now = micros();

  for (uint16_t i = 0; i < sup->esc_count; ++i) {
    const char *nm = (esc_names && esc_names[i]) ? esc_names[i] : "";
    uint16_t nid   = (node_ids) ? node_ids[i] : 0;

    sup->esc[i] = ESC(nm, nid);
    sup->esc[i].init();
    sup->last_esc_heartbeat_us[i] = now;

    if (nid < ESC_LOOKUP_SIZE) {
      esc_lookup[nid] = &sup->esc[i];
    }
  }

  sup->mode = SUP_MODE_IDLE;
  sup->imu.valid = false;
  sup->imu.pitch_rad = 0.0f;
  sup->imu.pitch_rate_raw = 0.0f;
  sup->imu.pitch_rate = 0.0f;
  sup->imu.last_update_us = now;

  // ---- Timing stats ----
  sup->timing.last_tick_us = now;
  sup->timing.dt_us = 0;
  sup->timing.exec_time_us = 0;
  resetLoopTimingStats(sup);

  sup->last_health_ms = 0;

  // ---- RC setup ----
  if (rc_count > RC_INPUT_MAX_PINS) rc_count = RC_INPUT_MAX_PINS;
  sup->rc_count = rc_count;

  for (uint8_t i = 0; i < sup->rc_count; i++) {
    g_rc_pins[i] = rc_pins[i];
    pinMode(g_rc_pins[i], INPUT);

    sup->rc_raw[i].raw_us = 0;
    sup->rc_raw[i].last_update = 0;
    sup->rc[i].norm = -1.0f;
    sup->rc[i].valid = false;

    attachInterrupt(digitalPinToInterrupt(g_rc_pins[i]), rc_isrs[i], CHANGE);
  }

  resetTelemetryStats(sup);
}


// ---------------- RC Normalization ----------------
// Converts raw RC input pulse widths into normalized values
// and checks for timeouts/validity.
void updateRC(Supervisor_typedef *sup) {
  if (!sup) return;

  for (uint8_t i = 0; i < sup->rc_count; i++) {
    uint32_t age = micros() - sup->rc_raw[i].last_update;

    if (age > RC_INPUT_TIMEOUT_US || sup->rc_raw[i].raw_us == 0) {
      sup->rc[i].norm  = -1.0f;
      sup->rc[i].raw_us = 0;
      sup->rc[i].valid = false;
      continue;
    }

    uint16_t pw = sup->rc_raw[i].raw_us;
    if (pw < RC_INPUT_MIN_US) pw = RC_INPUT_MIN_US;
    if (pw > RC_INPUT_MAX_US) pw = RC_INPUT_MAX_US;

    float norm = 2.0f * ((float)(pw - RC_INPUT_MIN_US) /
      (float)(RC_INPUT_MAX_US - RC_INPUT_MIN_US)) - 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    if (norm > 1.0f) norm = 1.0f;

    sup->rc[i].raw_us = pw;
    sup->rc[i].norm   = norm;
    sup->rc[i].valid  = true;
  }
}

// Compatibility wrapper if your header still declares updateSupervisorRC()
void updateSupervisorRC(Supervisor_typedef *sup) {
  updateRC(sup);
}

// ---------------- Reset Timing Stats ----------------
// Clears loop timing stats so the next window of data
// can be collected cleanly.
void resetLoopTimingStats(Supervisor_typedef *sup) {
  if (!sup) return;
  sup->timing.min_dt_us = UINT32_MAX;
  sup->timing.max_dt_us = 0;
  sup->timing.sum_dt_us = 0;
  sup->timing.count = 0;
  sup->timing.overruns = 0;
}

// ---------------- Reset Telemetry Stats ----------------
// Clears telemetry blocking statistics so the next window
// can be measured independently.
void resetTelemetryStats(Supervisor_typedef *sup) {
  sup->serial1_stats.last_block_us = 0;
  sup->serial1_stats.max_block_us = 0;
  sup->serial1_stats.sum_block_us = 0;
  sup->serial1_stats.count = 0;
  sup->last_health_ms = 0; 
}

// ---------------- Main Control Loop ----------------
static int telem_counter = 0;
static int verify_counter = 0;
static constexpr uint16_t VERIFY_RMS_WINDOW = 200u;

struct RollingRms {
  float buf[VERIFY_RMS_WINDOW] = {0.0f};
  uint16_t head = 0u;
  uint16_t count = 0u;
  float sumsq = 0.0f;

  void reset() {
    head = 0u;
    count = 0u;
    sumsq = 0.0f;
    for (uint16_t i = 0u; i < VERIFY_RMS_WINDOW; ++i) {
      buf[i] = 0.0f;
    }
  }

  void push(float x) {
    const float x2 = x * x;
    if (count < VERIFY_RMS_WINDOW) {
      buf[head] = x;
      sumsq += x2;
      ++count;
    } else {
      const float old = buf[head];
      sumsq -= old * old;
      buf[head] = x;
      sumsq += x2;
    }
    ++head;
    if (head >= VERIFY_RMS_WINDOW) head = 0u;
  }

  float rms() const {
    if (count == 0u) return 0.0f;
    return sqrtf(sumsq / (float)count);
  }
};

static RollingRms verify_rate_raw_rms;
static RollingRms verify_rate_filt_rms;

static inline bool send_torque_for_esc(Supervisor_typedef *sup,
                                       FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                                       FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2,
                                       uint8_t esc_num,
                                       float torque_nm) {
  if (sup == nullptr) return false;
  if (esc_num >= sup->esc_count) return false;
  const float torque_cmd_nm = (SEND_TORQUE != 0) ? torque_nm : 0.0f;

  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[esc_num].config.node_id);
  msg.len = 8;
  msg.flags.extended = 1;
  canPackFloat(torque_cmd_nm, msg.buf);
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

static inline void send_zero_torque_all(Supervisor_typedef *sup,
                                        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                                        FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {
  const uint16_t esc_n = sup->esc_count;
  for (uint16_t i = 0; i < esc_n; ++i) {
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[i].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(0.0f, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);

    const uint8_t tx_bus = can_tx_bus_for_node(sup->esc[i].config.node_id);
    if (tx_bus == CAN_TX_BUS_CAN1 || tx_bus == CAN_TX_BUS_BOTH) {
      can1.write(msg);
    }
    if (tx_bus == CAN_TX_BUS_CAN2 || tx_bus == CAN_TX_BUS_BOTH) {
      can2.write(msg);
    }
  }
}

void controlLoop(Supervisor_typedef *sup,
                 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                 FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2) {

  // ----- Update timing stats -----
  uint32_t start_us = micros();

  static uint32_t last_control_us = 0u;
  if (last_control_us == 0u) {
    last_control_us = start_us;
    return;
  }

  uint32_t dt_us = start_us - last_control_us;
  last_control_us = start_us;

  sup->timing.dt_us = dt_us;
  sup->timing.last_tick_us = start_us;

  if (dt_us < sup->timing.min_dt_us) sup->timing.min_dt_us = dt_us;
  if (dt_us > sup->timing.max_dt_us) sup->timing.max_dt_us = dt_us;
  sup->timing.sum_dt_us += dt_us;
  sup->timing.count++;

  if (dt_us > CONTROL_PERIOD_US + 100){
    sup->timing.overruns++;
  }

  // ---- Update RC PWM input ----
  updateRC(sup);

  // ---- ESC alive timeout gate ----
  for (uint16_t i = 0; i < sup->esc_count; ++i) {
    ESC &esc = sup->esc[i];
    if (esc.status.last_update_us == 0u) {
      esc.status.alive = false;
      esc.state.alive = false;
      if (sup->esc_alive_false_count[i] < UINT32_MAX) {
        sup->esc_alive_false_count[i]++;
      }
      continue;
    }
    const uint32_t age_us = (uint32_t)(start_us - esc.status.last_update_us);
    const bool alive = (age_us <= ESC_ALIVE_TIMEOUT_US);
    esc.status.alive = alive;
    esc.state.alive = alive;
    if (!alive && sup->esc_alive_false_count[i] < UINT32_MAX) {
      sup->esc_alive_false_count[i]++;
    }
  }

  // ---- Core control loop body ----
  static SupervisorMode s_prev_mode = SUP_MODE_IDLE;
  if (sup->mode != s_prev_mode) {
    if (!is_balance_mode(s_prev_mode) && is_balance_mode(sup->mode)) {
#if TELEMETRY_LINK_ENABLE
      Serial.printf(
          "{\"cmd\":\"TELEM_LOG_START\",\"from\":\"%s\",\"to\":\"%s\",\"packet_size_bytes\":%lu,\"queued_packets\":%lu,\"queued_bytes\":%lu}\r\n",
          mode_to_str(s_prev_mode),
          mode_to_str(sup->mode),
          (unsigned long)telemetry_packet_size_bytes(),
          (unsigned long)telemetry_pending_packets(),
          (unsigned long)telemetry_pending_bytes());
#else
      Serial.printf(
          "{\"cmd\":\"TELEM_LOG_START\",\"from\":\"%s\",\"to\":\"%s\",\"packet_size_bytes\":0,\"queued_packets\":0,\"queued_bytes\":0}\r\n",
          mode_to_str(s_prev_mode),
          mode_to_str(sup->mode));
#endif
    }
    if (is_balance_mode(s_prev_mode) && !is_balance_mode(sup->mode)) {
      if (s_prev_mode == SUP_MODE_BALANCE_DEBUG) {
        balance_debug_dump_on_mode_exit("mode_exit");
      } else {
        balance_TWR_dump_on_mode_exit("mode_exit");
      }
    }
    if (sup->mode == SUP_VERIFY_ANGLE) {
      verify_rate_raw_rms.reset();
      verify_rate_filt_rms.reset();
    }
    s_prev_mode = sup->mode;
  }

  switch (sup->mode) {
	  case SUP_MODE_IDLE: {
	    // --- Send zero torque (motor free) ---

	    if (++telem_counter >= TELEMETRY_DECIMATE) {
	      telem_counter = 0;
        send_zero_torque_all(sup, can1, can2);
	    }

    break;
  }

  case SUP_MODE_CALIBRATE: {
    // Keep motors de-energized while user performs kickstand calibration.
    if (++telem_counter >= TELEMETRY_DECIMATE) {
      telem_counter = 0;
      send_zero_torque_all(sup, can1, can2);
    }
    break;
  }

  case SUP_VERIFY_ANGLE: {
    // VERIFY_ANGLE always emits IMU telemetry; optionally drives motors for vibration checks.
    verify_rate_raw_rms.push(sup->imu.pitch_rate_raw);
    verify_rate_filt_rms.push(sup->imu.pitch_rate);

    if (sup->user_verify_motor_enable) {
      if (sup->esc_count > 0u) {
        send_torque_for_esc(sup, can1, can2, 0, sup->user_verify_tau_left);
      }
      if (sup->esc_count > 1u) {
        send_torque_for_esc(sup, can1, can2, 1, sup->user_verify_tau_right);
      }
    }

    if (++verify_counter >= TELEMETRY_DECIMATE) {
      verify_counter = 0;
      if (!sup->user_verify_motor_enable) {
        send_zero_torque_all(sup, can1, can2);
      }

      const uint32_t now_us = micros();
      const uint32_t imu_age_us = (sup->imu.last_update_us == 0u)
                                  ? UINT32_MAX
                                  : (uint32_t)(now_us - sup->imu.last_update_us);
      const float pitch_deg = sup->imu.pitch_rad * (180.0f / PI);
      const float pitch_rate_raw_deg_s = sup->imu.pitch_rate_raw * (180.0f / PI);
      const float pitch_rate_deg_s = sup->imu.pitch_rate * (180.0f / PI);

      Serial.printf(
        "{\"cmd\":\"VERIFY_ANGLE\",\"t\":%lu,\"pitch_rad\":%.6f,\"pitch_deg\":%.3f,\"pitch_rate_raw_rad_s\":%.6f,\"pitch_rate_raw_deg_s\":%.3f,\"pitch_rate_rad_s\":%.6f,\"pitch_rate_deg_s\":%.3f,\"rms_raw_rad_s\":%.6f,\"rms_filt_rad_s\":%.6f,\"rms_n\":%u,\"motor\":%d,\"tau_left_nm\":%.3f,\"tau_right_nm\":%.3f,\"imu_valid\":%d,\"imu_age_us\":%lu,\"loop_dt_us\":%lu}\r\n",
        (unsigned long)now_us,
        sup->imu.pitch_rad,
        pitch_deg,
        sup->imu.pitch_rate_raw,
        pitch_rate_raw_deg_s,
        sup->imu.pitch_rate,
        pitch_rate_deg_s,
        verify_rate_raw_rms.rms(),
        verify_rate_filt_rms.rms(),
        (unsigned int)verify_rate_filt_rms.count,
        sup->user_verify_motor_enable ? 1 : 0,
        sup->user_verify_tau_left,
        sup->user_verify_tau_right,
        sup->imu.valid ? 1 : 0,
        (unsigned long)imu_age_us,
        (unsigned long)sup->timing.dt_us);
    }
    break;
  }
 
  case SUP_MODE_BALANCE_HOLD: {
    balance_TWR_mode(sup, can1, can2);
    break;
  }

  case SUP_MODE_BALANCE_TWR: {
    balance_TWR_mode(sup, can1, can2);
    break;
  }

  case SUP_MODE_BALANCE_DEBUG: {
    balance_debug_mode(sup, can1, can2);
    break;
  }

  case SUP_MODE_TEST_CAN: {
    test_can_transmit_mode(sup, can1, can2);
    break;
  }

  default: {
    break;
  }
  }

  // ---- Finish timing measurement ----
  sup->timing.exec_time_us = micros() - start_us;
}
