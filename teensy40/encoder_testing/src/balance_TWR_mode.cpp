#include "supervisor.h"
#include "balance_TWR_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

#define SEND_TORQUE
#define SEND_LOG // data collection for unwrap debugging; can be a lot of data, so gate behind a define. Logs to ring buffer and can dump on demand via telemetry.
// #define SEND_TELEMETRY // potentially blocking
#define CAN_TX_PROOF_SUMMARY 1
//#define CAN_TX_PROOF_VERBOSE 1

struct UnwrapDebugTick {
  float dL_wrapped = 0.0f;
  float dR_wrapped = 0.0f;
  float max_step_L = 0.0f;
  float max_step_R = 0.0f;
  float dt_gate_s = 0.0f;
  float dt_gate_L_s = 0.0f;
  float dt_gate_R_s = 0.0f;
  float v_meas_from_pos_m_s = 0.0f;
  bool posL_finite = false;
  bool posR_finite = false;
  bool accept_L = false;
  bool accept_R = false;
  bool unwrap_init_event = false;
  bool new_pos_sample = false;
  bool new_pos_L = false;
  bool new_pos_R = false;
};

// ---------------- Logging ----------------
#ifdef SEND_LOG
constexpr uint16_t UNWRAP_LOG_CAPACITY = 2500;  // 5 s at 500 Hz

// Fixed-point scales for compact storage:
// rad/rad_s/delta -> mrad, unwrap -> mrad (int32), x -> mm, v -> mm/s.
constexpr float RAD_TO_MRAD      = 1000.0f;
constexpr float M_TO_MM          = 1000.0f;
constexpr float VEL_M_TO_MM_S    = 1000.0f;
constexpr uint16_t UNWRAP_DUMP_VERSION = 1u;
constexpr uint16_t UNWRAP_DUMP_MSG_TYPE = 1u;      // unwrap log dump
constexpr uint16_t UNWRAP_SAMPLE_RATE_HZ = 500u;

struct __attribute__((packed)) UnwrapLogSample {
  int16_t pos_L_raw_mrad;
  int16_t pos_R_raw_mrad;
  int16_t vel_L_raw_mrad_s;
  int16_t vel_R_raw_mrad_s;
  int16_t dL_mrad;
  int16_t dR_mrad;
  uint8_t accept_flags;       // bit0=L accepted, bit1=R accepted
  int32_t unwrap_L_mrad;
  int32_t unwrap_R_mrad;
  int32_t x_wheel_mm;
  int16_t x_dot_mm_s;
  int16_t x_dot_from_pos_mm_s;
};

struct __attribute__((packed)) UnwrapLogHeader {
  char magic[4];              // "TWR1"
  uint16_t version;
  uint16_t msg_type;
  uint16_t sample_rate_hz;
  uint16_t sample_bytes;
  uint32_t sample_count;
  uint32_t start_index;
  uint32_t payload_bytes;
  uint32_t payload_crc32;
  uint32_t header_crc32;      // optional, set to 0
};
static_assert(sizeof(UnwrapLogHeader) == 32, "UnwrapLogHeader must be 32 bytes");
static_assert(sizeof(UnwrapLogSample) == 29, "UnwrapLogHeader must be 29 bytes");

struct UnwrapCounters {
  uint32_t reject_L_count = 0;
  uint32_t reject_R_count = 0;
  uint32_t nonfinite_pos_count = 0;
  uint32_t unwrap_init_count = 0;
  int16_t max_abs_dL_seen_mrad = 0;
  int16_t max_abs_dR_seen_mrad = 0;
};

static UnwrapLogSample unwrap_log_ring[UNWRAP_LOG_CAPACITY];
static uint16_t unwrap_log_head = 0;   // next write index
static uint16_t unwrap_log_count = 0;  // valid samples in ring
static UnwrapCounters unwrap_counters;
static float unwrap_prev_x_wheel_m = 0.0f;
static bool unwrap_prev_x_valid = false;

enum class DumpPhase : uint8_t { Idle, Header, Payload, Trailer };
struct UnwrapDumpState {
  DumpPhase phase = DumpPhase::Idle;
  UnwrapLogHeader header{};
  uint32_t header_sent = 0;
  uint16_t sample_count = 0;
  uint16_t start_index = 0;
  uint16_t sample_cursor = 0;
  uint16_t sample_byte_offset = 0;
  uint32_t trailer_value = 0;
  uint32_t trailer_sent = 0;
};

static UnwrapDumpState unwrap_dump_state;

static inline int16_t sat_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline int16_t f_to_i16(float x, float scale) {
  if (!isfinite(x)) return 0;
  return sat_i16((int32_t)lroundf(x * scale));
}

static inline int32_t f_to_i32(float x, float scale) {
  if (!isfinite(x)) return 0;
  return (int32_t)lroundf(x * scale);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len) {
  uint32_t c = ~crc;
  for (uint32_t i = 0; i < len; i++) {
    c ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      c = (c & 1u) ? ((c >> 1) ^ 0xEDB88320u) : (c >> 1);
    }
  }
  return ~c;
}

static uint32_t crc32_payload_oldest_to_newest(uint16_t start_index, uint16_t count) {
  uint32_t crc = 0u;
  for (uint16_t i = 0; i < count; i++) {
    const uint16_t idx = (uint16_t)((start_index + i) % UNWRAP_LOG_CAPACITY);
    crc = crc32_update(crc, (const uint8_t *)&unwrap_log_ring[idx], (uint32_t)sizeof(UnwrapLogSample));
  }
  return crc;
}

static void telemetry_reset_unwrap_log() {
  unwrap_log_head = 0;
  unwrap_log_count = 0;
  unwrap_counters = UnwrapCounters{};
  unwrap_prev_x_wheel_m = 0.0f;
  unwrap_prev_x_valid = false;
  unwrap_dump_state = UnwrapDumpState{};
}

static void telemetry_log_unwrap(float pos_L_raw, float pos_R_raw,
                                 float vel_L_raw, float vel_R_raw,
                                 const UnwrapDebugTick &dbg,
                                 float unwrap_L_rad, float unwrap_R_rad,
                                 float x_wheel_m, float x_dot_m_s,
                                 float dt) {
  float x_dot_from_pos = 0.0f;
  if (unwrap_prev_x_valid && isfinite(dt) && dt > 0.0f && isfinite(x_wheel_m)) {
    x_dot_from_pos = (x_wheel_m - unwrap_prev_x_wheel_m) / dt;
  }
  if (isfinite(x_wheel_m)) {
    unwrap_prev_x_wheel_m = x_wheel_m;
    unwrap_prev_x_valid = true;
  } else {
    unwrap_prev_x_valid = false;
  }

  if (!dbg.posL_finite || !dbg.posR_finite) {
    unwrap_counters.nonfinite_pos_count++;
  }
  if (dbg.unwrap_init_event) {
    unwrap_counters.unwrap_init_count++;
  }
  if (dbg.posL_finite && !dbg.accept_L) {
    unwrap_counters.reject_L_count++;
  }
  if (dbg.posR_finite && !dbg.accept_R) {
    unwrap_counters.reject_R_count++;
  }

  const int16_t abs_dL = sat_i16((int32_t)lroundf(fabsf(dbg.dL_wrapped) * RAD_TO_MRAD));
  const int16_t abs_dR = sat_i16((int32_t)lroundf(fabsf(dbg.dR_wrapped) * RAD_TO_MRAD));
  if (abs_dL > unwrap_counters.max_abs_dL_seen_mrad) unwrap_counters.max_abs_dL_seen_mrad = abs_dL;
  if (abs_dR > unwrap_counters.max_abs_dR_seen_mrad) unwrap_counters.max_abs_dR_seen_mrad = abs_dR;

  UnwrapLogSample sample{};
  sample.pos_L_raw_mrad = f_to_i16(pos_L_raw, RAD_TO_MRAD);
  sample.pos_R_raw_mrad = f_to_i16(pos_R_raw, RAD_TO_MRAD);
  sample.vel_L_raw_mrad_s = f_to_i16(vel_L_raw, RAD_TO_MRAD);
  sample.vel_R_raw_mrad_s = f_to_i16(vel_R_raw, RAD_TO_MRAD);
  sample.dL_mrad = f_to_i16(dbg.dL_wrapped, RAD_TO_MRAD);
  sample.dR_mrad = f_to_i16(dbg.dR_wrapped, RAD_TO_MRAD);
  sample.accept_flags = (dbg.accept_L ? 0x01u : 0u) | (dbg.accept_R ? 0x02u : 0u);
  sample.unwrap_L_mrad = f_to_i32(unwrap_L_rad, RAD_TO_MRAD);
  sample.unwrap_R_mrad = f_to_i32(unwrap_R_rad, RAD_TO_MRAD);
  sample.x_wheel_mm = f_to_i32(x_wheel_m, M_TO_MM);
  sample.x_dot_mm_s = f_to_i16(x_dot_m_s, VEL_M_TO_MM_S);
  sample.x_dot_from_pos_mm_s = f_to_i16(x_dot_from_pos, VEL_M_TO_MM_S);

  unwrap_log_ring[unwrap_log_head] = sample;
  unwrap_log_head = (uint16_t)((unwrap_log_head + 1u) % UNWRAP_LOG_CAPACITY);
  if (unwrap_log_count < UNWRAP_LOG_CAPACITY) {
    unwrap_log_count++;
  }
}

bool telemetry_start_unwrap_dump() {
  if (unwrap_dump_state.phase != DumpPhase::Idle) return false;

  const uint16_t count = unwrap_log_count;
  const uint16_t start_index = (uint16_t)((unwrap_log_head + UNWRAP_LOG_CAPACITY - count) % UNWRAP_LOG_CAPACITY);
  const uint32_t payload_bytes = (uint32_t)count * (uint32_t)sizeof(UnwrapLogSample);
  const uint32_t payload_crc = crc32_payload_oldest_to_newest(start_index, count);

  unwrap_dump_state = UnwrapDumpState{};
  unwrap_dump_state.sample_count = count;
  unwrap_dump_state.start_index = start_index;
  unwrap_dump_state.trailer_value = payload_crc;
  unwrap_dump_state.phase = DumpPhase::Header;

  UnwrapLogHeader &h = unwrap_dump_state.header;
  h.magic[0] = 'T';
  h.magic[1] = 'W';
  h.magic[2] = 'R';
  h.magic[3] = '1';
  h.version = UNWRAP_DUMP_VERSION;
  h.msg_type = UNWRAP_DUMP_MSG_TYPE;
  h.sample_rate_hz = UNWRAP_SAMPLE_RATE_HZ;
  h.sample_bytes = (uint16_t)sizeof(UnwrapLogSample);
  h.sample_count = count;
  h.start_index = (uint32_t)start_index;
  h.payload_bytes = payload_bytes;
  h.payload_crc32 = payload_crc;
  h.header_crc32 = 0u;

  return true;
}

bool telemetry_unwrap_dump_active() {
  return unwrap_dump_state.phase != DumpPhase::Idle;
}

bool telemetry_service_unwrap_dump(Stream &out, uint16_t max_bytes_per_call) {
  if (unwrap_dump_state.phase == DumpPhase::Idle) return true;
  if (max_bytes_per_call == 0) return false;

  uint32_t budget = max_bytes_per_call;
  while (budget > 0) {
    int avail = out.availableForWrite();
    uint32_t io_cap = (avail > 0) ? (uint32_t)avail : budget;

    // Optional: cap io_cap so you don't try insane writes on weird Streams.
    if (io_cap > budget) io_cap = budget;
    if (io_cap > 2048u) io_cap = 2048u; // keeps write sizes reasonable
    if (io_cap == 0) break;

    if (unwrap_dump_state.phase == DumpPhase::Header) {
      const uint8_t *src = (const uint8_t *)&unwrap_dump_state.header;
      const uint32_t total = (uint32_t)sizeof(UnwrapLogHeader);
      const uint32_t remaining = total - unwrap_dump_state.header_sent;
      uint32_t chunk = (remaining < io_cap) ? remaining : io_cap;
      size_t wrote = out.write(src + unwrap_dump_state.header_sent, (size_t)chunk);
      if (wrote == 0) break;
      unwrap_dump_state.header_sent += (uint32_t)wrote;
      budget -= (uint32_t)wrote;
      if (unwrap_dump_state.header_sent >= total) {
        unwrap_dump_state.phase = DumpPhase::Payload;
      }
      continue;
    }

    if (unwrap_dump_state.phase == DumpPhase::Payload) {
      if (unwrap_dump_state.sample_cursor >= unwrap_dump_state.sample_count) {
        unwrap_dump_state.phase = DumpPhase::Trailer;
        continue;
      }

      const uint16_t ring_idx = (uint16_t)((unwrap_dump_state.start_index + unwrap_dump_state.sample_cursor) % UNWRAP_LOG_CAPACITY);
      const uint8_t *src = (const uint8_t *)&unwrap_log_ring[ring_idx];
      const uint32_t sample_bytes = (uint32_t)sizeof(UnwrapLogSample);
      const uint32_t remaining = sample_bytes - unwrap_dump_state.sample_byte_offset;
      uint32_t chunk = (remaining < io_cap) ? remaining : io_cap;
      size_t wrote = out.write(src + unwrap_dump_state.sample_byte_offset, (size_t)chunk);
      if (wrote == 0) break;
      unwrap_dump_state.sample_byte_offset += (uint16_t)wrote;
      budget -= (uint32_t)wrote;
      if (unwrap_dump_state.sample_byte_offset >= sample_bytes) {
        unwrap_dump_state.sample_byte_offset = 0;
        unwrap_dump_state.sample_cursor++;
      }
      continue;
    }

    if (unwrap_dump_state.phase == DumpPhase::Trailer) {
      const uint8_t *src = (const uint8_t *)&unwrap_dump_state.trailer_value;
      const uint32_t total = (uint32_t)sizeof(uint32_t);
      const uint32_t remaining = total - unwrap_dump_state.trailer_sent;
      uint32_t chunk = (remaining < io_cap) ? remaining : io_cap;
      size_t wrote = out.write(src + unwrap_dump_state.trailer_sent, (size_t)chunk);
      if (wrote == 0) break;
      unwrap_dump_state.trailer_sent += (uint32_t)wrote;
      budget -= (uint32_t)wrote;
      if (unwrap_dump_state.trailer_sent >= total) {
        unwrap_dump_state.phase = DumpPhase::Idle;
        return true;
      }
      continue;
    }
  }

  return unwrap_dump_state.phase == DumpPhase::Idle;
}

void telemetry_dump_unwrap(Stream &out) {
  if (!telemetry_start_unwrap_dump()) return;
  while (!telemetry_service_unwrap_dump(out, 1024u)) {
    delay(1); // convert to delay(0)? 
  }
}
#else
void telemetry_dump_unwrap(Stream &out) {
  (void)out;
}
bool telemetry_start_unwrap_dump() { return false; }
bool telemetry_service_unwrap_dump(Stream &out, uint16_t max_bytes_per_call) {
  (void)out;
  (void)max_bytes_per_call;
  return true;
}
bool telemetry_unwrap_dump_active() { return false; }
#endif

// ---------------- Discrete LQR gains ----------------
// State ordering assumed: [theta, theta_dot, x_wheel, x_dot]^T
/*
static const float K_disc[4] = {
  8.69066899f,
  1.12293145f,
  -2.96754297f,
  -6.8f  
};
*/

static const float K_disc[4] = {
  9.0f,
  1.12f,
  -3.8f,
  -5.0f  
};
constexpr float WHEEL_RADIUS_M = 0.05278f; 

// ---------------- Control constants ----------------
constexpr float TORQUE_CLAMP   = 3.0f;    // max |Nm| per wheel
constexpr float SAFETY_SCALE   = 0.4f;   // global scaling (tune; set to 1.0f when confident)
constexpr float THETA_FAIL_RAD = 0.6f;    // ~34 deg: beyond this, bail to idle

static int report_counter = 0;

// Continuous wheel angle state
static bool  unwrap_init = false;
static float prev_L = 0.0f, prev_R = 0.0f;
static float unwrap_L = 0.0f, unwrap_R = 0.0f;

// Velocity filtering state
static float x_wheel_state_m = 0.0f;
static float x_dot_state_m_s = 0.0f;
static float prev_x_unwrap_m = 0.0f;
static bool  prev_x_unwrap_valid = false;
static uint32_t last_posvel_rx_used_L_us = 0;
static uint32_t last_posvel_rx_used_R_us = 0;

// One-shot per entry
static bool first_entry = true;
static uint32_t start_time_us = 0;

struct RunningStats {
  uint32_t n = 0;
  float mean = 0.0f;
  float m2 = 0.0f;

  void reset() { n = 0; mean = 0.0f; m2 = 0.0f; }
  void push(float x) {
    n++;
    float d = x - mean;
    mean += d / (float)n;
    float d2 = x - mean;
    m2 += d * d2;
  }
  float variance() const { return (n > 1) ? (m2 / (float)(n - 1)) : 0.0f; }
  float stddev() const { return sqrtf(variance()); }
};

static RunningStats pitch_rate_rms;
static RunningStats rate_rms_raw;
static RunningStats rate_rms_filt;
static int rms_counter = 0;

struct CanTxProofStats {
  uint32_t tx_attempts = 0;
  uint32_t tx_ok = 0;
  uint32_t tx_fail = 0;
  uint32_t last_report_us = 0;
};
static CanTxProofStats g_can_tx_proof;

// ---------------- Helper: update continuous wheel angles ----------------

static void updateWheelUnwrap(float pos_L_raw, float pos_R_raw,
                              float &x_wheel, float &x_dot,
                              float vel_L, float vel_R,
                              bool new_pos_L,
                              bool new_pos_R,
                              float dt_pos_L_s,
                              float dt_pos_R_s,
                              float dt_loop_s,
                              UnwrapDebugTick *dbg = nullptr)
{
  const float two_pi = 2.0f * PI;
  const float dt_loop_safe = (isfinite(dt_loop_s) && dt_loop_s > 0.0f) ? dt_loop_s : (CONTROL_PERIOD_US * 1e-6f);
  const float dt_gate_L = (isfinite(dt_pos_L_s) && dt_pos_L_s > 0.0f) ? dt_pos_L_s : dt_loop_safe;
  const float dt_gate_R = (isfinite(dt_pos_R_s) && dt_pos_R_s > 0.0f) ? dt_pos_R_s : dt_loop_safe;
  const bool new_pos_sample = new_pos_L || new_pos_R;
  constexpr float STEP_MARGIN_RAD = 0.03f;
  constexpr float STEP_MIN_RAD = 0.01f;
  constexpr float STEP_MAX_RAD = 0.60f;
  constexpr float VEL_TAU_S = 0.02f;

  // Reject non-finite position samples entirely for this tick.
  const bool posL_finite = isfinite(pos_L_raw);
  const bool posR_finite = isfinite(pos_R_raw);
  if (dbg) {
    *dbg = UnwrapDebugTick{};
    dbg->posL_finite = posL_finite;
    dbg->posR_finite = posR_finite;
    dbg->new_pos_sample = new_pos_sample;
    dbg->new_pos_L = new_pos_L;
    dbg->new_pos_R = new_pos_R;
    dbg->dt_gate_L_s = dt_gate_L;
    dbg->dt_gate_R_s = dt_gate_R;
    dbg->dt_gate_s = (new_pos_L && new_pos_R) ? (0.5f * (dt_gate_L + dt_gate_R))
                                               : (new_pos_L ? dt_gate_L : (new_pos_R ? dt_gate_R : dt_loop_safe));
  }

  // Initialize unwrap on first call in this mode
  if (!unwrap_init) {
    if (!posL_finite || !posR_finite) {
      x_wheel = x_wheel_state_m;
      x_dot = x_dot_state_m_s;
      return;
    }

    prev_L = pos_L_raw;
    prev_R = pos_R_raw;
    unwrap_L = 0.0f;
    unwrap_R = 0.0f;
    x_wheel_state_m = 0.0f;
    x_dot_state_m_s = 0.0f;
    prev_x_unwrap_m = 0.0f;
    prev_x_unwrap_valid = false;
    unwrap_init = true;
    if (dbg) dbg->unwrap_init_event = true;
  }

  if (new_pos_sample) {
    // Compute incremental angles with wrap handling only on fresh POSVEL samples.
    if (new_pos_L && posL_finite) {
      float dL = pos_L_raw - prev_L;
      while (dL >= PI) dL -= two_pi;
      while (dL < -PI) dL += two_pi;
      if (dbg) dbg->dL_wrapped = dL;

      float max_step_L = fabsf(vel_L) * dt_gate_L + STEP_MARGIN_RAD;
      if (max_step_L < STEP_MIN_RAD) max_step_L = STEP_MIN_RAD;
      if (max_step_L > STEP_MAX_RAD) max_step_L = STEP_MAX_RAD;
      if (dbg) dbg->max_step_L = max_step_L;

      if (fabsf(dL) <= max_step_L) {
        unwrap_L += dL;
        if (dbg) dbg->accept_L = true;
      }
      // Advance prev on both accept and reject to avoid lockout cascades.
      prev_L = pos_L_raw;
    }

    if (new_pos_R && posR_finite) {
      float dR = pos_R_raw - prev_R;
      while (dR >= PI) dR -= two_pi;
      while (dR < -PI) dR += two_pi;
      if (dbg) dbg->dR_wrapped = dR;

      float max_step_R = fabsf(vel_R) * dt_gate_R + STEP_MARGIN_RAD;
      if (max_step_R < STEP_MIN_RAD) max_step_R = STEP_MIN_RAD;
      if (max_step_R > STEP_MAX_RAD) max_step_R = STEP_MAX_RAD;
      if (dbg) dbg->max_step_R = max_step_R;

      if (fabsf(dR) <= max_step_R) {
        unwrap_R += dR;
        if (dbg) dbg->accept_R = true;
      }
      // Advance prev on both accept and reject to avoid lockout cascades.
      prev_R = pos_R_raw;
    }

    const float x_unwrap_m = 0.5f * (unwrap_L + unwrap_R) * WHEEL_RADIUS_M;
    float v_meas = x_dot_state_m_s;
    const float dt_for_v =
        (new_pos_L && new_pos_R) ? (0.5f * (dt_gate_L + dt_gate_R))
                                 : (new_pos_L ? dt_gate_L : dt_gate_R);
    if (prev_x_unwrap_valid && dt_for_v > 0.0f) {
      v_meas = (x_unwrap_m - prev_x_unwrap_m) / dt_for_v;
    }

    float alpha = dt_for_v / (VEL_TAU_S + dt_for_v);
    if (!isfinite(alpha) || alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    x_dot_state_m_s += alpha * (v_meas - x_dot_state_m_s);
    x_wheel_state_m = x_unwrap_m;
    prev_x_unwrap_m = x_unwrap_m;
    prev_x_unwrap_valid = true;
    if (dbg) dbg->v_meas_from_pos_m_s = v_meas;
  } else {
    // No fresh POS sample this tick: predict position forward with filtered velocity.
    x_wheel_state_m += x_dot_state_m_s * dt_loop_safe;
  }

  x_wheel = x_wheel_state_m;
  x_dot = x_dot_state_m_s;
}

bool test_pin_state = true;

// ---------------- Main TWR balance mode ----------------
void balance_TWR_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can)
{
  if (!sup) return;

  // Must have both ESCs alive before we attempt torque control
  if (!sup->esc[0].state.alive || !sup->esc[1].state.alive) {
    // If either ESC died mid-balance, immediately go idle
    first_entry = true;
    unwrap_init = false;
    x_wheel_state_m = 0.0f;
    x_dot_state_m_s = 0.0f;
    prev_x_unwrap_m = 0.0f;
    prev_x_unwrap_valid = false;
    last_posvel_rx_used_L_us = 0;
    last_posvel_rx_used_R_us = 0;

    // Send zero torque
    CAN_message_t msgL, msgR;
    msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);
    msgL.len = msgR.len = 8;
    msgL.flags.extended = msgR.flags.extended = 1;
    canPackFloat(0.0f, msgL.buf);
    canPackFloat(0.0f, msgL.buf + 4);
    canPackFloat(0.0f, msgR.buf);
    canPackFloat(0.0f, msgR.buf + 4);
    #ifdef SEND_TORQUE
      // prevent sending torque commands if ESCs are not alive;
      Serial.println("{\"cmd\":\"PRINT\",\"note\":\"ESC not alive: cutting torque and exiting to idle\"}");      
      can.write(msgL);
      can.write(msgR);
    #endif 
    Serial.println("balance exit: esc not alive -> idle");
    sup->mode = SUP_MODE_IDLE;
    return;
  }

  // ---------------- Initialize on first entry ----------------
  if (first_entry) {
    first_entry = false;
    unwrap_init = false;
    g_can_tx_proof = CanTxProofStats{};
    g_can_tx_proof.last_report_us = micros();
    start_time_us = micros();
    x_wheel_state_m = 0.0f;
    x_dot_state_m_s = 0.0f;
    prev_x_unwrap_m = 0.0f;
    prev_x_unwrap_valid = false;
    last_posvel_rx_used_L_us = 0;
    last_posvel_rx_used_R_us = 0;
    #ifdef SEND_LOG
    telemetry_reset_unwrap_log();
    #endif

    Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance mode started\"}");
  }

  // ---------------- Sensor feedback ----------------
  const uint32_t elapsed_us = micros() - start_time_us;

  // Body angle and rate from IMU (radians and rad/s)

  float theta     = sup->imu.pitch_rad - sup->balance_theta_target_rad;
  float theta_dot = sup->imu.pitch_rate;
  pitch_rate_rms.push(sup->imu.pitch_rate); 
  rate_rms_raw.push(sup->imu.pitch_rate_raw);
  rate_rms_filt.push(sup->imu.pitch_rate);

  // Wheel encoder positions (as reported by ESC, wrapped)
  float pos_L = sup->esc[0].state.pos_rad;
  float pos_R = sup->esc[1].state.pos_rad;

  float vel_L = sup->esc[0].state.vel_rad_s;
  float vel_R = sup->esc[1].state.vel_rad_s;

  // Control-loop tick and POS sample timing are independent.
  const float dt_loop_s = CONTROL_PERIOD_US * 1e-6f;
  const uint32_t pos_L_us = sup->esc[0].status.last_update_us;
  const uint32_t pos_R_us = sup->esc[1].status.last_update_us;
  const bool new_pos_L = (pos_L_us != 0u) && (pos_L_us != last_posvel_rx_used_L_us);
  const bool new_pos_R = (pos_R_us != 0u) && (pos_R_us != last_posvel_rx_used_R_us);
  float dt_pos_L_s = dt_loop_s;
  float dt_pos_R_s = dt_loop_s;
  if (new_pos_L) {
    if (last_posvel_rx_used_L_us != 0u) {
      dt_pos_L_s = (float)((uint32_t)(pos_L_us - last_posvel_rx_used_L_us)) * 1e-6f;
    } else {
      dt_pos_L_s = 0.002f;
    }
    last_posvel_rx_used_L_us = pos_L_us;
  }
  if (new_pos_R) {
    if (last_posvel_rx_used_R_us != 0u) {
      dt_pos_R_s = (float)((uint32_t)(pos_R_us - last_posvel_rx_used_R_us)) * 1e-6f;
    } else {
      dt_pos_R_s = 0.002f;
    }
    last_posvel_rx_used_R_us = pos_R_us;
  }

  float x_wheel = 0.0f;
  float x_dot   = 0.0f;
  UnwrapDebugTick unwrap_dbg;
  updateWheelUnwrap(pos_L, pos_R, x_wheel, x_dot, vel_L, vel_R,
                    new_pos_L, new_pos_R, dt_pos_L_s, dt_pos_R_s, dt_loop_s, &unwrap_dbg);

  #ifdef SEND_LOG
  if (unwrap_dbg.new_pos_sample) {
    telemetry_log_unwrap(pos_L, pos_R, vel_L, vel_R, unwrap_dbg,
                         unwrap_L, unwrap_R, x_wheel, x_dot, unwrap_dbg.dt_gate_s);
  }
  #endif

  // digitalWrite(TEST_PIN, test_pin_state);
  // test_pin_state = !test_pin_state;

  // ---------------- Safety: fall detection ----------------
  // If robot is too far from upright, cut torque and exit.
  if (fabsf(theta) > THETA_FAIL_RAD) {
    CAN_message_t msgL, msgR;
    msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);
    msgL.len = msgR.len = 8;
    msgL.flags.extended = msgR.flags.extended = 1;

    canPackFloat(0.0f, msgL.buf);
    canPackFloat(0.0f, msgL.buf + 4);
    canPackFloat(0.0f, msgR.buf);
    canPackFloat(0.0f, msgR.buf + 4);

    #ifdef SEND_TORQUE
    // prevent sending torque commands during a fall
    Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Fall detected: cutting torque and exiting to idle\"}");
    can.write(msgL);
    can.write(msgR);
    #endif
    
    Serial.println("balance exit: theta fail -> idle");
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    unwrap_init = false;
    return;
  }

  // ---------------- LQR control law ----------------
  // State vector: x = [theta, theta_dot, x_wheel, x_dot]^T
  float u = -(K_disc[0] * theta +
	      K_disc[1] * theta_dot +
	      K_disc[2] * x_wheel +
	      K_disc[3] * x_dot);

  // Global safety scaling (start small during tuning)
  u *= SAFETY_SCALE;

  // Clamp commanded torque
  if (u > TORQUE_CLAMP)  u = TORQUE_CLAMP;
  if (u < -TORQUE_CLAMP) u = -TORQUE_CLAMP;

  // ESC torque directions are mirrored.
  // Using opposite-sign commands produces same physical wheel torque (forward/back).
  float torque_left  =  -u;
  float torque_right = u;
  torque_left  =  2.0f;
  torque_right = -2.0f;

  // ---------------- Send torque over CAN ----------------
  CAN_message_t msgL, msgR;
  msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
  msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);

  msgL.len = 8;
  msgR.len = 8;
  msgL.flags.extended = 1;
  msgR.flags.extended = 1;

  canPackFloat(torque_left,  msgL.buf);
  canPackFloat(0.0f,         msgL.buf + 4);
  canPackFloat(torque_right, msgR.buf);
  canPackFloat(0.0f,         msgR.buf + 4);
    // send torque commands based on control law
#ifdef SEND_TORQUE
  // Serial.printf("{\"cmd\":\"TORQUE\",\"left\":%.3f,\"right\":%.3f}\r\n", torque_left, torque_right);
  const bool okL = can.write(msgL);
  const bool okR = can.write(msgR);
  g_can_tx_proof.tx_attempts += 2u;
  g_can_tx_proof.tx_ok += (okL ? 1u : 0u) + (okR ? 1u : 0u);
  g_can_tx_proof.tx_fail += (!okL ? 1u : 0u) + (!okR ? 1u : 0u);
  #if CAN_TX_PROOF_VERBOSE
  Serial.printf(
      "{\"cmd\":\"CAN_TXQ\",\"t_us\":%lu,\"okL\":%u,\"okR\":%u,\"idL\":\"0x%08lX\",\"idR\":\"0x%08lX\",\"left\":%.3f,\"right\":%.3f}\r\n",
      micros(),
      okL ? 1u : 0u,
      okR ? 1u : 0u,
      (unsigned long)msgL.id,
      (unsigned long)msgR.id,
      torque_left,
      torque_right);
  #endif
#endif

  #if CAN_TX_PROOF_SUMMARY
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - g_can_tx_proof.last_report_us) >= 1000000u) {
    const uint32_t attempts = g_can_tx_proof.tx_attempts;
    const float fail_pct = (attempts > 0u)
                             ? (100.0f * (float)g_can_tx_proof.tx_fail / (float)attempts)
                             : 0.0f;
    const uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
    const uint32_t posvel_age_us =
        (last_posvel_rx_us > 0u) ? (uint32_t)(now_us - last_posvel_rx_us) : UINT32_MAX;
    Serial.printf(
        "{\"cmd\":\"CAN_TXQ_SUM\",\"attempts\":%lu,\"ok\":%lu,\"fail\":%lu,\"fail_pct\":%.3f,\"mode\":%d,\"posvel_age_us\":%lu}\r\n",
        (unsigned long)g_can_tx_proof.tx_attempts,
        (unsigned long)g_can_tx_proof.tx_ok,
        (unsigned long)g_can_tx_proof.tx_fail,
        fail_pct,
        (int)sup->mode,
        (unsigned long)posvel_age_us);
    g_can_tx_proof.last_report_us += 1000000u; // keep stable cadence
  }
  #endif

  // ---------------- Telemetry ----------------
  if (++report_counter >= TELEMETRY_DECIMATE) {
    report_counter = 0;

    float pitch_deg      = sup->imu.pitch_rad * 180.0f / PI;
    float pitch_rate_deg = sup->imu.pitch_rate * 180.0f / PI;
    uint32_t age_us = micros() - sup->imu.last_update_us;
    const uint32_t loop_dt_us = sup->timing.dt_us;
    const float loop_hz = (loop_dt_us > 0u) ? (1000000.0f / (float)loop_dt_us) : 0.0f;
    const int32_t dt_err_from_500hz_us = (int32_t)loop_dt_us - 2000;  // 500 Hz target = 2000 us
    const int dt_ok_500hz = (abs(dt_err_from_500hz_us) <= 200) ? 1 : 0; // +/-10% window
			    
    #ifdef SEND_TELEMETRY
    Serial.printf(
		  "{\"t\":%lu,"
		  "\"pitch\":%.3f,"
		  "\"rate\":%.3f,"
		  "\"pos_L_raw\":%.6f,"
		  "\"pos_R_raw\":%.6f,"
		  "\"vel_L_raw\":%.6f,"
		  "\"vel_R_raw\":%.6f,"
		  "\"new_pos\":%d,"
		  "\"new_pos_L\":%d,"
		  "\"new_pos_R\":%d,"
		  "\"dt_pos_us\":%lu,"
		  "\"dt_pos_L_us\":%lu,"
		  "\"dt_pos_R_us\":%lu,"
		  "\"dL\":%.6f,"
		  "\"dR\":%.6f,"
		  "\"max_step_L\":%.6f,"
		  "\"max_step_R\":%.6f,"
		  "\"accept_L\":%d,"
		  "\"accept_R\":%d,"
		  "\"v_meas_pos\":%.6f,"
		  "\"x_wheel\":%.6f,"
		  "\"x_dot\":%.6f,"
		  "\"loop_dt_us\":%lu,"
		  "\"loop_hz\":%.2f,"
		  "\"dt_err_500hz_us\":%ld,"
		  "\"dt_ok_500hz\":%d,"
		  "\"rms_raw\":%.6f,"
		  "\"rms_filt\":%.6f,"
		  "\"n_rms\":%lu,"
		  "\"age_us\":%lu,"
		  "\"valid\":%d,"
		  "\"exec_us\":%lu,"
		  "\"ovr\":%lu}\r\n",
		  micros(),
		  sup->imu.pitch_rad * 180.0f / PI,
		  sup->imu.pitch_rate * 180.0f / PI,
		  pos_L,                         // ESC raw POSVEL position [rad]
		  pos_R,                         // ESC raw POSVEL position [rad]
		  vel_L,                         // ESC raw POSVEL velocity [rad/s]
		  vel_R,                         // ESC raw POSVEL velocity [rad/s]
		  unwrap_dbg.new_pos_sample ? 1 : 0,
		  unwrap_dbg.new_pos_L ? 1 : 0,
		  unwrap_dbg.new_pos_R ? 1 : 0,
		  (unsigned long)lroundf(unwrap_dbg.dt_gate_s * 1e6f),
		  (unsigned long)lroundf(unwrap_dbg.dt_gate_L_s * 1e6f),
		  (unsigned long)lroundf(unwrap_dbg.dt_gate_R_s * 1e6f),
		  unwrap_dbg.dL_wrapped,
		  unwrap_dbg.dR_wrapped,
		  unwrap_dbg.max_step_L,
		  unwrap_dbg.max_step_R,
		  unwrap_dbg.accept_L ? 1 : 0,
		  unwrap_dbg.accept_R ? 1 : 0,
		  unwrap_dbg.v_meas_from_pos_m_s,
		  x_wheel,
		  x_dot,
		  (unsigned long)loop_dt_us,
		  loop_hz,
		  (long)dt_err_from_500hz_us,
		  dt_ok_500hz,
		  rate_rms_raw.stddev(),          // raw gyro RMS (rad/s)
		  rate_rms_filt.stddev(),         // filtered RMS (rad/s)
		  (unsigned long)rate_rms_filt.n, // sample count
		  age_us,
		  sup->imu.valid ? 1 : 0,
		  sup->timing.exec_time_us,
		  sup->timing.overruns
		  );
    #endif

    // reset window (so this RMS corresponds to the last ~0.1s at 10Hz printing)
    pitch_rate_rms.reset();
    rate_rms_raw.reset();
    rate_rms_filt.reset();
  }

  // Optional timed run stop. Dump is sent later from main() while idle.
  if (sup->user_total_us > 0 && elapsed_us > sup->user_total_us) {
    Serial.printf("balance exit: timed stop -> idle (elapsed=%lu us, limit=%lu us)\r\n",
                  (unsigned long)elapsed_us,
                  (unsigned long)sup->user_total_us);
    sup->mode = SUP_MODE_IDLE;
    if (telemetry_start_unwrap_dump()) {
      Serial.println("unwrap dump: queued");
    } else {
      Serial.println("unwrap dump: not started (busy or SEND_LOG disabled)");
    }
    first_entry = true;
    unwrap_init = false;
  }

}
