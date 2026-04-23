#include "telemetry_link.h"

#include <stddef.h>
#include <string.h>

namespace {

// Ring uses one slot as the empty/full discriminator, so use 5001 capacity
// to hold a full 5000-packet burst test without dropping one.
static constexpr uint16_t FAST_QUEUE_SIZE = 5001u;
static constexpr uint16_t EVENT_QUEUE_SIZE = 64u;
static constexpr uint8_t FAST_DECIMATE = 1u;
static constexpr bool EVENT_LOGGING_ENABLED = false;
static constexpr uint16_t EVENT_STATUS_COMPARE_MASK =
    (uint16_t)~(uint16_t)(TELEMETRY_STATUS_TX_DUE);

struct PacketQueue {
  TelemetryPacket *buf = nullptr;
  volatile uint16_t head = 0u;
  volatile uint16_t tail = 0u;
  uint16_t capacity = FAST_QUEUE_SIZE;
};

struct EventQueue {
  TelemetryPacket buf[EVENT_QUEUE_SIZE];
  volatile uint16_t head = 0u;
  volatile uint16_t tail = 0u;
  uint16_t capacity = EVENT_QUEUE_SIZE;
};

static PacketQueue g_fast_q;
static EventQueue g_event_q;
DMAMEM static TelemetryPacket g_fast_storage[FAST_QUEUE_SIZE];
static HardwareSerial *g_telem_serial = nullptr;
static uint32_t g_seq_counter = 0u;
static uint8_t g_fast_decimate_counter = 0u;
static uint16_t g_last_status_flags = 0u;
static uint32_t g_last_state_bits = 0u;

static TelemetryPacket g_tx_packet = {};
static bool g_tx_active = false;
static uint16_t g_tx_offset = 0u;

static inline uint16_t next_index(uint16_t idx, uint16_t capacity) {
  idx++;
  if (idx >= capacity) idx = 0u;
  return idx;
}

static inline bool fast_q_is_full() {
  return next_index(g_fast_q.head, g_fast_q.capacity) == g_fast_q.tail;
}

static inline bool fast_q_is_empty() {
  return g_fast_q.head == g_fast_q.tail;
}

static inline bool event_q_is_full() {
  return next_index(g_event_q.head, g_event_q.capacity) == g_event_q.tail;
}

static inline bool event_q_is_empty() {
  return g_event_q.head == g_event_q.tail;
}

static inline uint16_t q_count(uint16_t head, uint16_t tail, uint16_t capacity) {
  if (head >= tail) return (uint16_t)(head - tail);
  return (uint16_t)(capacity - (tail - head));
}

static void fast_q_drop_oldest() {
  if (!fast_q_is_empty()) {
    g_fast_q.tail = next_index(g_fast_q.tail, g_fast_q.capacity);
  }
}

static void event_q_drop_oldest() {
  if (!event_q_is_empty()) {
    g_event_q.tail = next_index(g_event_q.tail, g_event_q.capacity);
  }
}

static void fast_q_push_drop_oldest(const TelemetryPacket &pkt) {
  if (fast_q_is_full()) {
    fast_q_drop_oldest();
  }
  if (g_fast_q.buf == nullptr) return;
  g_fast_q.buf[g_fast_q.head] = pkt;
  g_fast_q.head = next_index(g_fast_q.head, g_fast_q.capacity);
}

static void event_q_push_prioritized(const TelemetryPacket &pkt) {
  if (event_q_is_full()) {
    if (!fast_q_is_empty()) {
      fast_q_drop_oldest();
    } else {
      event_q_drop_oldest();
    }
  }
  g_event_q.buf[g_event_q.head] = pkt;
  g_event_q.head = next_index(g_event_q.head, g_event_q.capacity);
}

static bool event_q_pop(TelemetryPacket *out) {
  if (event_q_is_empty() || out == nullptr) return false;
  *out = g_event_q.buf[g_event_q.tail];
  g_event_q.tail = next_index(g_event_q.tail, g_event_q.capacity);
  return true;
}

static bool fast_q_pop(TelemetryPacket *out) {
  if (g_fast_q.buf == nullptr || fast_q_is_empty() || out == nullptr) return false;
  *out = g_fast_q.buf[g_fast_q.tail];
  g_fast_q.tail = next_index(g_fast_q.tail, g_fast_q.capacity);
  return true;
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFu;
  if (data == nullptr) return crc;
  for (size_t i = 0u; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0u; b < 8u; ++b) {
      if ((crc & 0x8000u) != 0u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static TelemetryPacket make_packet(uint16_t type,
                                   uint32_t timestamp_us,
                                   uint16_t status_flags,
                                   uint32_t state_bits,
                                   const float payload[TELEMETRY_PAYLOAD_FLOATS]) {
  TelemetryPacket pkt = {};
  pkt.seq = ++g_seq_counter;
  pkt.timestamp_us = timestamp_us;
  pkt.type = type;
  pkt.status_flags = status_flags;
  memcpy(pkt.payload, payload, sizeof(pkt.payload));
  pkt.state_bits = state_bits;
  pkt.footer = TELEMETRY_PACKET_FOOTER;
  pkt.checksum = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt),
                             offsetof(TelemetryPacket, checksum));
  return pkt;
}

static void fill_fast_payload(const BalanceTelemetrySample &s,
                              float payload[TELEMETRY_PAYLOAD_FLOATS]) {
  payload[0] = s.dt_s;
  payload[1] = s.theta;
  payload[2] = s.theta_dot;
  payload[3] = s.theta_err;
  payload[4] = s.x_wheel_m;
  payload[5] = s.x_dot_m_s;
  payload[6] = s.x_pos_err_m;
  payload[7] = s.tau_l_cmd;
  payload[8] = s.tau_r_cmd;
  payload[9] = s.u_sync;
  payload[10] = s.u_i;
}

static uint32_t compose_state_bits(const BalanceTelemetrySample &s) {
  const uint32_t base = (s.state_bits & ~TELEMETRY_STATE_EVENT_CODE_MASK);
  const uint32_t event = ((uint32_t)(s.event_code & 0xFFu) << TELEMETRY_STATE_EVENT_CODE_SHIFT);
  return (base | event);
}

static void queue_event_if_needed(const BalanceTelemetrySample &s) {
  const uint16_t filtered_status = (uint16_t)(s.status_flags & EVENT_STATUS_COMPARE_MASK);
  const bool status_changed = (filtered_status != g_last_status_flags);
  const bool state_changed = (s.state_bits != g_last_state_bits);
  const bool has_event = (s.event_code != TELEMETRY_EVENT_NONE);
  g_last_status_flags = filtered_status;
  g_last_state_bits = s.state_bits;
  if (!status_changed && !state_changed && !has_event) return;

  float payload[TELEMETRY_PAYLOAD_FLOATS] = {0.0f};
  fill_fast_payload(s, payload);
  const TelemetryPacket pkt =
      make_packet(TELEMETRY_TYPE_EVENT, s.timestamp_us, s.status_flags, compose_state_bits(s), payload);
  event_q_push_prioritized(pkt);
}

} // namespace

void telemetry_link_init(HardwareSerial &serial_port, uint32_t baud) {
  g_telem_serial = &serial_port;
  g_telem_serial->begin(baud);
  g_fast_q.buf = g_fast_storage;
  g_fast_q.head = g_fast_q.tail = 0u;
  g_event_q.head = g_event_q.tail = 0u;
  g_seq_counter = 0u;
  g_fast_decimate_counter = 0u;
  g_last_status_flags = 0u;
  g_last_state_bits = 0u;
  g_tx_active = false;
  g_tx_offset = 0u;
}

void telemetry_publish_balance_tick(const BalanceTelemetrySample &sample) {
  // EVENT telemetry is intentionally disabled while validating FAST continuity.
  if (EVENT_LOGGING_ENABLED) {
    queue_event_if_needed(sample);
  }

  g_fast_decimate_counter++;
  if (g_fast_decimate_counter < FAST_DECIMATE) return;
  g_fast_decimate_counter = 0u;

  float payload[TELEMETRY_PAYLOAD_FLOATS] = {0.0f};
  fill_fast_payload(sample, payload);
  const TelemetryPacket pkt =
      make_packet(TELEMETRY_TYPE_FAST, sample.timestamp_us, sample.status_flags, compose_state_bits(sample), payload);
  fast_q_push_drop_oldest(pkt);
}

void telemetry_uart_pump() {
  if (g_telem_serial == nullptr) return;

  const uint16_t total = (uint16_t)sizeof(TelemetryPacket);
  while (true) {
    if (!g_tx_active) {
      if (event_q_pop(&g_tx_packet) || fast_q_pop(&g_tx_packet)) {
        g_tx_active = true;
        g_tx_offset = 0u;
      } else {
        return;
      }
    }

    const int writable = g_telem_serial->availableForWrite();
    if (writable <= 0) return;

    const uint8_t *raw = reinterpret_cast<const uint8_t*>(&g_tx_packet);
    const uint16_t remaining = (uint16_t)(total - g_tx_offset);
    uint16_t chunk = (uint16_t)writable;
    if (chunk > remaining) chunk = remaining;
    if (chunk == 0u) return;

    const size_t written = g_telem_serial->write(raw + g_tx_offset, chunk);
    if (written == 0u) return;

    g_tx_offset = (uint16_t)(g_tx_offset + (uint16_t)written);
    if (g_tx_offset >= total) {
      g_tx_active = false;
      g_tx_offset = 0u;
    }
  }
}

bool telemetry_pop_next_packet(TelemetryPacket *out) {
  if (out == nullptr) return false;
  if (event_q_pop(out)) return true;
  return fast_q_pop(out);
}

bool telemetry_has_pending() {
  return telemetry_pending_bytes() > 0u;
}

uint32_t telemetry_pending_packets() {
  const uint32_t queued_fast = (uint32_t)q_count(g_fast_q.head, g_fast_q.tail, g_fast_q.capacity);
  const uint32_t queued_event = (uint32_t)q_count(g_event_q.head, g_event_q.tail, g_event_q.capacity);
  const uint32_t in_flight = g_tx_active ? 1u : 0u;
  return queued_fast + queued_event + in_flight;
}

uint32_t telemetry_pending_bytes() {
  const uint32_t packet_size = (uint32_t)sizeof(TelemetryPacket);
  const uint32_t queued_fast = (uint32_t)q_count(g_fast_q.head, g_fast_q.tail, g_fast_q.capacity);
  const uint32_t queued_event = (uint32_t)q_count(g_event_q.head, g_event_q.tail, g_event_q.capacity);
  const uint32_t queued_bytes = (queued_fast + queued_event) * packet_size;
  const uint32_t inflight_bytes =
      g_tx_active ? (uint32_t)((uint16_t)sizeof(TelemetryPacket) - g_tx_offset) : 0u;
  return queued_bytes + inflight_bytes;
}

uint32_t telemetry_packet_size_bytes() {
  return (uint32_t)sizeof(TelemetryPacket);
}
