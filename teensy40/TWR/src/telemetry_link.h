#pragma once

#include <Arduino.h>
#include <stdint.h>

// Packet types for the wire protocol.
enum TelemetryPacketType : uint16_t {
  TELEMETRY_TYPE_FAST = 1u,
  TELEMETRY_TYPE_EVENT = 2u,
  TELEMETRY_TYPE_DEBUG = 3u
};

// Status bits (bitwise OR into BalanceTelemetrySample::status_flags).
enum TelemetryStatusFlags : uint16_t {
  TELEMETRY_STATUS_IMU_FRESH = (1u << 0),
  TELEMETRY_STATUS_ESC_L_FRESH = (1u << 1),
  TELEMETRY_STATUS_ESC_R_FRESH = (1u << 2),
  TELEMETRY_STATUS_POS_RANGE_OK = (1u << 3),
  TELEMETRY_STATUS_POS_STEP_OK = (1u << 4),
  TELEMETRY_STATUS_TX_ENABLED = (1u << 5),
  TELEMETRY_STATUS_TX_DUE = (1u << 6),
  TELEMETRY_STATUS_HOLD_MODE = (1u << 7),
  TELEMETRY_STATUS_X_HOLD_ENABLED = (1u << 8),
  TELEMETRY_STATUS_FAIL_SAFE = (1u << 9)
};

// Event codes for low-rate prioritized event packets.
enum TelemetryEventCode : uint16_t {
  TELEMETRY_EVENT_NONE = 0u,
  TELEMETRY_EVENT_STALE_IMU = 1u,
  TELEMETRY_EVENT_STALE_ESC_L = 2u,
  TELEMETRY_EVENT_STALE_ESC_R = 3u,
  TELEMETRY_EVENT_POS_RANGE_FAIL = 4u,
  TELEMETRY_EVENT_POS_STEP_FAIL = 5u,
  TELEMETRY_EVENT_THETA_FAIL = 6u,
  TELEMETRY_EVENT_VEL_FAIL = 7u,
  TELEMETRY_EVENT_BALANCE_ACTIVE = 8u
};

static constexpr uint16_t TELEMETRY_PACKET_FOOTER = 0xDEADu;
static constexpr size_t TELEMETRY_PAYLOAD_FLOATS = 11u;

// Compact boolean/metadata bits sent per packet in TelemetryPacket::state_bits.
enum TelemetryStateBits : uint32_t {
  TELEMETRY_STATE_X_HOLD_ENABLED = (1u << 0),
  TELEMETRY_STATE_IS_TARED = (1u << 1),
  TELEMETRY_STATE_ESC_COMMS_OK = (1u << 2),
  TELEMETRY_STATE_STABLE_ON_KICKSTAND = (1u << 3)
};

static constexpr uint32_t TELEMETRY_STATE_EVENT_CODE_SHIFT = 8u;
static constexpr uint32_t TELEMETRY_STATE_EVENT_CODE_MASK = (0xFFu << TELEMETRY_STATE_EVENT_CODE_SHIFT);

// Packed fixed-size telemetry packet.
struct __attribute__((packed)) TelemetryPacket {
  uint32_t seq;
  uint32_t timestamp_us;
  uint16_t type;
  uint16_t status_flags;
  float payload[TELEMETRY_PAYLOAD_FLOATS];
  uint32_t state_bits;
  uint16_t checksum; // CRC16-CCITT of all bytes before checksum/footer.
  uint16_t footer;   // TELEMETRY_PACKET_FOOTER
};

static_assert(sizeof(TelemetryPacket) == 64u, "TelemetryPacket must stay fixed-size (64 bytes)");

// Compact fast-path balance sample.
struct BalanceTelemetrySample {
  uint32_t timestamp_us;
  uint32_t state_bits;
  uint16_t status_flags;
  uint16_t event_code;
  float dt_s;
  float theta;
  float theta_dot;
  float theta_err;
  float x_wheel_m;
  float x_dot_m_s;
  float x_pos_err_m;
  float tau_l_cmd;
  float tau_r_cmd;
  float u_sync;
  float u_i;
};

void telemetry_link_init(HardwareSerial &serial_port, uint32_t baud);
void telemetry_publish_balance_tick(const BalanceTelemetrySample &sample);
void telemetry_uart_pump();
bool telemetry_pop_next_packet(TelemetryPacket *out);
bool telemetry_has_pending();
uint32_t telemetry_pending_packets();
uint32_t telemetry_pending_bytes();
uint32_t telemetry_packet_size_bytes();
