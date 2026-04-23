#include <FlexCAN_T4.h>
#include "LED.h"
#include <ArduinoJson.h>
#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <SerialTransfer.h>
#include "main.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "tones.h"
#include "ICM42688.h"
#include "ESC.h"
#include "CAN_helper.h"
#include "supervisor.h"
#include "test_can_transmit_mode.h"
#include "balance_TWR_mode.h"
#include "balance_debug_mode.h"
#include "console_commands.h"
#include "telemetry_link.h"
#include "IMU_helper.h"

// Compile-time A/B selector for balance mode entry routing.
// 0 -> enter normal balance_TWR_mode()
// 1 -> enter balance_debug_mode() clone
#ifndef BALANCE_USE_DEBUG_MODE
#define BALANCE_USE_DEBUG_MODE 0
#endif

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;
Supervisor_typedef supervisor;

const char* esc_names[]   = {"left", "right"};
const uint16_t esc_ids[]  = {11, 12}; // node_ids of the ESCs
const uint8_t rc_pins[]   = {RC_INPUT1, RC_INPUT2, RC_INPUT3};
static constexpr uint8_t RC_PIN_COUNT = sizeof(rc_pins) / sizeof(rc_pins[0]);


// -------------------- CAN Communication --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
CANBuffer canRxBuf1;  // CAN1 software RX queue
CANBuffer canRxBuf2;  // CAN2 software RX queue

// CAN reliability improvement:
// Push RX frames from FIFO ISR callbacks into software queues immediately.
static void can1_on_receive(const CAN_message_t &msg) {
  canNoteBusRead(1u);
  canNotePosvelIngress(msg);
  if (!canBufferPush(canRxBuf1, msg, 1u)) {
    canNoteRxOverflow();
  }
}

static void can2_on_receive(const CAN_message_t &msg) {
  canNoteBusRead(2u);
  canNotePosvelIngress(msg);
  if (!canBufferPush(canRxBuf2, msg, 2u)) {
    canNoteRxOverflow();
  }
}

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
static constexpr uint32_t BALANCE_EXIT_BEEP_HZ = 2200u;
static constexpr uint32_t BALANCE_EXIT_BEEP_MS = 30u;
static constexpr uint32_t BALANCE_EXIT_BEEP_GAP_MS = 0u;
PushButton g_button(PUSHBUTTON_PIN, true, 50000u);
ICM42688 imu(SPI, CS_PIN);
static constexpr uint32_t CAN_POSVEL_RX_TIMEOUT_US = 400000u;
static constexpr uint32_t CAN_TEST_RUN_DEFAULT_US = 30000000u;  // 30 seconds
static constexpr uint32_t BALANCE_BUTTON_RUN_US = 0u;  // 0 = no auto-timeout
static constexpr uint32_t BALANCE_BUTTON_STOP_GUARD_US = 800000u; // Ignore stop press for first 0.8 s.
static constexpr uint16_t CAL_KICKSTAND_SAMPLES = 100u;
static constexpr float CAL_TARGET_DELTA_RAD = -1.270f;
static constexpr float CAL_TARGET_TOL_RAD = 0.020f;
static constexpr float CAL_RATE_MAX_RAD_S = 0.10f;
static constexpr uint32_t CAL_HOLD_US = 300000u;
static constexpr uint32_t CAL_AX_PRINT_PERIOD_US = 20000u; // 50 Hz while target reached
static constexpr uint32_t IDLE_LONG_HOLD_US = 2000000u; // 2 seconds
static constexpr uint32_t SERIALTRANSFER_SEND_PERIOD_US = 10000u; // 100 Hz
static constexpr uint32_t SERIALTRANSFER_ACK_TIMEOUT_US = 20000u;
static uint32_t g_balance_mode_enter_us = 0u;
static SupervisorMode g_prev_mode_for_exit_tweet = SUP_MODE_IDLE;
static SupervisorMode g_prev_mode_for_telem_flush = SUP_MODE_IDLE;
static bool g_telem_flush_after_balance_twr = false;
static uint32_t g_telem_flush_start_us = 0u;
static uint32_t g_telem_flush_start_packets = 0u;
static uint32_t g_telem_flush_start_bytes = 0u;
static uint16_t g_cal_kickstand_count = 0u;
static float g_cal_kickstand_sum = 0.0f;
static float g_cal_kickstand_pitch = 0.0f;
static bool g_cal_kickstand_ready = false;
static float g_cal_theta_eq_target = 0.0f;
static uint32_t g_cal_hold_start_us = 0u;
static bool g_cal_target_in_window = false;
static bool g_cal_target_reached_announced = false;
static uint32_t g_cal_last_ax_print_us = 0u;
static bool g_idle_long_hold_tracking = false;
static uint32_t g_idle_long_hold_start_us = 0u;
static bool g_idle_long_hold_fired = false;
static SerialTransfer g_telem_transfer;
static uint32_t g_telem_transfer_last_send_us = 0u;
static uint32_t g_telem_transfer_seq = 0u;
static uint32_t g_telem_flush_last_tx_us = 0u;
static uint32_t g_telem_flush_sent_packets = 0u;
static uint32_t g_telem_flush_acked_packets = 0u;
static uint32_t g_telem_flush_retries = 0u;
static uint32_t g_telem_flush_outstanding_seq = 0u;
static bool g_telem_flush_waiting_ack = false;
static TelemetryPacket g_telem_flush_outstanding_pkt = {};
// Temporary debug stream: while in balance mode, print raw accel vs filtered pitch.
static constexpr bool TEMP_PRINT_BALANCE_IMU_ENABLE = false;
static constexpr uint32_t TEMP_PRINT_BALANCE_IMU_PERIOD_US = 10000u; // 100 Hz
static uint32_t g_last_balance_imu_print_us = 0u;
// Runtime decimated printing can perturb timing; keep off for measurement runs.
// Uncomment to ignore pushbutton state transitions.
// #define PB_OVERRIDE

static inline void reset_calibration_state() {
  g_cal_kickstand_count = 0u;
  g_cal_kickstand_sum = 0.0f;
  g_cal_kickstand_pitch = 0.0f;
  g_cal_kickstand_ready = false;
  g_cal_theta_eq_target = 0.0f;
  g_cal_hold_start_us = 0u;
  g_cal_target_in_window = false;
  g_cal_target_reached_announced = false;
  g_cal_last_ax_print_us = 0u;
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

static void sendDatum(const TelemetryPacket &pkt) {
  g_telem_transfer.sendDatum(pkt);
}

struct __attribute__((packed)) TelemetryAck {
  uint32_t seq;
  uint16_t code;
  uint16_t reserved;
};

static void telemetry_serialtransfer_flush_tick(uint32_t now_us) {
#if TELEMETRY_SERIALTRANSFER_TX_ENABLE
  // Drain inbound ACKs first.
  static constexpr uint8_t MAX_RX_PER_TICK = 8u;
  for (uint8_t i = 0u; i < MAX_RX_PER_TICK; ++i) {
    if (!g_telem_transfer.available()) break;

    if (g_telem_transfer.bytesRead == sizeof(TelemetryAck)) {
      TelemetryAck ack = {};
      uint16_t rx_size = 0u;
      rx_size = g_telem_transfer.rxObj(ack, rx_size);
      if (rx_size == sizeof(TelemetryAck) &&
          g_telem_flush_waiting_ack &&
          ack.seq == g_telem_flush_outstanding_seq) {
        g_telem_flush_waiting_ack = false;
        g_telem_flush_acked_packets++;
      }
    }
  }

  if (!g_telem_flush_after_balance_twr || supervisor.mode != SUP_MODE_IDLE) return;

  if (g_telem_flush_waiting_ack) {
    if ((uint32_t)(now_us - g_telem_flush_last_tx_us) >= SERIALTRANSFER_ACK_TIMEOUT_US) {
      sendDatum(g_telem_flush_outstanding_pkt);
      g_telem_flush_last_tx_us = now_us;
      g_telem_flush_retries++;
    }
    return;
  }

  TelemetryPacket next_pkt = {};
  if (telemetry_pop_next_packet(&next_pkt)) {
    g_telem_flush_outstanding_pkt = next_pkt;
    g_telem_flush_outstanding_seq = next_pkt.seq;
    sendDatum(g_telem_flush_outstanding_pkt);
    g_telem_flush_last_tx_us = now_us;
    g_telem_flush_sent_packets++;
    g_telem_flush_waiting_ack = true;
    return;
  }

  g_telem_flush_after_balance_twr = false;
  const uint32_t end_packets = telemetry_pending_packets();
  const uint32_t end_bytes = telemetry_pending_bytes();
  const uint32_t elapsed_us = (uint32_t)(micros() - g_telem_flush_start_us);
  Serial.printf(
      "{\"cmd\":\"TELEM_FLUSH_END\",\"driver\":\"SerialTransfer\",\"packet_size_bytes\":%lu,\"start_packets\":%lu,\"start_bytes\":%lu,\"remaining_packets\":%lu,\"remaining_bytes\":%lu,\"sent_packets\":%lu,\"acked_packets\":%lu,\"retries\":%lu,\"elapsed_us\":%lu}\r\n",
      (unsigned long)telemetry_packet_size_bytes(),
      (unsigned long)g_telem_flush_start_packets,
      (unsigned long)g_telem_flush_start_bytes,
      (unsigned long)end_packets,
      (unsigned long)end_bytes,
      (unsigned long)g_telem_flush_sent_packets,
      (unsigned long)g_telem_flush_acked_packets,
      (unsigned long)g_telem_flush_retries,
      (unsigned long)elapsed_us);
#else
  (void)now_us;
#endif
}

static inline void telemetry_serialtransfer_send_100hz(uint32_t now_us) {
#if TELEMETRY_SERIALTRANSFER_TX_ENABLE
  if ((uint32_t)(now_us - g_telem_transfer_last_send_us) < SERIALTRANSFER_SEND_PERIOD_US) return;
  g_telem_transfer_last_send_us = now_us;

  TelemetryPacket pkt = {};
  pkt.seq = ++g_telem_transfer_seq;
  pkt.timestamp_us = now_us;
  pkt.type = TELEMETRY_TYPE_FAST;
  pkt.status_flags = 0u;
  pkt.payload[0] = (supervisor.esc_count > 0u) ? supervisor.esc[0].state.bus_voltage : NAN;
  pkt.payload[1] = (supervisor.esc_count > 1u) ? supervisor.esc[1].state.bus_voltage : NAN;
  pkt.state_bits = (uint32_t)supervisor.mode;
  pkt.footer = TELEMETRY_PACKET_FOOTER;
  pkt.checksum = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt),
                             offsetof(TelemetryPacket, checksum));
  sendDatum(pkt);
#else
  (void)now_us;
#endif
}

static inline bool is_balance_mode(SupervisorMode m) {
  return (m == SUP_MODE_BALANCE_HOLD) ||
         (m == SUP_MODE_BALANCE_TWR) ||
         (m == SUP_MODE_BALANCE_DEBUG);
}

static inline SupervisorMode selected_balance_entry_mode() {
#if BALANCE_USE_DEBUG_MODE
  return SUP_MODE_BALANCE_DEBUG;
#else
  return SUP_MODE_BALANCE_HOLD;
#endif
}

static inline void dump_selected_balance_mode_on_exit(SupervisorMode mode, const char *reason) {
  if (mode == SUP_MODE_BALANCE_DEBUG) {
    balance_debug_dump_on_mode_exit(reason);
  } else {
    balance_TWR_dump_on_mode_exit(reason);
  }
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

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

void setup() {

  Serial.begin(921600);
  while (!Serial && millis() < 1500) {}
  // CAN2 uses pins 0/1 on Teensy 4.0, so avoid Serial1 on those same pins.
#if TELEMETRY_SERIALTRANSFER_TX_ENABLE
  // Initialize telemetry queues used by balance_TWR_mode logging.
  telemetry_link_init(Serial4, TELEMETRY_SERIALTRANSFER_BAUD);
  Serial4.begin(TELEMETRY_SERIALTRANSFER_BAUD);
  g_telem_transfer.begin(Serial4, false);
#if TELEMETRY_SERIALTRANSFER_CONTINUOUS_TX_ENABLE
  Serial.printf("{\"cmd\":\"TELEM_LINK_CFG\",\"enabled\":1,\"driver\":\"SerialTransfer\",\"port\":\"Serial4\",\"baud\":%lu,\"mode\":\"continuous\",\"rate_hz\":100}\r\n",
                (unsigned long)TELEMETRY_SERIALTRANSFER_BAUD);
#else
  Serial.printf("{\"cmd\":\"TELEM_LINK_CFG\",\"enabled\":1,\"driver\":\"SerialTransfer\",\"port\":\"Serial4\",\"baud\":%lu,\"mode\":\"balance_flush_on_idle\"}\r\n",
                (unsigned long)TELEMETRY_SERIALTRANSFER_BAUD);
#endif
#elif TELEMETRY_LINK_ENABLE
  telemetry_link_init(Serial4, TELEMETRY_LINK_BAUD);
  Serial.printf("{\"cmd\":\"TELEM_LINK_CFG\",\"enabled\":1,\"port\":\"Serial4\",\"baud\":%lu}\r\n",
                (unsigned long)TELEMETRY_LINK_BAUD);
#else
  Serial.printf("{\"cmd\":\"TELEM_LINK_CFG\",\"enabled\":0}\r\n");
#endif

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);
  tone_init(&g_tone, SPEAKER_PIN);

  // ---- IMU Setup (hybrid: TWR filter behavior + vibration_test robustness) ----
  const IMUHelperInitResult imu_init = imu_helper_init(imu, CS_PIN, INT_PIN);
  if (!imu_init.ok) {
    Serial.printf("{\"cmd\":\"IMU_INIT_FAIL\",\"step\":\"%s\",\"status\":%d}\r\n",
                  imu_init.step,
                  imu_init.status);
    while (true) {
      delay(1000);
    }
  }

  // ---- CAN Setup ----
  // CAN1 default routing on Teensy 4.0: RX=pin 23, TX=pin 22.
  Can1.setRX(CAN1_PINSEL);
  Can1.setTX(CAN1_PINSEL);
  Can1.begin();
  Can1.setBaudRate(1000000);
  // CAN reliability improvement:
  // FIFO + IRQ receive path reduces loop-latency-induced drops.
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(can1_on_receive);

  // CAN2 default routing on Teensy 4.0: RX=pin 0, TX=pin 1.
  Can2.setRX(CAN2_PINSEL);
  Can2.setTX(CAN2_PINSEL);
  Can2.begin();
  Can2.setBaudRate(1000000);
  // CAN reliability improvement:
  // Mirror CAN1 FIFO + IRQ setup so both buses have symmetric RX behavior.
  Can2.enableFIFO();
  Can2.enableFIFOInterrupt();
  Can2.onReceive(can2_on_receive);
  pinMode(CAN_STB, OUTPUT);
  digitalWrite(CAN_STB, LOW);

  init_supervisor(&supervisor,
                  2,           // esc_count -- FIX: dont hard code this number
                  esc_names,   // ESC names
                  esc_ids,     // ESC node IDs
                  rc_pins,     // RC pins
                  RC_PIN_COUNT); // RC count

  // Start in idle; require explicit serial command or pushbutton calibrate flow.
  supervisor.mode = SUP_MODE_IDLE;
  g_prev_mode_for_exit_tweet = supervisor.mode;
  g_prev_mode_for_telem_flush = supervisor.mode;
  reset_calibration_state();
  supervisor.user_total_us = 0;
  supervisor.user_tx_enable = true;
  supervisor.user_tx_period_us = 2000;  // default 500 Hz command TX
  canResetPosvelStats();
  canResetRuntimeStats();
  canRxBuf1.overflow_count = 0;
  canRxBuf2.overflow_count = 0;
  Serial.printf("{\"cmd\":\"CONSOLE_CFG\",\"enabled\":%d,\"echo\":%d,\"char_budget\":%lu}\r\n",
                SERIAL_CONSOLE_ENABLE ? 1 : 0,
                SERIAL_CONSOLE_ECHO ? 1 : 0,
                (unsigned long)SERIAL_CONSOLE_CHAR_BUDGET_PER_CALL);

  // ---- Control tick ISR ----
  g_ctrlTimer.priority(CONTROL_LOOP_PRIORITY);
  g_ctrlTimer.begin(controlLoop_isr, CONTROL_PERIOD_US);

}

void loop() {
  // -------- HIGH PRIORITY --------
  // Set to CONTROL_PERIOD_US = 1000 µs (1000 Hz).
  // NOTE: ESC POSVEL over CAN is expected around 500 Hz (about every 2 ms),
  // while this control loop runs at 1 kHz (every 1 ms). They are intentionally
  // asynchronous, so many control ticks will reuse the latest POSVEL sample.
  // TODO(balance): IQREQ command spacing is currently jittered (~2-3 ms in tests),
  // so balancing control must use measured dt/freshness, not fixed-period assumptions.
  // The system uses an ISR-driven scheduler to tell the main loop to go into controlLoop. 
  // ---
  while (true) {
    noInterrupts();
    const uint32_t pending = g_control_pending_ticks;
    if (pending == 0u) {
      interrupts();
      break;
    }
    g_control_pending_ticks = pending - 1u;
    interrupts();
    controlLoop(&supervisor, Can1, Can2);
  }

  // Tweet exactly once whenever balance mode exits.
  if (is_balance_mode(g_prev_mode_for_exit_tweet) &&
      !is_balance_mode(supervisor.mode)) {
    tone_start(&g_tone, BALANCE_EXIT_BEEP_HZ, BALANCE_EXIT_BEEP_MS, BALANCE_EXIT_BEEP_GAP_MS);
    Serial.printf("{\"cmd\":\"BALANCE_EXIT_TWEET\",\"from\":%d,\"to\":%d}\r\n",
                  (int)g_prev_mode_for_exit_tweet,
                  (int)supervisor.mode);
  }
  g_prev_mode_for_exit_tweet = supervisor.mode;

  // -------- CAN RX --------
  // ISR callbacks enqueue frames; events() dispatch drains hardware FIFOs.
  // ---
  imu_helper_poll(imu, supervisor, micros());

  CAN_message_t msg;
  // CAN reliability improvement:
  // Dispatch queued FlexCAN callbacks every loop so ISR-fed FIFO data drains promptly.
  const uint32_t can1_events_us = micros();
  canNoteEventsDispatch(1u, can1_events_us);
  Can1.events();
  const uint32_t can2_events_us = micros();
  canNoteEventsDispatch(2u, can2_events_us);
  Can2.events();
  uint8_t rx_bus = 0u;
  while (true) {
    bool popped_any = false;
    if (canBufferPop(canRxBuf1, msg, &rx_bus)) {
      handleCANMessage(msg, rx_bus);
      popped_any = true;
    }
    if (canBufferPop(canRxBuf2, msg, &rx_bus)) {
      handleCANMessage(msg, rx_bus);
      popped_any = true;
    }
    if (!popped_any) break;
  }

  uint32_t now_us = micros();
  uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
  bool posvel_rx_fresh = (last_posvel_rx_us != 0u) &&
                         ((uint32_t)(now_us - last_posvel_rx_us) <= CAN_POSVEL_RX_TIMEOUT_US);
  led_set_state(&g_led_red, posvel_rx_fresh ? LED_PULSE : LED_OFF);
  led_update(&g_led_red, now_us);

  const bool button_raw_pressed = (g_button.readRaw() == PB_PRESSED);
  if (button_raw_pressed) {
    if (!g_idle_long_hold_tracking) {
      g_idle_long_hold_tracking = true;
      g_idle_long_hold_start_us = now_us;
      g_idle_long_hold_fired = false;
    } else if (!g_idle_long_hold_fired &&
               ((uint32_t)(now_us - g_idle_long_hold_start_us) >= IDLE_LONG_HOLD_US)) {
      g_idle_long_hold_fired = true;
      if (is_balance_mode(supervisor.mode)) {
        dump_selected_balance_mode_on_exit(supervisor.mode, "button_long_hold");
      }
      supervisor.mode = SUP_MODE_IDLE;
      g_balance_mode_enter_us = 0u;
      supervisor.user_total_us = 0u;
      reset_calibration_state();
      start_idle_long_hold_song();
      Serial.printf("{\"cmd\":\"MODE\",\"source\":\"button\",\"mode\":\"SUP_MODE_IDLE\",\"reason\":\"button_long_hold\"}\r\n");
    }
  } else {
    g_idle_long_hold_tracking = false;
    g_idle_long_hold_fired = false;
  }

  if (supervisor.mode == SUP_MODE_CALIBRATE) {
    if (supervisor.imu.valid) {
      const float pitch_accel = imu_helper_pitch_accel_from_last_sample();
      if (!g_cal_kickstand_ready) {
        g_cal_kickstand_sum += pitch_accel;
        g_cal_kickstand_count++;
        if (g_cal_kickstand_count >= CAL_KICKSTAND_SAMPLES) {
          g_cal_kickstand_pitch = g_cal_kickstand_sum / (float)g_cal_kickstand_count;
          g_cal_theta_eq_target = g_cal_kickstand_pitch + CAL_TARGET_DELTA_RAD;
          g_cal_kickstand_ready = true;
          Serial.printf(
              "{\"cmd\":\"CALIBRATE_REF\",\"kickstand_pitch_rad\":%.6f,\"target_eq_rad\":%.6f,\"target_delta_rad\":%.6f,\"samples\":%u}\r\n",
              g_cal_kickstand_pitch,
              g_cal_theta_eq_target,
              CAL_TARGET_DELTA_RAD,
              (unsigned int)g_cal_kickstand_count);
        }
      } else {
        const float delta_from_kickstand = pitch_accel - g_cal_kickstand_pitch;
        const float eq_err_rad = pitch_accel - g_cal_theta_eq_target;
        const bool in_window = fabsf(eq_err_rad) <= CAL_TARGET_TOL_RAD;
        const bool low_energy = fabsf(supervisor.imu.pitch_rate) <= CAL_RATE_MAX_RAD_S;
        g_cal_target_in_window = in_window;

        if (in_window && !g_cal_target_reached_announced) {
          g_cal_target_reached_announced = true;
          balance_zero_cross_tweet();
          Serial.printf(
              "{\"cmd\":\"CAL_TARGET_REACHED\",\"pitch_accel_rad\":%.6f,\"kickstand_pitch_rad\":%.6f,\"delta_rad\":%.6f,\"target_delta_rad\":%.6f}\r\n",
              pitch_accel,
              g_cal_kickstand_pitch,
              delta_from_kickstand,
              CAL_TARGET_DELTA_RAD);
        }

        if (in_window && low_energy) {
          if (g_cal_hold_start_us == 0u) g_cal_hold_start_us = now_us;
          if ((uint32_t)(now_us - g_cal_hold_start_us) >= CAL_HOLD_US) {
            canRxBuf1.overflow_count = 0;
            canRxBuf2.overflow_count = 0;
            for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
              supervisor.esc_alive_false_count[i] = 0u;
            }
            supervisor.user_tx_enable = true;
            supervisor.user_total_us = BALANCE_BUTTON_RUN_US;
            supervisor.mode = SUP_MODE_BALANCE_TWR;
            g_balance_mode_enter_us = now_us;
            balance_TWR_set_theta_reference(g_cal_theta_eq_target, 0.0f);
            balance_zero_cross_tweet();
            Serial.printf(
                "{\"cmd\":\"MODE\",\"source\":\"calibrate\",\"mode\":\"SUP_MODE_BALANCE_TWR\",\"theta_eq_rad\":%.6f,\"pitch_accel_rad\":%.6f,\"pitch_rate_rad_s\":%.6f,\"hold_ms\":%lu}\r\n",
                g_cal_theta_eq_target,
                pitch_accel,
                supervisor.imu.pitch_rate,
                (unsigned long)(CAL_HOLD_US / 1000u));
            reset_calibration_state();
          }
        } else {
          g_cal_hold_start_us = 0u;
        }
      }
    }
  }

  // LED2 behavior:
  // - Idle: OFF.
  // - Calibrate: ON only when near the zero-equilibrium target angle.
  // - Other modes: link-health indicator.
  if (supervisor.mode == SUP_MODE_IDLE) {
    led_set_state(&g_led_green, LED_OFF);
  } else if (supervisor.mode == SUP_MODE_CALIBRATE) {
    led_set_state(&g_led_green, g_cal_target_in_window ? LED_ON_CONTINUOUS : LED_OFF);
  } else {
    const bool can_link_ok = canRxBuf1.link_ok || canRxBuf2.link_ok;
    led_set_state(&g_led_green, can_link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
  }
  led_update(&g_led_green, now_us);
  tone_update(&g_tone, now_us);
  update_idle_long_hold_song();
  if (supervisor.mode == SUP_MODE_CALIBRATE) {
    update_calibrate_song();
  }

  if (TEMP_PRINT_BALANCE_IMU_ENABLE &&
      is_balance_mode(supervisor.mode) &&
      supervisor.imu.valid &&
      ((uint32_t)(now_us - g_last_balance_imu_print_us) >= TEMP_PRINT_BALANCE_IMU_PERIOD_US)) {
    g_last_balance_imu_print_us = now_us;
    float accel_x_g = 0.0f;
    float accel_y_g = 0.0f;
    float accel_z_g = 0.0f;
    imu_helper_get_last_accel_g(&accel_x_g, &accel_y_g, &accel_z_g);
    Serial.printf(
        "{\"cmd\":\"BALANCE_IMU\",\"t\":%lu,\"mode\":\"%s\",\"ax_g\":%.5f,\"ay_g\":%.5f,\"az_g\":%.5f,\"pitch_filt_rad\":%.6f,\"pitch_filt_deg\":%.3f,\"pitch_rate_raw_rad_s\":%.6f}\r\n",
        (unsigned long)now_us,
        mode_to_str(supervisor.mode),
        accel_x_g,
        accel_y_g,
        accel_z_g,
        supervisor.imu.pitch_rad,
        supervisor.imu.pitch_rad * (180.0f / PI),
        supervisor.imu.pitch_rate_raw);
  }

  // PUSHBUTTON (run every loop to avoid latency in target capture)
#ifndef PB_OVERRIDE
  g_button.update(now_us);
  if (g_button.hasChanged()) {
    PBState pb_state = g_button.getState();

    if (pb_state == PB_PRESSED) {
      Serial.printf(
          "{\"cmd\":\"BUTTON_PRESS_ANGLE\",\"t\":%lu,\"imu_valid\":%d,\"pitch_raw_rad\":%.6f,\"pitch_raw_deg\":%.3f,\"pitch_rate_raw_rad_s\":%.6f,\"pitch_rate_raw_deg_s\":%.3f}\r\n",
          (unsigned long)now_us,
          supervisor.imu.valid ? 1 : 0,
          supervisor.imu.pitch_rad,
          supervisor.imu.pitch_rad * (180.0f / PI),
          supervisor.imu.pitch_rate_raw,
          supervisor.imu.pitch_rate_raw * (180.0f / PI));
      if (is_balance_mode(supervisor.mode)) {
        dump_selected_balance_mode_on_exit(supervisor.mode, "button_press_reset");
        supervisor.mode = SUP_MODE_IDLE;
        // Suppress BALANCE_EXIT_TWEET for this explicit button-reset path.
        g_prev_mode_for_exit_tweet = supervisor.mode;
        g_balance_mode_enter_us = 0u;
        supervisor.user_total_us = 0u;
        reset_calibration_state();
        start_idle_long_hold_song();
        Serial.printf("{\"cmd\":\"MODE\",\"source\":\"button\",\"mode\":\"SUP_MODE_IDLE\",\"reason\":\"button_press_reset\"}\r\n");
      } else if (supervisor.mode == SUP_MODE_IDLE) {
        reset_calibration_state();
        supervisor.mode = SUP_MODE_CALIBRATE;
        start_calibrate_song();
        Serial.printf("{\"cmd\":\"MODE\",\"source\":\"button\",\"mode\":\"SUP_MODE_CALIBRATE\"}\r\n");
      }
    }
    else if (pb_state == PB_RELEASED) {
      if (!g_button.isArmed()) {
        g_button.clearChanged();
        return;
      }
      if (supervisor.mode != SUP_MODE_IDLE &&
                 supervisor.mode != SUP_MODE_CALIBRATE) {
        Serial.printf("{\"cmd\":\"BUTTON_IGNORED\",\"reason\":\"unsupported_mode\",\"mode\":%d}\r\n",
                      (int)supervisor.mode);
      }
      g_button.clearArmed();
    }
    g_button.clearChanged();
  }
#endif

  // Keep ESP32 telemetry link drained continuously without blocking control flow.
#if TELEMETRY_LINK_ENABLE
  if (!g_telem_flush_after_balance_twr &&
      g_prev_mode_for_telem_flush == SUP_MODE_BALANCE_TWR &&
      supervisor.mode == SUP_MODE_IDLE) {
    g_telem_flush_after_balance_twr = true;
    g_telem_flush_start_us = micros();
    g_telem_flush_start_packets = telemetry_pending_packets();
    g_telem_flush_start_bytes = telemetry_pending_bytes();
#if TELEMETRY_SERIALTRANSFER_TX_ENABLE
    g_telem_flush_last_tx_us = 0u;
    g_telem_flush_sent_packets = 0u;
    g_telem_flush_acked_packets = 0u;
    g_telem_flush_retries = 0u;
    g_telem_flush_outstanding_seq = 0u;
    g_telem_flush_waiting_ack = false;
    Serial.printf(
        "{\"cmd\":\"TELEM_FLUSH_START\",\"driver\":\"SerialTransfer\",\"from\":%d,\"to\":%d,\"packet_size_bytes\":%lu,\"queued_packets\":%lu,\"queued_bytes\":%lu}\r\n",
        (int)g_prev_mode_for_telem_flush,
        (int)supervisor.mode,
        (unsigned long)telemetry_packet_size_bytes(),
        (unsigned long)g_telem_flush_start_packets,
        (unsigned long)g_telem_flush_start_bytes);
#else
    Serial.printf(
        "{\"cmd\":\"TELEM_FLUSH_START\",\"from\":%d,\"to\":%d,\"packet_size_bytes\":%lu,\"queued_packets\":%lu,\"queued_bytes\":%lu}\r\n",
        (int)g_prev_mode_for_telem_flush,
        (int)supervisor.mode,
        (unsigned long)telemetry_packet_size_bytes(),
        (unsigned long)g_telem_flush_start_packets,
        (unsigned long)g_telem_flush_start_bytes);
#endif
  }

#if !TELEMETRY_SERIALTRANSFER_TX_ENABLE
  if (g_telem_flush_after_balance_twr && supervisor.mode == SUP_MODE_IDLE) {
    telemetry_uart_pump();
    if (!telemetry_has_pending()) {
      const uint32_t end_packets = telemetry_pending_packets();
      const uint32_t end_bytes = telemetry_pending_bytes();
      const uint32_t elapsed_us = (uint32_t)(micros() - g_telem_flush_start_us);
      Serial.printf(
          "{\"cmd\":\"TELEM_FLUSH_END\",\"packet_size_bytes\":%lu,\"start_packets\":%lu,\"start_bytes\":%lu,\"remaining_packets\":%lu,\"remaining_bytes\":%lu,\"elapsed_us\":%lu}\r\n",
          (unsigned long)telemetry_packet_size_bytes(),
          (unsigned long)g_telem_flush_start_packets,
          (unsigned long)g_telem_flush_start_bytes,
          (unsigned long)end_packets,
          (unsigned long)end_bytes,
          (unsigned long)elapsed_us);
      g_telem_flush_after_balance_twr = false;
    }
  }
#endif
#endif
  g_prev_mode_for_telem_flush = supervisor.mode;

#if TELEMETRY_SERIALTRANSFER_TX_ENABLE
  telemetry_serialtransfer_flush_tick(now_us);
#endif

#if TELEMETRY_SERIALTRANSFER_TX_ENABLE && TELEMETRY_SERIALTRANSFER_CONTINUOUS_TX_ENABLE
  // Non-blocking 100 Hz packet send in low-priority context.
  telemetry_serialtransfer_send_100hz(now_us);
#endif

  // -------- LOW PRIORITY --------
  // These functions are intentionally throttled and run infrequently.
  // ---

  static uint32_t last_lowprio_us = 0;

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 100)) {
    last_lowprio_us = now_us;

#if SERIAL_CONSOLE_ENABLE
    ConsoleCommandContext cmd_ctx{
      &supervisor,
      &g_tone,
      &canRxBuf1,
      &canRxBuf2,
      &g_balance_mode_enter_us,
      selected_balance_entry_mode(),
      CAN_TEST_RUN_DEFAULT_US,
      BALANCE_BUTTON_RUN_US
    };
    console_process_serial(cmd_ctx);
#endif

    // LOW-RATE UPDATES
    if (millis() - supervisor.last_health_ms > 1000) {
      supervisor.last_health_ms = millis();
    }

  } // end of low priority loop
}
