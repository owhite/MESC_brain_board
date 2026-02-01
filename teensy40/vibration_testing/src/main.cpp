#include <FlexCAN_T4.h>
#include "LED.h"
#include <ArduinoJson.h>
#include "main.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "ICM42688.h"
#include "ESC.h"
#include "CAN_helper.h"
#include "supervisor.h"

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;
Supervisor_typedef supervisor;

const char* esc_names[]   = {"left", "right"};
const uint16_t esc_ids[]  = {11, 12}; // node_ids of the ESCs
const uint8_t rc_pins[]   = {RC_INPUT1, RC_INPUT2, RC_INPUT3, RC_INPUT4};

// -------------------- CAN Communication --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
CANBuffer canRxBuf;  // ✅ holds buffer + link_ok

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
PushButton g_button(PUSHBUTTON_PIN, true, 50000u);

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

ICM42688 imu(SPI, CS_PIN, IMU_SPI_HS_CLOCK);

void setup() {
#if SERIAL_WRITE
  Serial.begin(921600);
  while (!Serial && millis() < 1500) {}
#endif

#if TELEMETRY_WRITE
  Serial1.begin(115200);
#endif

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);
  tone_init(&g_tone, SPEAKER_PIN);

  // ---- Control tick ISR ----
  g_ctrlTimer.priority(CONTROL_LOOP_PRIORITY);
  g_ctrlTimer.begin(controlLoop_isr, CONTROL_PERIOD_US);

  // ---- CAN Setup ----
  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.enableFIFO();

  pinMode(TEST_PIN, OUTPUT);

  init_supervisor(&supervisor,
                  imu,
                  2,           // esc_count -- FIX: dont hard code this number
                  esc_names,   // ESC names
                  esc_ids,     // ESC node IDs
                  rc_pins,     // RC pins
                  4);          // RC count -- FIX: dont hard code this number
}

void loop() {
  // -------- HIGH PRIORITY --------
  // Set to CONTROL_PERIOD_US = 1000 µs (1000 Hz).
  // The system uses an ISR-driven scheduler to tell the main loop to go into controlLoop.
  //
  // IMPORTANT: clear g_control_due atomically to avoid missing ticks if the ISR fires
  // while controlLoop() is running.
  if (g_control_due) {
    uint32_t tick_us;

    noInterrupts();
    g_control_due = false;
    tick_us = g_control_now_us;
    interrupts();

    supervisor.timing.last_tick_us = tick_us;
    controlLoop(imu, &supervisor, Can1);
  }

  // -------- CAN POLLING --------
  // Non-blocking and not based on an ISR because FLEXCAN_T4 did seem to work.
  CAN_message_t msg;
  while (Can1.read(msg)) {
    canBufferPush(canRxBuf, msg);
  }
  while (canBufferPop(canRxBuf, msg)) {
    handleCANMessage(msg);
  }

  static String input = "";

  // -------- LOW PRIORITY --------
  // These functions are intentionally throttled using a x10 time divider
  static uint32_t last_lowprio_us = 0;
  uint32_t now_us = micros();

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 100)) {
    last_lowprio_us = now_us;

    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        // Try to parse the accumulated line
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, input);
        if (!err) {
          supervisor.user_pulse_torque = DEFAULT_PULSE_TORQUE;
          supervisor.user_pulse_us     = DEFAULT_PULSE_US;
          supervisor.user_total_us     = DEFAULT_TOTAL_US;
          supervisor.user_Kd_term      = DEFAULT_KD_TERM;
          supervisor.user_Kp_term      = DEFAULT_KP_TERM;

          if (doc.containsKey("cmd") && doc["cmd"] == "verify_angle") {
            tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
            if (supervisor.mode == SUP_VERIFY_ANGLE) {
              supervisor.mode = SUP_MODE_IDLE;
            }
            else {
              supervisor.mode = SUP_VERIFY_ANGLE;
            }
          }

          // start with json command:
          //   {'cmd': 'torque_reps', 'pulse_torque': 0.4, 'pulse_us': 250000}
          if (doc.containsKey("cmd") && doc["cmd"] == "torque_reps") {
            if (doc.containsKey("pulse_torque"))
              supervisor.user_pulse_torque = doc["pulse_torque"];
            if (doc.containsKey("pulse_us"))
              supervisor.user_pulse_us = doc["pulse_us"];

            tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
            if (supervisor.mode == SUP_TORQUE_REPS) {
              supervisor.mode = SUP_MODE_IDLE;
            }
            else {
              supervisor.mode = SUP_TORQUE_REPS;
            }

            Serial.printf("{\"cmd\":\"PRINT\",\"note\":\"%s\",\"pulse_torque\":%.3f,\"pulse_us\":%lu}\r\n",
                          "Torque response run started",
                          supervisor.user_pulse_torque,
                          supervisor.user_pulse_us);
          }
        }
        input = "";  // reset buffer
      } else {
        input += c;  // append char to buffer
      }
    }

    // LED CONTROL
    tone_update(&g_tone, now_us);
    led_update(&g_led_red, now_us);
    led_update(&g_led_green, now_us);

    // 1 Hz HEALTH CHECK
    if (millis() - supervisor.last_health_ms > 1000) {
      supervisor.last_health_ms = millis();
      led_set_state(&g_led_green, canRxBuf.link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
    }

    // PUSHBUTTON
    g_button.update(now_us);
    if (g_button.hasChanged()) {
      PBState pb_state = g_button.getState();

      if (pb_state == PB_PRESSED) {
        // SPEAKER
        tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
      }
      else if (pb_state == PB_RELEASED && g_button.isArmed()) {

        supervisor.mode = SUP_MODE_IDLE;

        LEDState cur  = g_led_red.state;
        LEDState next = (cur == LED_BLINK_FAST) ? LED_BLINK_SLOW : LED_BLINK_FAST;
        if (cur != LED_BLINK_FAST && cur != LED_BLINK_SLOW) next = LED_BLINK_FAST;
        led_set_state(&g_led_red, next);
        g_button.clearArmed();
      }
      g_button.clearChanged();
    }

    // Resets timing stats
    if (supervisor.timing.count > 0) { resetLoopTimingStats(&supervisor);  }
  } // end of low priority loop
}
