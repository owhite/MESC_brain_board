#include <FlexCAN_T4.h>
#include "LED.h"
#include <ArduinoJson.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include "main.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "ICM42688.h"
#include "ESC.h"
#include "CAN_helper.h"
#include "supervisor.h"
#include "test_can_transmit_mode.h"
#include "balance_TWR_mode.h"

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
CANBuffer canRxBuf;  // ✅ holds buffer + link_ok

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
static constexpr uint32_t ZERO_CROSS_BEEP_HZ = 2800u;
static constexpr uint32_t ZERO_CROSS_BEEP_MS = 10u;
static constexpr uint32_t ZERO_CROSS_BEEP_GAP_MS = 0u;
PushButton g_button(PUSHBUTTON_PIN, true, 50000u);
ICM42688 imu(SPI, CS_PIN);
static volatile bool g_imu_data_ready = false;
static constexpr uint32_t CAN_POSVEL_RX_TIMEOUT_US = 400000u;
static constexpr uint32_t BALANCE_BUTTON_RUN_US = 0u;  // 0 = no auto-timeout
// Runtime decimated printing can perturb timing; keep off for measurement runs.
// Uncomment to ignore pushbutton state transitions.
// #define PB_OVERRIDE

static void imu_data_ready_isr() {
  g_imu_data_ready = true;
}

// ---- Mahony 6DOF state (ported from TWR_test) ----
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
static const float twoKp = 2.0f * 0.5f;  // Kp = 0.5
static const float twoKi = 2.0f * 0.1f;  // Ki = 0.1

static void mahony_update_imu(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float dt_s) {
  float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
  bool accel_valid = false;

  if (acc_mag > 1e-6f) {
    float recip = 1.0f / acc_mag;
    ax *= recip;
    ay *= recip;
    az *= recip;
    accel_valid = (acc_mag > 0.85f && acc_mag < 1.15f);
  }

  float ex = 0.0f, ey = 0.0f, ez = 0.0f;

  if (accel_valid) {
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    if (twoKi > 0.0f) {
      integralFBx += twoKi * ex * dt_s;
      integralFBy += twoKi * ey * dt_s;
      integralFBz += twoKi * ez * dt_s;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = integralFBy = integralFBz = 0.0f;
    }

    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;
  } else {
    integralFBx = integralFBy = integralFBz = 0.0f;
  }

  gx *= 0.5f * dt_s;
  gy *= 0.5f * dt_s;
  gz *= 0.5f * dt_s;

  float qa = q0;
  float qb = q1;
  float qc = q2;
  float qd = q3;

  q0 += (-qb * gx - qc * gy - qd * gz);
  q1 += (qa * gx + qc * gz - qd * gy);
  q2 += (qa * gy - qb * gz + qd * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  float norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
}

static void update_supervisor_imu(ICM42688 &imu_dev,
                                  Supervisor_typedef &sup,
                                  uint32_t now_us) {
  float dt_s = (now_us - sup.imu.last_update_us) * 1e-6f;
  if (dt_s < 0.0005f || dt_s > 0.005f) {
    dt_s = CONTROL_PERIOD_US * 1e-6f;
  }

  float ax = imu_dev.accX();
  float ay = imu_dev.accY();
  float az = imu_dev.accZ();
  float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
  uint8_t accel_valid = (acc_mag > 0.85f && acc_mag < 1.15f) ? 1u : 0u;

  float gx = imu_dev.gyrX() * DEG_TO_RAD;
  float gy = imu_dev.gyrY() * DEG_TO_RAD;
  float gz = imu_dev.gyrZ() * DEG_TO_RAD;

  mahony_update_imu(gx, gy, gz, ax, ay, az, dt_s);

  float pitch_rad = asinf(2.0f * (q0 * q2 - q3 * q1));
  float pitch_rate_raw = gy;

  const float rate_alpha = 0.15f;
  float pitch_rate = rate_alpha * pitch_rate_raw +
                     (1.0f - rate_alpha) * sup.imu.pitch_rate;

  sup.imu.pitch_rad = pitch_rad;
  sup.imu.pitch_rate_raw = pitch_rate_raw;
  sup.imu.pitch_rate = pitch_rate;
  sup.imu.accel_mag_g = acc_mag;
  sup.imu.accel_valid = accel_valid;
  sup.imu.mahony_int_fb_y = integralFBy;
  sup.imu.valid = true;
  sup.imu.last_update_us = now_us;
}

static void trim_ascii(char *s) {
  if (s == nullptr) return;
  size_t len = strlen(s);
  size_t start = 0;
  while (start < len && isspace((unsigned char)s[start])) start++;
  size_t end = len;
  while (end > start && isspace((unsigned char)s[end - 1])) end--;
  if (start > 0 && end > start) {
    memmove(s, s + start, end - start);
  }
  s[end - start] = '\0';
}

static void print_rc_channels_snapshot(const Supervisor_typedef &sup) {
  Serial.printf("{\"cmd\":\"RC_CH\",\"count\":%u", (unsigned int)sup.rc_count);
  for (uint8_t i = 0u; i < sup.rc_count; ++i) {
    Serial.printf(",\"ch%u_valid\":%d,\"ch%u_raw_us\":%u,\"ch%u_norm\":%.3f",
                  (unsigned int)(i + 1u), sup.rc[i].valid ? 1 : 0,
                  (unsigned int)(i + 1u), (unsigned int)sup.rc[i].raw_us,
                  (unsigned int)(i + 1u), sup.rc[i].norm);
  }
  Serial.printf("}\r\n");
}

void balance_zero_cross_tweet(void) {
  tone_start(&g_tone, ZERO_CROSS_BEEP_HZ, ZERO_CROSS_BEEP_MS, ZERO_CROSS_BEEP_GAP_MS);
}

static void process_serial_line(const char *line) {
  if (line == nullptr) return;

  if (strcmp(line, "run") == 0) {
    tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    canRxBuf.overflow_count = 0;
    for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
      supervisor.esc_alive_false_count[i] = 0u;
    }
    supervisor.user_tx_enable = true;
    supervisor.user_total_us = BALANCE_BUTTON_RUN_US;
    supervisor.mode = SUP_MODE_BALANCE_TWR;
    Serial.printf("{\"cmd\":\"MODE\",\"source\":\"serial\",\"mode\":\"SUP_MODE_BALANCE_TWR\",\"run_us\":%lu}\r\n",
                  (unsigned long)supervisor.user_total_us);
    return;
  }

  if (strcmp(line, "rc show") == 0) {
    print_rc_channels_snapshot(supervisor);
    return;
  }

  if (strcmp(line, "rc run") == 0) {
    tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    canRxBuf.overflow_count = 0;
    for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
      supervisor.esc_alive_false_count[i] = 0u;
    }
    supervisor.user_tx_enable = true;
    supervisor.user_total_us = 0u;
    supervisor.user_rc_drive_enable = true;
    supervisor.mode = SUP_MODE_TEST_CAN;
    Serial.printf(
        "{\"cmd\":\"MODE\",\"source\":\"serial\",\"mode\":\"SUP_MODE_TEST_CAN\",\"rc_drive\":1,"
        "\"throttle_ch\":%u,\"steer_ch\":%u,\"deadband\":%.3f,\"max_torque_nm\":%.3f,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
        (unsigned int)(supervisor.user_rc_throttle_ch + 1u),
        (unsigned int)(supervisor.user_rc_steer_ch + 1u),
        supervisor.user_rc_deadband,
        supervisor.user_rc_max_torque_nm,
        (unsigned long)supervisor.user_tx_period_us,
        (supervisor.user_tx_period_us > 0u)
            ? (1000000.0f / (float)supervisor.user_tx_period_us)
            : 0.0f);
    return;
  }

  if (strcmp(line, "rc stop") == 0) {
    supervisor.user_rc_drive_enable = false;
    supervisor.mode = SUP_MODE_IDLE;
    supervisor.user_total_us = 0u;
    Serial.printf("{\"cmd\":\"MODE\",\"source\":\"serial\",\"mode\":\"SUP_MODE_IDLE\",\"reason\":\"rc_stop\"}\r\n");
    return;
  }

  float rc_max_nm = 0.0f;
  if (sscanf(line, "rc max %f", &rc_max_nm) == 1) {
    if (!(rc_max_nm >= 0.0f && rc_max_nm <= 2.0f)) {
      Serial.printf("{\"cmd\":\"RC_CFG_ERR\",\"reason\":\"max_torque_out_of_range\",\"value\":%.3f,\"min\":0.0,\"max\":2.0}\r\n",
                    rc_max_nm);
    } else {
      supervisor.user_rc_max_torque_nm = rc_max_nm;
      Serial.printf("{\"cmd\":\"RC_CFG\",\"max_torque_nm\":%.3f}\r\n", supervisor.user_rc_max_torque_nm);
    }
    return;
  }

  uint32_t rc_ch_t = 0u;
  uint32_t rc_ch_s = 0u;
  if (sscanf(line, "rc ch %lu %lu", &rc_ch_t, &rc_ch_s) == 2) {
    if (rc_ch_t < 1u || rc_ch_s < 1u ||
        rc_ch_t > supervisor.rc_count || rc_ch_s > supervisor.rc_count) {
      Serial.printf("{\"cmd\":\"RC_CFG_ERR\",\"reason\":\"channel_out_of_range\",\"count\":%u}\r\n",
                    (unsigned int)supervisor.rc_count);
    } else {
      supervisor.user_rc_throttle_ch = (uint8_t)(rc_ch_t - 1u);
      supervisor.user_rc_steer_ch = (uint8_t)(rc_ch_s - 1u);
      Serial.printf("{\"cmd\":\"RC_CFG\",\"throttle_ch\":%u,\"steer_ch\":%u}\r\n",
                    (unsigned int)rc_ch_t, (unsigned int)rc_ch_s);
    }
    return;
  }

  float rc_deadband = 0.0f;
  if (sscanf(line, "rc deadband %f", &rc_deadband) == 1) {
    if (!(rc_deadband >= 0.0f && rc_deadband < 0.5f)) {
      Serial.printf("{\"cmd\":\"RC_CFG_ERR\",\"reason\":\"deadband_out_of_range\",\"value\":%.3f,\"min\":0.0,\"max\":0.49}\r\n",
                    rc_deadband);
    } else {
      supervisor.user_rc_deadband = rc_deadband;
      Serial.printf("{\"cmd\":\"RC_CFG\",\"deadband\":%.3f}\r\n", supervisor.user_rc_deadband);
    }
    return;
  }

  if (strcmp(line, "verify_angle") == 0) {
    tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    supervisor.user_verify_motor_enable = false;
    supervisor.mode = SUP_VERIFY_ANGLE;
    Serial.printf(
      "{\"cmd\":\"MODE\",\"mode\":\"SUP_VERIFY_ANGLE\",\"motor\":0}\r\n");
    return;
  }

  if (strcmp(line, "motor") == 0) {
    tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    canRxBuf.overflow_count = 0;
    for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
      supervisor.esc_alive_false_count[i] = 0u;
    }
    supervisor.mode = SUP_VERIFY_ANGLE;
    if (supervisor.user_verify_motor_enable) {
      supervisor.user_verify_motor_enable = false;
      Serial.printf(
        "{\"cmd\":\"MODE\",\"mode\":\"SUP_VERIFY_ANGLE\",\"motor\":0,\"tau_left_nm\":%.3f,\"tau_right_nm\":%.3f}\r\n",
        supervisor.user_verify_tau_left,
        supervisor.user_verify_tau_right);
    } else {
      supervisor.user_verify_motor_enable = true;
      supervisor.user_verify_tau_left = 2.0f;
      supervisor.user_verify_tau_right = -2.0f;
      Serial.printf(
        "{\"cmd\":\"MODE\",\"mode\":\"SUP_VERIFY_ANGLE\",\"motor\":1,\"tau_left_nm\":%.3f,\"tau_right_nm\":%.3f}\r\n",
        supervisor.user_verify_tau_left,
        supervisor.user_verify_tau_right);
    }
    return;
  }

  if (strcmp(line, "tx off") == 0) {
    supervisor.user_tx_enable = false;
    Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":0,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                  (unsigned long)supervisor.user_tx_period_us,
                  (supervisor.user_tx_period_us > 0u)
                    ? (1000000.0f / (float)supervisor.user_tx_period_us)
                    : 0.0f);
    return;
  }

  if (strcmp(line, "tx on") == 0) {
    supervisor.user_tx_enable = true;
    Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":1,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                  (unsigned long)supervisor.user_tx_period_us,
                  (supervisor.user_tx_period_us > 0u)
                    ? (1000000.0f / (float)supervisor.user_tx_period_us)
                    : 0.0f);
    return;
  }

  if (strcmp(line, "stats reset") == 0) {
    canRxBuf.overflow_count = 0;
    for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
      supervisor.esc_alive_false_count[i] = 0u;
    }
    Serial.printf("{\"cmd\":\"CAN_STATS_RESET\",\"ok\":1}\r\n");
    return;
  }

  uint32_t hz = 0;
  if (sscanf(line, "tx hz %lu", &hz) == 1) {
    if (hz == 0u || hz > 2000u) {
      Serial.printf("{\"cmd\":\"CAN_TX_CFG_ERR\",\"reason\":\"hz_out_of_range\",\"hz\":%lu,\"min\":1,\"max\":2000}\r\n",
                    (unsigned long)hz);
    } else {
      supervisor.user_tx_period_us = 1000000u / hz;
      if (supervisor.user_tx_period_us == 0u) supervisor.user_tx_period_us = 1u;
      Serial.printf("{\"cmd\":\"CAN_TX_CFG\",\"tx_enable\":%d,\"tx_period_us\":%lu,\"tx_hz\":%.2f}\r\n",
                    supervisor.user_tx_enable ? 1 : 0,
                    (unsigned long)supervisor.user_tx_period_us,
                    (supervisor.user_tx_period_us > 0u)
                      ? (1000000.0f / (float)supervisor.user_tx_period_us)
                      : 0.0f);
    }
    return;
  }

  Serial.printf("{\"cmd\":\"CAN_CMD_ERR\",\"line\":\"%s\"}\r\n", line);
}

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

void setup() {

  Serial.begin(921600);
  while (!Serial && millis() < 1500) {}
  // CAN2 uses pins 0/1 on Teensy 4.0, so avoid Serial1 on those same pins.

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);
  tone_init(&g_tone, SPEAKER_PIN);

  // ---- IMU Setup (hybrid: TWR filter behavior + vibration_test robustness) ----
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(INT_PIN, INPUT);
  SPI.begin();

  int imu_status = imu.begin();
  if (imu_status < 0) {
    Serial.printf("{\"cmd\":\"IMU_INIT_FAIL\",\"step\":\"begin\",\"status\":%d}\r\n", imu_status);
    while (true) { delay(1000); }
  }
  imu_status = imu.setAccelODR(ICM42688::odr2k);
  if (imu_status < 0) {
    Serial.printf("{\"cmd\":\"IMU_INIT_FAIL\",\"step\":\"setAccelODR\",\"status\":%d}\r\n", imu_status);
    while (true) { delay(1000); }
  }
  imu_status = imu.setGyroODR(ICM42688::odr2k);
  if (imu_status < 0) {
    Serial.printf("{\"cmd\":\"IMU_INIT_FAIL\",\"step\":\"setGyroODR\",\"status\":%d}\r\n", imu_status);
    while (true) { delay(1000); }
  }
  imu_status = imu.enableDataReadyInterrupt();
  if (imu_status < 0) {
    Serial.printf("{\"cmd\":\"IMU_INIT_FAIL\",\"step\":\"enableDataReadyInterrupt\",\"status\":%d}\r\n", imu_status);
    while (true) { delay(1000); }
  }
  // Clear any pending DRDY latch before attaching ISR.
  (void)imu.getRawAGT();
  g_imu_data_ready = false;
  attachInterrupt(digitalPinToInterrupt(INT_PIN), imu_data_ready_isr, RISING);

  // ---- CAN Setup ----
  // CAN1 default routing on Teensy 4.0: RX=pin 23, TX=pin 22.
  Can1.setRX(CAN1_PINSEL);
  Can1.setTX(CAN1_PINSEL);
  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.enableFIFO();

  // CAN2 default routing on Teensy 4.0: RX=pin 0, TX=pin 1.
  Can2.setRX(CAN2_PINSEL);
  Can2.setTX(CAN2_PINSEL);
  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.enableFIFO();
  pinMode(CAN_STB, OUTPUT);
  digitalWrite(CAN_STB, LOW);

  init_supervisor(&supervisor,
                  2,           // esc_count -- FIX: dont hard code this number
                  esc_names,   // ESC names
                  esc_ids,     // ESC node IDs
                  rc_pins,     // RC pins
                  RC_PIN_COUNT); // RC count

  // Start in idle; require explicit serial command "run" or pushbutton release to begin balance mode.
  supervisor.mode = SUP_MODE_IDLE;
  supervisor.user_total_us = 0;
  supervisor.user_tx_enable = true;
  supervisor.user_tx_period_us = 4000;  // default 250 Hz command TX
  canRxBuf.overflow_count = 0;

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

  // -------- CAN POLLING --------
  // Non-blocking and not based on an ISR because FLEXCAN_T4 did seem to work. 
  // ---
  if (g_imu_data_ready) {
    noInterrupts();
    g_imu_data_ready = false;
    interrupts();
    if (imu.getAGT() > 0) {
      update_supervisor_imu(imu, supervisor, micros());
    } else {
      supervisor.imu.valid = false;
      supervisor.imu.last_update_us = micros();
    }
  }

  CAN_message_t msg;
  while (Can1.read(msg)) {
    canNoteBusRead(1u);
    if (!canBufferPush(canRxBuf, msg, 1u)) {
      canNoteRxOverflow();
    }
  }
  while (Can2.read(msg)) {
    canNoteBusRead(2u);
    if (!canBufferPush(canRxBuf, msg, 2u)) {
      canNoteRxOverflow();
    }
  }
  uint8_t rx_bus = 0u;
  while (canBufferPop(canRxBuf, msg, &rx_bus)) {
    handleCANMessage(msg, rx_bus);
  }

  uint32_t now_us = micros();
  uint32_t last_posvel_rx_us = canGetLastPosVelRxUs();
  bool posvel_rx_fresh = (last_posvel_rx_us != 0u) &&
                         ((uint32_t)(now_us - last_posvel_rx_us) <= CAN_POSVEL_RX_TIMEOUT_US);
  led_set_state(&g_led_red, posvel_rx_fresh ? LED_PULSE : LED_OFF);
  led_update(&g_led_red, now_us);

  // PUSHBUTTON (run every loop to avoid latency in target capture)
#ifndef PB_OVERRIDE
  g_button.update(now_us);
  if (g_button.hasChanged()) {
    PBState pb_state = g_button.getState();

    if (pb_state == PB_PRESSED) {
      tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
    }
	    else if (pb_state == PB_RELEASED) {
	      SupervisorMode balance_mode = SUP_MODE_BALANCE_TWR;
	      if (supervisor.mode == balance_mode) {
          // Button press while balancing: exit to idle.
          balance_TWR_dump_on_mode_exit("button_stop");
          supervisor.mode = SUP_MODE_IDLE;
          supervisor.user_total_us = 0u;
          Serial.printf("{\"cmd\":\"BUTTON_IDLE\",\"source\":\"button\",\"mode\":\"SUP_MODE_IDLE\"}\r\n");
          Serial.printf("{\"cmd\":\"MODE\",\"source\":\"button\",\"mode\":\"SUP_MODE_IDLE\",\"reason\":\"button_stop\"}\r\n");
        } else {
          canRxBuf.overflow_count = 0;
          for (uint16_t i = 0; i < supervisor.esc_count; ++i) {
            supervisor.esc_alive_false_count[i] = 0u;
          }
	          supervisor.user_tx_enable = true;
	        supervisor.user_total_us = BALANCE_BUTTON_RUN_US;
	        supervisor.mode = balance_mode;
          Serial.printf("{\"cmd\":\"MODE\",\"source\":\"button\",\"mode\":\"SUP_MODE_BALANCE_TWR\",\"run_us\":%lu}\r\n",
                        (unsigned long)supervisor.user_total_us);
	      }
	    }
    g_button.clearChanged();
  }
#endif

  static char input_buf[96] = {0};
  static size_t input_len = 0;
  // -------- LOW PRIORITY --------
  // These functions are intentionally throttled and run infrequently.
  // ---

  static uint32_t last_lowprio_us = 0;

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 100)) {
    last_lowprio_us = now_us;

	    while (Serial.available()) {
	      char c = Serial.read();
        Serial.write((uint8_t)c);  // Echo input characters back to terminal.
	      if (c == '\r' || c == '\n') {
          if (input_len > 0u) {
            input_buf[input_len] = '\0';
            trim_ascii(input_buf);
            if (input_buf[0] != '\0') {
              process_serial_line(input_buf);
            }
            input_len = 0u;
            input_buf[0] = '\0';
          }
	      } else if (c == '\b' || c == 127) {
          if (input_len > 0u) {
            input_len--;
            input_buf[input_len] = '\0';
          }
        } else if (isprint((unsigned char)c)) {
          if (input_len < (sizeof(input_buf) - 1u)) {
            input_buf[input_len++] = c;
            input_buf[input_len] = '\0';
          }
        }
	    }

    // LED CONTROL
    tone_update(&g_tone, now_us);
    led_update(&g_led_green, now_us);

    // 1 Hz HEALTH CHECK
    if (millis() - supervisor.last_health_ms > 1000) {
      supervisor.last_health_ms = millis();
      led_set_state(&g_led_green, canRxBuf.link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
    }

  } // end of low priority loop
}
