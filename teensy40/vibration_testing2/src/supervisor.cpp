#include "supervisor.h"
#include <string.h>
#include "main.h"

void balance_TWR_mode(Supervisor_typedef *sup,
		      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can);

// ---------------- Global Flags ----------------
// These are set by the control ISR to signal the main loop.
volatile bool g_control_due = false;
volatile uint32_t g_control_now_us = 0;

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
  g_control_due = true;
}

// --- Helper: compute shortest angular difference (target - actual) in [-π, +π]
float angle_diff(float target, float actual) {
  float diff = fmodf(target - actual + M_PI, 2.0f * M_PI);
  if (diff < 0) diff += 2.0f * M_PI;
  return diff - M_PI;
}

volatile bool dataReady = false;
void setImuFlag() { dataReady = true; }

// ---------------- Supervisor Initialization ----------------
// Sets up ESCs, IMU, RC inputs, and resets timing/telemetry stats.
// Called once at startup from main().
void init_supervisor(Supervisor_typedef *sup,
		     ICM42688 &imu,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count) {

  if (!sup) return;
  g_sup = sup;

  *sup = Supervisor_typedef{};

  // Clear ESC lookup table
  for (uint16_t i = 0; i < ESC_LOOKUP_SIZE; ++i) {
    esc_lookup[i] = nullptr;
  }

  // ESC setup and registration
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

  // set supervisor's IMU initial state
  //   does not touch the chip at this point
  sup->imu.valid = false;
  sup->imu.roll_rad = sup->imu.pitch_rad = sup->imu.yaw_rad = 0.0f;
  sup->imu.last_update_us = now;

  // Supervisor state machine initial mode
  sup->mode = SUP_MODE_IDLE;
  sup->gait_mode = GAIT_IDLE;
  sup->last_imu_update_us = now;

  // ---- IMU Setup ----
  int status = imu.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println(status);
    while (1) {}
  }

  // attaching the interrupt to micro controller pin INT_PIN
  pinMode(INT_PIN, INPUT);
  attachInterrupt(INT_PIN, setImuFlag, RISING);
  imu.setAccelODR(ICM42688::odr2k);
  imu.setGyroODR(ICM42688::odr2k);
  imu.enableDataReadyInterrupt();

  // Timing stats initialization
  sup->timing.last_tick_us = now;
  sup->timing.dt_us = 0;
  sup->timing.exec_time_us = 0;
  resetLoopTimingStats(sup);

  sup->last_health_ms = 0;

  // RC input setup: attach interrupts to each pin
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

  // Telemetry stats initialization
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

    float norm = (float)(pw - RC_INPUT_MIN_US) /
      (float)(RC_INPUT_MAX_US - RC_INPUT_MIN_US);

    sup->rc[i].raw_us = pw;
    sup->rc[i].norm   = norm;
    sup->rc[i].valid  = true;
  }
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

// ---- Mahony 6DOF state ----
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// Gains: tune these later if needed.
static const float twoKp = 2.0f * 0.5f;  // Kp = 0.5
static const float twoKi = 2.0f * 0.1f;  // Ki = 0.1

static void mahonyUpdateIMU(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt) {
  // -----------------------------------------------------------
  // Normalize accelerometer
  // -----------------------------------------------------------
  float accMag = sqrtf(ax*ax + ay*ay + az*az);

  bool accelValid = false;

  if (accMag > 1e-6f) {
    float recip = 1.0f / accMag;
    ax *= recip;
    ay *= recip;
    az *= recip;

    // accel gating: ignore accel when far from 1g
    if (fabsf(accMag - 1.0f) < 0.25f) {
      accelValid = true;
    }
  }

  // -----------------------------------------------------------
  // If accel is valid → compute correction terms (ex, ey, ez)
  // -----------------------------------------------------------
  float ex = 0.0f, ey = 0.0f, ez = 0.0f;

  if (accelValid) {
    // Estimated gravity direction from quaternion
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // Cross product error
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Integral feedback
    if (twoKi > 0.0f) {
      integralFBx += twoKi * ex * dt;
      integralFBy += twoKi * ey * dt;
      integralFBz += twoKi * ez * dt;

      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = integralFBy = integralFBz = 0.0f;
    }

    // Proportional feedback
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;
  }
  else {
    // accel invalid → no correction, but clear integral
    integralFBx = integralFBy = integralFBz = 0.0f;
  }

  // -----------------------------------------------------------
  // Integrate gyro to update quaternion
  // -----------------------------------------------------------
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  float qa = q0;
  float qb = q1;
  float qc = q2;
  float qd = q3;

  q0 += (-qb*gx - qc*gy - qd*gz);
  q1 += ( qa*gx + qc*gz - qd*gy);
  q2 += ( qa*gy - qb*gz + qd*gx);
  q3 += ( qa*gz + qb*gy - qc*gx);

  // -----------------------------------------------------------
  // Normalize
  // -----------------------------------------------------------
  float norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
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
// This function is called deterministically by the ISR.
// Strategy to preserve determinism:
//   - Do CAN bus draining and RC input updates outside
//     of this function.
//   - Inside here, only handle time-critical tasks:
//       * Poll IMU via I²C and update orientation
//       * Update timing statistics (jitter, overruns)
//       * Run balance control law (TODO)
//

static float ax_min = 999.0f, ax_max = -999.0f;
static float gx_min = 999.0f, gx_max = -999.0f;
static int dbg_counter = 0;
static int telem_counter = 0;

void controlLoop(ICM42688 &imu, Supervisor_typedef *sup,
                 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {

  // ----- Update timing stats -----
  uint32_t start_us = micros();

  static uint32_t last_us = 0;
  if (last_us == 0) {
    last_us = start_us;
    return;
  }

  uint32_t dt_us = start_us - last_us;
  last_us = start_us;

  sup->timing.dt_us = dt_us;
  sup->timing.exec_time_us = 0;
  sup->timing.last_tick_us = start_us;

  if (dt_us < sup->timing.min_dt_us) sup->timing.min_dt_us = dt_us;
  if (dt_us > sup->timing.max_dt_us) sup->timing.max_dt_us = dt_us;
  sup->timing.sum_dt_us += dt_us;
  sup->timing.count++;

  if (dt_us > CONTROL_PERIOD_US + 100){
    sup->timing.overruns++;
  }

  // ---- IMU: read & update pitch_rad / pitch_rate ----
  // ---- IMU: read & update orientation / pitch using Mahony ----
  if (dataReady) {
    dataReady = false;
    imu.getAGT();

    uint32_t now_imu_us = micros();

    // --- Raw accelerometer (assumed in "g" units) ---
    float ax = imu.accX();
    float ay = imu.accY();
    float az = imu.accZ();

    // --- Raw gyro (deg/s) ---
    float gx_dps = imu.gyrX();
    float gy_dps = imu.gyrY();
    float gz_dps = imu.gyrZ();

    // Convert gyro to rad/s for Mahony
    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;

    // --- Compute dt since last update ---
    float dt = (now_imu_us - sup->imu.last_update_us) * 1e-6f;
    if (dt < 0.0005f || dt > 0.005f) {
      dt = CONTROL_PERIOD_US * 1e-6f;  // fallback ~1 ms
    }

    // --- Run full 3D Mahony update ---
    mahonyUpdateIMU(gx, gy, gz, ax, ay, az, dt);

    // --- Extract pitch from quaternion ---
    // Using standard aerospace convention:
    // roll  = atan2(2(q0q1 + q2q3), 1 - 2(q1^2 + q2^2))
    // pitch =  asin(2(q0q2 - q3q1))
    // yaw   = atan2(2(q0q3 + q1q2), 1 - 2(q2^2 + q3^2))
    float pitch_rad = asinf(2.0f * (q0*q2 - q3*q1));

    // --- Approximate pitch rate from gyro (pick correct axis!) ---
    // If your pitch axis is better aligned with gy, swap to gy.
    float pitch_rate_raw = gx;   // rad/s

    const float rate_alpha = 0.03f; // LPF against vibration
    float pitch_rate =
        rate_alpha * pitch_rate_raw +
        (1.0f - rate_alpha) * sup->imu.pitch_rate;

    // --- Store into supervisor IMU struct ---
    sup->imu.pitch_rad      = pitch_rad;
    sup->imu.pitch_rate     = pitch_rate;
    sup->imu.valid          = true;
    sup->imu.last_update_us = now_imu_us;

    // Optional: telemetry (re-enable if you want)
    /*
    if (++dbg_counter >= TELEMETRY_DECIMATE) {
      dbg_counter = 0;
      float pitch_deg      = pitch_rad * RAD_TO_DEG;
      float pitch_rate_deg = pitch_rate * RAD_TO_DEG;

      Serial.printf(
        "{\"t\":%lu,"
        "\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
        "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f,"
        "\"pitch\":%.3f,"
        "\"pitch_rate\":%.3f}\r\n",
        micros(),
        ax, ay, az,
        gx_dps, gy_dps, gz_dps,
        pitch_deg,
        pitch_rate_deg
      );
    }
    */
  }


  // ---- Update RC PWM input ----
  updateRC(sup);

  // ---- Core control loop body ----
  switch (sup->mode) {
  case SUP_MODE_IDLE: {
    // --- Send zero torque (motor free) ---

    if (++telem_counter >= TELEMETRY_DECIMATE) {
      telem_counter = 0;

      CAN_message_t msg1;
      msg1.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
      msg1.len = 8;
      msg1.flags.extended = 1;
      canPackFloat(0.0f, msg1.buf);
      canPackFloat(0.0f, msg1.buf + 4);

      CAN_message_t msg2;
      msg2.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);
      msg2.len = 8;
      msg2.flags.extended = 1;
      canPackFloat(0.0f, msg2.buf);
      canPackFloat(0.0f, msg2.buf + 4);

      can.write(msg1);
      can.write(msg2);
    }

    break;
  }
 
  case SUP_MODE_BALANCE_TWR: {
    balance_TWR_mode(sup, can);
    break;
  }

  default: {
    break;
  }
  }

  // ---- Finish timing measurement ----
  sup->timing.exec_time_us = micros() - start_us;
}
