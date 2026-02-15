#include "supervisor.h"
#include "balance_TWR_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ---------------- Logging ----------------
struct LogEntry {
  unsigned long t_us;
  float torque_left;
  float torque_right;
  float theta;
  float theta_dot;
  float x_wheel;
  float x_dot;
};

#define LOGLEN 500
static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

// ---------------- Discrete LQR gains ----------------
// State ordering assumed: [theta, theta_dot, x_wheel, x_dot]^T
static const float K_disc[4] = {
  10.28505873560549f,
  1.0301541575776232f,
  -2.9755190901969173f,
  -5.948216517508814f
};

constexpr float WHEEL_RADIUS_M = 0.040f; // use your real value


#define SEND_TORQUE 0

// ---------------- Control constants ----------------
constexpr float TORQUE_CLAMP   = 4.0f;    // max |Nm| per wheel
constexpr float SAFETY_SCALE   = 0.5f;   // global scaling (tune; set to 1.0f when confident)
constexpr float THETA_EQ       = 0.0f;    // body upright = 0 rad
constexpr float THETA_FAIL_RAD = 0.6f;    // ~34 deg: beyond this, bail to idle

static int report_counter = 0;

// Continuous wheel angle state
static bool  unwrap_init = false;
static float prev_L = 0.0f, prev_R = 0.0f;
static float unwrap_L = 0.0f, unwrap_R = 0.0f;

// Velocity filtering state
static float vel_filt_L = 0.0f;
static float vel_filt_R = 0.0f;

// One-shot per entry
static bool first_entry = true;
static unsigned long start_time = 0;

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

/* 
POTENTIAL ISSUES:

*) SEE ALSO: system_assumptions.md

*) Using filtered velocity for x_dot but unwrapped angle for x_wheel
That’s not “wrong,” but note the implication:
x_wheel is derived from position integration (unwrap)
x_dot is derived from (filtered) ESC-reported velocity, not the derivative of x_wheel
So position and velocity can become slightly inconsistent (especially if ESC velocity has bias or filtering). If you use both in LQR, that’s usually okay, but if you ever see weird state mismatch, this is a candidate.
A possible fix is x_dot = (x_wheel - x_wheel_prev)/dt (and optionally filter that),

*) dt 
Unlike some code where dt only scales a constant, here dt directly affects alpha, so if your real loop jitters, your filter cutoff jitters too.
At small jitter it’s fine. If you ever see weird filtering, you can:
feed dt from your measured dt_us (instead of constant), or
clamp dt to a sane range before using it in the filter (like you already do elsewhere for IMU dt)

*) M_PI
code uses both M_PI and PI. On Teensy/Arduino, PI is typically defined; M_PI is sometimes available but not guaranteed depending on includes. wrap uses M_PI, filter uses PI

*) Initialization
unwrap_init, prev_L, prev_R, unwrap_L, unwrap_R, vel_filt_L, vel_filt_R
They must be static or otherwise persistent across calls and must be reset when you exit balance mode (you are resetting unwrap_init=false in your mode exit paths—good).

*) WHEEL_RADIUS_M scaling
Correct conceptually (rad → meters), but note: x_wheel is now linear distance (m), while your controller gain vector K_disc must match this scaling. Just keep that consistent.

*) Limit maximum angle before shutting off

Add cutoff at ±45 deg.

*) Add angle offset calibration

*) Add theta_offset and subtract from roll.

*) Add roll-rate deadband and startup ramp

*) Timing / Supervisor Infrastructure
A. Confirm supervisor uses radians. 
B. Signature mismatch fixed. 
C. Tune

*/ 



// ---------------- Helper: update continuous wheel angles ----------------

static void updateWheelUnwrap(float pos_L_raw, float pos_R_raw,
                              float &x_wheel, float &x_dot,
                              float vel_L, float vel_R,
                              float dt)
{
  // Initialize unwrap on first call in this mode
  if (!unwrap_init) {
    prev_L = pos_L_raw;
    prev_R = pos_R_raw;
    unwrap_L = 0.0f;
    unwrap_R = 0.0f;
    vel_filt_L = vel_L;
    vel_filt_R = vel_R;
    unwrap_init = true;
  }

  // Compute incremental angles with wrap handling
  float dL = pos_L_raw - prev_L;
  float dR = pos_R_raw - prev_R;

  if (dL >  M_PI) dL -= 2.0f * M_PI;
  if (dL < -M_PI) dL += 2.0f * M_PI;
  if (dR >  M_PI) dR -= 2.0f * M_PI;
  if (dR < -M_PI) dR += 2.0f * M_PI;

  unwrap_L += dL;
  unwrap_R += dR;

  prev_L = pos_L_raw;
  prev_R = pos_R_raw;

  // Optional velocity low-pass filter (simple 1st-order)
  // Cutoff ~20 Hz at CONTROL_PERIOD
  const float fc    = 20.0f;
  const float RC    = 1.0f / (2.0f * PI * fc);
  const float alpha = dt / (dt + RC);

  vel_filt_L += alpha * (vel_L - vel_filt_L);
  vel_filt_R += alpha * (vel_R - vel_filt_R);

  // Forward position and velocity (average of two wheels)
  x_wheel = 0.5f * (unwrap_L + unwrap_R);
  x_dot   = 0.5f * (vel_filt_L + vel_filt_R);

  x_wheel *= WHEEL_RADIUS_M;
  x_dot   *= WHEEL_RADIUS_M;
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
#if SEND_TORQUE
    can.write(msgL);
    can.write(msgR);
#endif 
    sup->mode = SUP_MODE_IDLE;
    return;
  }

  // ---------------- Initialize on first entry ----------------
  if (first_entry) {
    first_entry = false;
    unwrap_init = false;
    logIndex    = 0;
    start_time  = micros();

    Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance mode started\"}");
  }

  unsigned long now_time = micros();
  unsigned long elapsed  = now_time - start_time;

  // ---------------- Sensor feedback ----------------
  // Body angle and rate from IMU (radians and rad/s)
  float theta     = sup->imu.pitch_rad - THETA_EQ;
  float theta_dot = sup->imu.pitch_rate;
  pitch_rate_rms.push(sup->imu.pitch_rate); 
  rate_rms_raw.push(sup->imu.pitch_rate_raw);
  rate_rms_filt.push(sup->imu.pitch_rate);

  // Wheel encoder positions (as reported by ESC, wrapped)
  float pos_L = sup->esc[0].state.pos_rad;
  float pos_R = sup->esc[1].state.pos_rad;

  float vel_L = sup->esc[0].state.vel_rad_s;
  float vel_R = sup->esc[1].state.vel_rad_s;

  // Control period DT (assumed constant; matches ISR period)
  const float dt = CONTROL_PERIOD_US * 1e-6f;

  float x_wheel = 0.0f;
  float x_dot   = 0.0f;
  updateWheelUnwrap(pos_L, pos_R, x_wheel, x_dot, vel_L, vel_R, dt);

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

#if SEND_TORQUE
    can.write(msgL);
    can.write(msgR);
#endif
    Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance aborted: tilt too large\"}");

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

  // Symmetric torque to both wheels (signs match ESC expectations)
  float torque_left  =  u;
  float torque_right = -u;

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

#if SEND_TORQUE
  can.write(msgL);
  can.write(msgR);
#endif

  // ---------------- Telemetry ----------------
  if (++report_counter >= TELEMETRY_DECIMATE) {
    report_counter = 0;

    float pitch_deg      = sup->imu.pitch_rad * 180.0f / PI;
    float pitch_rate_deg = sup->imu.pitch_rate * 180.0f / PI;
    uint32_t age_us = micros() - sup->imu.last_update_us;
		    
    Serial.printf(
		  "{\"t\":%lu,"
		  "\"pitch\":%.3f,"
		  "\"rate\":%.3f,"
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
		  rate_rms_raw.stddev(),          // raw gyro RMS (rad/s)
		  rate_rms_filt.stddev(),         // filtered RMS (rad/s)
		  (unsigned long)rate_rms_filt.n, // sample count
		  age_us,
		  sup->imu.valid ? 1 : 0,
		  sup->timing.exec_time_us,
		  sup->timing.overruns
		  );

    // reset window (so this RMS corresponds to the last ~0.1s at 10Hz printing)
    pitch_rate_rms.reset();
    rate_rms_raw.reset();
    rate_rms_filt.reset();
  }

  // ---------------- Logging ----------------
  if (logIndex < LOGLEN) {
    logBuffer[logIndex++] = {
      elapsed,
      torque_left,
      torque_right,
      theta,
      theta_dot,
      x_wheel,
      x_dot
    };
  }

  // ---------------- Optional timed exit ----------------
  // Currently disabled (as you had it).
  if (elapsed > sup->user_total_us && false) {
    Serial.println("{\"samples\":[");
    for (int i = 0; i < logIndex; i++) {
      Serial.printf(
		    "{\"t\":%lu,\"uL\":%.4f,\"uR\":%.4f,"
		    "\"theta\":%.4f,\"theta_dot\":%.4f,"
		    "\"x\":%.4f,\"x_dot\":%.4f}%s\n",
		    logBuffer[i].t_us,
		    logBuffer[i].torque_left,
		    logBuffer[i].torque_right,
		    logBuffer[i].theta,
		    logBuffer[i].theta_dot,
		    logBuffer[i].x_wheel,
		    logBuffer[i].x_dot,
		    (i < logIndex - 1) ? "," : ""
		    );
    }
    Serial.println("]}");

    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    unwrap_init = false;
  }
}
