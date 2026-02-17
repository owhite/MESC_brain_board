#include "supervisor.h"
#include "position_control_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ---------------- Logging ----------------
struct LogEntry {
  unsigned long t_us;
  float torque;
  // other values to be assigned
  float theta;
  float theta_dot;
  float x_wheel;
  float x_dot;
};

#define LOGLEN 500
static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

constexpr float WHEEL_RADIUS_M = 0.05278f; 

// #define SEND_TORQUE
#define SEND_TELEMETRY

// ---------------- Control constants ----------------
constexpr float TORQUE_CLAMP   = 4.0f;    // max |Nm| per wheel
constexpr float SAFETY_SCALE   = 0.5f;    // global scaling (tune; set to 1.0f when confident)

static int report_counter = 0;

// Continuous wheel angle state
static bool  unwrap_init = false;
static float prev_wheel = 0.0f;
static float unwrap_wheel = 0.0f;

// Velocity filtering state
static float vel_filt_wheel = 0.0f;

// One-shot per entry
static bool first_entry = true;
static unsigned long start_time = 0;

// ---------------- Helper: update continuous wheel angles ----------------

static void updateWheelUnwrap(float pos_raw, float vel_raw, float dt) {
  // Initialize unwrap/filter state on first call in this mode.
  if (!unwrap_init) {
    prev_wheel     = pos_raw;
    unwrap_wheel   = 0.0f;
    vel_filt_wheel = vel_raw;
    unwrap_init    = true;
    return;
  }

  // Compute wrapped increment in [-PI, PI].
  float dpos = pos_raw - prev_wheel;
  if (dpos > PI) dpos -= 2.0f * PI;
  if (dpos < -PI) dpos += 2.0f * PI;

  unwrap_wheel += dpos;
  prev_wheel    = pos_raw;

  // Optional 1st-order low-pass filter on velocity.
  const float fc    = 20.0f;
  const float RC    = 1.0f / (2.0f * PI * fc);
  const float alpha = dt / (dt + RC);
  vel_filt_wheel += alpha * (vel_raw - vel_filt_wheel);
} 

// ---------------- Main TWR balance mode ----------------
void position_control_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can)
{
  if (!sup) return;

  if (!sup->esc[0].state.alive) {
    // If either ESC died go idle
    first_entry = true;
    unwrap_init = false;

    // Send zero torque
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(0.0f, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);

    #if SEND_TORQUE
    can.write(msg);
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
  }

  unsigned long now_time = micros();
  unsigned long elapsed  = now_time - start_time;

  // Wheel encoder positions (as reported by ESC, wrapped)
  float position = sup->esc[0].state.pos_rad;
  float velocity = sup->esc[0].state.vel_rad_s;

  // Control period DT (assumed constant; matches ISR period)
  const float dt = CONTROL_PERIOD_US * 1e-6f;

  // If unwrap occurs perform here
  updateWheelUnwrap(position, velocity, dt);

  // ---------------- CONTROL THEORY HERE ----------------
  float torque = 0.0f;

  // Global safety scaling (start small during tuning)
  torque *= SAFETY_SCALE;

  // Clamp torque
  if (torque > TORQUE_CLAMP)  torque = TORQUE_CLAMP;
  if (torque < -TORQUE_CLAMP) torque = -TORQUE_CLAMP;

  // ---------------- Send torque over CAN ----------------
  #ifdef SEND_TORQUE

  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);

  msg.len = 8;
  msg.flags.extended = 1;

  canPackFloat(torque,  msg.buf);
  canPackFloat(0.0f,    msg.buf + 4);

  can.write(msg);
  #endif

  // ---------------- Logging ----------------
  if (logIndex < LOGLEN) {
    logBuffer[logIndex++] = {
      elapsed,
      torque
    };
  }

  // ---------------- Optional timed exit ----------------
  // Currently disabled (as you had it).
  if (elapsed > sup->user_total_us && false) {
    #ifdef SEND_TELEMETRY
    Serial.println("{\"samples\":[");

    for (int i = 0; i < logIndex; i++) {
      // SERIAL PRINT ALL RELEVANT DATA
      // example:
      /*
      Serial.printf(
		    "{\"t\":%lu,\"uL\":%.4f,\"uR\":%.4f,"
		    "\"theta\":%.4f,\"theta_dot\":%.4f,"
		    "\"x\":%.4f,\"x_dot\":%.4f}%s\n",
		    logBuffer[i].t_us,
		    logBuffer[i].torque,
		    logBuffer[i].torque_right,
		    logBuffer[i].x_wheel,
		    logBuffer[i].x_dot,
		    (i < logIndex - 1) ? "," : ""
		    );

      */ 
    }
    /* 
    Serial.println("]}");
    */

    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
    unwrap_init = false;
  }
  #endif

}
