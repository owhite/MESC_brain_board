#include "supervisor.h"
#include "torque_reps.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ---------------- Logging ----------------
struct LogEntry {
  unsigned long t_us;
  float torque_left;
  float torque_right;
  float pos_rad;
  float vel_rad_s;
};

#define LOGLEN 500
static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

#define SEND_TORQUE 1

// ---------------- Control constants ----------------
constexpr float TORQUE_CLAMP   = 4.0f;    // max |Nm| per wheel
constexpr float SAFETY_SCALE   = 0.5f;   // global scaling (tune; set to 1.0f when confident)
constexpr float THETA_EQ       = 0.0f;    // body upright = 0 rad
constexpr float THETA_FAIL_RAD = 0.78f;    // ~34 deg: beyond this, bail to idle

static int report_counter = 0;

// One-shot per entry
static bool first_entry = true;
static unsigned long start_time = 0;

// ---------------- Main TWR balance mode ----------------
void torque_reps(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can)
{
    if (!sup) return;

    // ---------------- Initialize on first entry ----------------
    if (first_entry) {
        first_entry = false;
        logIndex    = 0;
        start_time  = micros();
    }

    // ---------------- Resolve parameters (use supervisor values if set) ----------------
    float    pulse_torque = (sup->user_pulse_torque != 0.0f) ? sup->user_pulse_torque : DEFAULT_PULSE_TORQUE;
    uint32_t pulse_us     = (sup->user_pulse_us   != 0)      ? sup->user_pulse_us     : DEFAULT_PULSE_US;

    unsigned long now_time = micros();
    unsigned long elapsed  = now_time - start_time;

    digitalWrite(TEST_PIN, test_pin_state);
    test_pin_state = !test_pin_state;

    float torque_left  = sup->user_pulse_torque;
    float torque_right = sup->user_pulse_torque;

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

    float pitch_deg      = sup->imu.pitch_rad * 180.0f / PI;
    float pitch_rate_deg = sup->imu.pitch_rate * 180.0f / PI;

    // {'cmd': 'torque_reps', 'pulse_torque': 0.4, 'pulse_us': 250000}

    // ---------------- Telemetry ----------------
    if (++report_counter >= TELEMETRY_DECIMATE) {
        report_counter = 0;

	/*
        Serial.printf(
          "{\"t\":%lu,"
          "\"pitch\":%.3f,"
          "\"pitch_rate\":%.3f,"
          "\"u\":%.3f}\r\n",
          micros(),
          pitch_deg,
          pitch_deg,
	  torque_left
        );
	*/
    }

    // ---------------- Logging ----------------
    if (logIndex < LOGLEN) {
        logBuffer[logIndex++] = {
            elapsed,
            torque_left,
            torque_right,
	    sup->esc[0].state.pos_rad, 
	    sup->esc[0].state.vel_rad_s
        };
    }

    // ---------------- Optional timed exit ----------------
    if (elapsed >= pulse_us) {
        Serial.println("{\"samples\":[");
        for (int i = 0; i < logIndex; i++) {
            Serial.printf(
              "{\"t\":%lu,\"uL\":%.4f,\"uR\":%.4f,\"pos_rad\":%.4f,\"vel_rad\":%.4f}%s\n",
              logBuffer[i].t_us,
              logBuffer[i].torque_left,
              logBuffer[i].torque_right,
	      logBuffer[i].pos_rad,
	      logBuffer[i].vel_rad_s,
              (i < logIndex - 1) ? "," : ""
            );
        }
        Serial.println("]}");

        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
    }
}
