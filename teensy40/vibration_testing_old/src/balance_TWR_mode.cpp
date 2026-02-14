#include "supervisor.h"
#include "verify_angle.h"
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


#define SEND_TORQUE 1

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

    digitalWrite(TEST_PIN, test_pin_state);
    test_pin_state = !test_pin_state;

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

        Serial.printf(
          "{\"t\":%lu,"
          "\"pitch\":%.3f,"
          "\"pitch_rate\":%.3f,"
          "\"u\":%.3f,"
          "\"x\":%.3f,"
          "\"x_dot\":%.3f}\r\n",
          micros(),
          pitch_deg,
	  (sup->esc[0].state.pos_rad  * 90.0f / PI) - 24, 
          // pitch_rate_deg,
          u,
          x_wheel,
          x_dot
        );
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
