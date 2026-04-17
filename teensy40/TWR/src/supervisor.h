#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <Arduino.h>
#include "ESC.h"
#include <FlexCAN_T4.h>
#include "CAN_helper.h"
#include "main.h"

#define TELEMETRY_DECIMATE 100

#define SUPERVISOR_MAX_ESCS   4
#define RC_INPUT_MAX_PINS     8
#define RC_INPUT_MIN_US       1000
#define RC_INPUT_MAX_US       2000
#define RC_INPUT_TIMEOUT_US   100000 // 100 ms

#define CONTROL_LOOP_PRIORITY 16
#define CONTROL_PERIOD_US     1000   // 1 kHz

// ---------------- Defaults ----------------
static constexpr uint32_t DEFAULT_PULSE_US     = 0;
static constexpr uint32_t DEFAULT_TOTAL_US     = 0;
static constexpr float    DEFAULT_KD_TERM = 0.0f;
static constexpr float    DEFAULT_KP_TERM = 0.0f;
static constexpr float    DEFAULT_PULSE_TORQUE = 0.0f;

// ---------------- Loop Timing Stats ----------------
// Captures jitter and execution time statistics of the control loop.
struct LoopTimingStats {
  uint32_t last_tick_us;   // Timestamp of last loop tick
  uint32_t dt_us;          // Time between consecutive loop ticks
  uint32_t exec_time_us;   // Execution time of last loop

  uint32_t min_dt_us;      // Minimum observed loop period
  uint32_t max_dt_us;      // Maximum observed loop period
  uint64_t sum_dt_us;      // Sum of loop periods (for averaging)
  uint32_t count;          // Number of loop samples collected

  uint32_t overruns;       // Number of times loop exceeded CONTROL_PERIOD_US
};

// ---------------- Telemetry Stats ----------------
// Measures blocking time when writing telemetry over Serial1.
struct SerialStats {
  uint32_t last_block_us;   // Duration of the most recent blocking write
  uint32_t max_block_us;    // Longest blocking time observed
  uint64_t sum_block_us;    // Accumulated total of all blocking times
  uint32_t count;           // Number of writes measured
};

// ---------------- RC Input ----------------
// Raw RC signal input (pulse width in microseconds).
struct RCInputRaw {
  volatile uint16_t raw_us;     // Most recent raw PWM input
  volatile uint32_t last_update;// Timestamp of last update (µs)
};

// Normalized RC channel data after processing.
struct RCChannel {
  float norm;       // Normalized value in [-1.0, 1.0], centered at stick midpoint
  uint16_t raw_us;  // Raw pulse width in µs
  bool valid;       // True if the channel is valid and updated recently
};

// ---------------- IMU ----------------
struct IMUState {
  bool valid = false;
  float pitch_rad = 0.0f;
  float pitch_rate_raw = 0.0f;  // unfiltered gyro-derived pitch rate (rad/s)
  float pitch_rate = 0.0f;   // rad/s
  float accel_mag_g = 0.0f;      // |a| in g-units from IMU sample
  uint8_t accel_valid = 0u;      // 1 when accel magnitude gate passes
  float mahony_int_fb_y = 0.0f;  // Mahony integral feedback (pitch-axis proxy)
  uint32_t last_update_us = 0;
};

// ---------------- Supervisor Modes ----------------
// High-level supervisor state machine.
enum SupervisorMode {
  SUP_MODE_IDLE = 0,   // System idle, no active control
  SUP_MODE_CALIBRATE,  // User-held calibration from kickstand to upright target
  SUP_VERIFY_ANGLE,    // IMU verification: zero torque, report pitch/rate
  SUP_MODE_BALANCE_HOLD, // Balance with position-hold only (no RC motion commands)
  SUP_MODE_BALANCE_TWR,  // Balance with RC motion shaping enabled
  SUP_MODE_BALANCE_DEBUG, // Debug clone of balance mode for A/B testing
  SUP_MODE_TEST_CAN
};

// ---------------- Supervisor ----------------
// Central state container for the system.
// Holds ESC state, IMU, RC input, timing stats, telemetry stats, etc.
struct Supervisor_typedef {
  uint16_t       esc_count;                                // Number of ESCs managed
  ESC            esc[SUPERVISOR_MAX_ESCS];                 // Array of ESC objects
  uint32_t       last_esc_heartbeat_us[SUPERVISOR_MAX_ESCS]; // Last CAN heartbeat timestamps per ESC
  uint32_t       esc_alive_false_count[SUPERVISOR_MAX_ESCS]; // Per-ESC count of control ticks where alive=false

  SupervisorMode mode;                                     // Current supervisor mode
  IMUState imu;                                            // IMU estimate for balance mode

  float user_setpoint = M_PI;
  float user_Kp_term = 0.0f;
  float user_Kd_term = 0.0f; 
  float user_pulse_torque   = 0.2f;    // amplitude of torque pulse
  float user_test_tau_left = 1.0f;     // test_can constant torque for node 11
  float user_test_tau_right = -1.0f;   // test_can constant torque for node 12
  bool user_verify_motor_enable = false; // if true, VERIFY_ANGLE sends non-zero torque
  float user_verify_tau_left = 2.0f;     // left torque during verify+motor mode
  float user_verify_tau_right = -2.0f;   // right torque during verify+motor mode
  bool user_rc_drive_enable = false;     // if true, TEST_CAN uses RC throttle/steering torque mixing
  uint8_t user_rc_throttle_ch = 1u;      // 0-based index into rc[] array (RC_INPUT2 default)
  uint8_t user_rc_steer_ch = 0u;         // 0-based index into rc[] array (RC_INPUT1 default)
  bool user_rc_throttle_invert = true;   // true: lower PWM => positive throttle command
  bool user_rc_steer_invert = true;      // true: lower PWM => positive steer command
  float user_rc_deadband = 0.05f;        // deadband in normalized stick units
  float user_rc_max_torque_nm = 1.0f;    // max absolute torque requested by RC mixer
  uint32_t user_pulse_us    = 1000;   // duration of pulse (µs)
  uint32_t user_total_us    = 1000;
  bool user_tx_enable       = true;   // Enables/disables torque command TX during test mode.
  // CAN reliability improvement baseline:
  // 2000 us (500 Hz) default command cadence used for stable dual-channel operation.
  uint32_t user_tx_period_us = 2000;  // Command TX period in microseconds (2000 us = 500 Hz).

  SerialStats serial1_stats;                               // Telemetry serial performance stats

  LoopTimingStats timing;                                  // Loop timing / jitter stats
  uint32_t last_health_ms;                                 // Timestamp of last health update (ms)

  RCInputRaw rc_raw[RC_INPUT_MAX_PINS];                    // Raw RC inputs
  RCChannel  rc[RC_INPUT_MAX_PINS];                        // Normalized RC channels
  uint8_t    rc_count;                                     // Number of RC channels active
};

// ---------------- Globals / Prototypes ----------------
extern volatile uint32_t g_control_pending_ticks; // Pending control ISR ticks to service
extern volatile uint32_t g_control_now_us;// Timestamp of control loop trigger (µs)

void controlLoop_isr(void); // ISR triggered at CONTROL_PERIOD_US

void controlLoop(Supervisor_typedef *sup, // Core control loop logic
                 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                 FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2);
void test_can_transmit_mode(Supervisor_typedef *sup,
                            FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1,
                            FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can2);

void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count);

void updateSupervisorRC(Supervisor_typedef *sup);   // Update RC input channels
void resetLoopTimingStats(Supervisor_typedef *sup); // Reset loop timing stats
void resetTelemetryStats(Supervisor_typedef *sup);  // Reset telemetry stats
float angle_diff(float target, float actual);


#endif
