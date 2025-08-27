#ifndef SUPERVISOR_H
#define SUPERVISOR_H

// Teensy/Arduino-friendly, C/C++ compatible supervisor for multi-ESC robots.
// Role: Supervisory (high-level) controller coordinating low-level ESCs,
// IMU fusion, and real-time control tasks (balancing, gait, velocity planning).

#include <stdint.h>
#include <stdbool.h>
#include "esc.h"   // for ESC_motor_typedef

#ifdef __cplusplus
extern "C" {
#endif

// ============================== Tunables / Defaults ==============================

// Control loop rate (Hz). Teensy 4.0 can comfortably hit 1 kHz.
#ifndef SUPERVISOR_CTRL_HZ
#define SUPERVISOR_CTRL_HZ       1000
#endif

// Maximum number of ESCs present on the robot (adjust as needed).
#ifndef SUPERVISOR_MAX_ESCS
#define SUPERVISOR_MAX_ESCS      2
#endif

// Timeout thresholds (microseconds)
#ifndef SUPERVISOR_ESC_HEARTBEAT_TIMEOUT_US
#define SUPERVISOR_ESC_HEARTBEAT_TIMEOUT_US  (200000)   // 200 ms
#endif

#ifndef SUPERVISOR_IMU_TIMEOUT_US
#define SUPERVISOR_IMU_TIMEOUT_US            (100000)   // 100 ms
#endif

// ============================== Types / Enums ==============================

// High-level operating mode (single active value)
typedef enum {
  SUP_MODE_IDLE = 0,
  SUP_MODE_CALIBRATION,
  SUP_MODE_ARMED,
  SUP_MODE_MANUAL_RESET,
  SUP_MODE_TIPPED,
  SUP_MODE_SHUTTING_DOWN,
  SUP_MODE_BALANCING,
  SUP_MODE_FAULT
} SupervisorMode_e;

// Gait modes (extend as needed)
typedef enum {
  GAIT_IDLE = 0,
  GAIT_INPLACE,
  GAIT_TROT,        // example placeholder
  GAIT_WADDLE
} GaitMode_e;

// Bitwise error flags (0 = OK; multiple bits may be set)
typedef enum {
  SUP_ERR_OK           = 0u,
  SUP_ERR_ESC_COMM     = 1u << 0,  // lost/bad comms with one/more ESCs
  SUP_ERR_IMU_TIMEOUT  = 1u << 1,  // stale IMU data
  SUP_ERR_OVERVOLT     = 1u << 2,  // system bus too high
  SUP_ERR_UNDERVOLT    = 1u << 3,  // system bus too low / brownout risk
  SUP_ERR_OVERTEMP     = 1u << 4,  // supervisory temp sensor (if any)
  SUP_ERR_ESTIMATION   = 1u << 5,  // state estimator unhealthy (e.g., NaN)
  SUP_ERR_WATCHDOG     = 1u << 6   // control loop deadline miss
} SupervisorErrorFlags_e;

// Minimal IMU structure (float for clarity; switch to fixed-point if desired)
typedef struct {
  // Linear acceleration (m/s^2)
  float ax, ay, az;
  // Angular rates (rad/s)
  float gx, gy, gz;
  // Orientation quaternion (w,x,y,z)
  float qw, qx, qy, qz;
  // Timestamp (us) of last IMU update
  uint32_t t_us;
  // Validity flag
  bool valid;
} IMU_typedef;

// High-level setpoints coming from planner/UI/etc.
typedef struct {
  float v_forward_mps;    // desired forward velocity
  float yaw_rate_rps;     // desired yaw rate
  float body_pitch_rad;   // optional pitch setpoint (balancing ref)
} SupervisorSetpoints_typedef;

// Measured/estimated robot state (subset; extend as needed)
typedef struct {
  float v_forward_mps;    // estimated forward velocity
  float yaw_rate_rps;     // estimated yaw rate
  float body_pitch_rad;   // estimated pitch from IMU fusion
} SupervisorState_typedef;

// Top-level supervisory controller (brain)
typedef struct {
  // Configuration
  uint16_t           esc_count;                              // number of active ESCs (<= SUPERVISOR_MAX_ESCS)

  // Subsystems
  ESC_motor_typedef  esc[SUPERVISOR_MAX_ESCS];               // controlled motor subsystems
  IMU_typedef        imu;                                    // inertial data

  // Control
  SupervisorMode_e        mode;
  GaitMode_e              gait_mode;
  SupervisorSetpoints_typedef sp;                            // desired commands
  SupervisorState_typedef    xhat;                          // estimated state

  // Timing
  uint32_t t_now_us;            // current time (us)
  uint32_t t_prev_us;           // previous control tick (us)
  float    dt_s;                // control period (s), derived from timestamps

  // Health / supervision
  uint32_t last_esc_heartbeat_us[SUPERVISOR_MAX_ESCS];       // last time we heard from each ESC
  uint32_t last_imu_update_us;                               // last IMU update time
  uint16_t error_flags;                                      // bitmask of SupervisorErrorFlags_e

  // Logging / diagnostics (lightweight placeholders)
  uint32_t control_cycle_count;
  uint32_t can_rx_count;
  uint32_t can_tx_count;
} Supervisor_typedef;

// ============================== API ==============================

// Initialize supervisor. Sets defaults, zeroes state, and initializes ESC instances.
// esc_count must be <= SUPERVISOR_MAX_ESCS.
// esc_names: array of C strings for naming ESCs (may be NULL).
// node_ids: array of ESC node IDs (may be NULL -> treated as 0).
void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[], const uint16_t node_ids[]);

// ============================== Inline helpers ==============================

static inline float supervisor_clampf(float v, float vmin, float vmax) {
  return (v < vmin) ? vmin : (v > vmax) ? vmax : v;
}

// Error flag helpers
static inline void sup_set_error(Supervisor_typedef *sup, uint16_t flags) {
  sup->error_flags |= flags;
}

static inline void sup_clear_error(Supervisor_typedef *sup, uint16_t flags) {
  sup->error_flags &= (uint16_t)~flags;
}

static inline bool sup_has_any_error(const Supervisor_typedef *sup) {
  return sup->error_flags != SUP_ERR_OK;
}

static inline bool sup_has_error(const Supervisor_typedef *sup, uint16_t flags) {
  return (sup->error_flags & flags) != 0u;
}

static inline void sup_clear_all_errors(Supervisor_typedef *sup) {
  sup->error_flags = SUP_ERR_OK;
}

#ifdef __cplusplus
}
#endif

#endif // SUPERVISOR_H
