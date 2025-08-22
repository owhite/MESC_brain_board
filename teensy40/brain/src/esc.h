#ifndef ESC_H
#define ESC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration for an ESC instance
typedef struct {
  char     name[32];
  uint16_t node_id;         // CAN or network node ID
  int16_t  pole_pairs;      // motor pole pairs
  int16_t  encoder_offset;  // encoder electrical offset
  int16_t  max_amps;        // current limit (A * 1 scaling or your fixed-point)
  int16_t  max_volts;       // max bus voltage (units per your scaling)
  int16_t  min_volts;       // min bus voltage (brownout threshold)
  int16_t  direction;       // +1 or -1
} ESC_config;

// Raw telemetry coming from the ESC
typedef struct {
  int16_t adc1;
  int16_t adc2;
  int16_t bus_volt;
  int16_t bus_current;
  int16_t FOC_angle;
  int16_t TMOS;             // MOSFET temperature (scaled)
  int16_t TMOT;             // Motor temperature (scaled)
} ESC_raw_typedef;

// 
// Fault / error codes bitfields
typedef enum {
  ESC_ERR_OK        = 0,        // no error
  ESC_ERROR         = 1u << 0,  // generic fault
  ESC_OVERHEAT      = 1u << 1,  // over-temperature
  ESC_OVERCURRENT   = 1u << 2,  // over-current
  ESC_CAN           = 1u << 3,  // CAN bus comms fault
  ESC_RESPONSE      = 1u << 4   // controller not responding / bad response
} ESC_error_flags_e;

// Main ESC state
typedef struct {
  uint16_t         error;      // error code (0 = OK)

  // Setpoints / commands to send to the ESC
  int16_t         adc1_req;   // requested throttle (scaled)
  int16_t         direction;  // +1 or -1

  // Live data and configuration
  ESC_raw_typedef Raw;
  ESC_config      Config;
} ESC_motor_typedef;


// Initialization API
void init_ESC(ESC_motor_typedef *esc, const char *name, uint16_t node_id);

// Set one or more error bits
static inline void esc_set_error(ESC_motor_typedef *esc, uint16_t flags) {
  esc->error |= flags;
}

// Clear one or more error bits
static inline void esc_clear_error(ESC_motor_typedef *esc, uint16_t flags) {
  esc->error &= (uint16_t)~flags;
}

// Check if any error bit is set
static inline bool esc_has_any_error(const ESC_motor_typedef *esc) {
  return esc->error != ESC_ERR_OK;
}

// Check if a specific bit (or bits) is set
static inline bool esc_has_error(const ESC_motor_typedef *esc, uint16_t flags) {
  return (esc->error & flags) != 0;
}

// Clear all errors
static inline void esc_clear_all_errors(ESC_motor_typedef *esc) {
  esc->error = ESC_ERR_OK;
}

#ifdef __cplusplus
}
#endif

#endif // ESC_H
