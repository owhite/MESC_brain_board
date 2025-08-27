#ifndef PUSHBUTTON_H
#define PUSHBUTTON_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PB_RELEASED = 0,
  PB_PRESSED  = 1
} PBState;

typedef struct {
  uint8_t  pin;
  bool     invert;         // true if pressed == LOW (INPUT_PULLUP wiring)
  uint32_t debounce_us;    // debounce interval in microseconds
  PBState  state;          // current debounced (stable) state
  PBState  _last_sampled;  // last instantaneous sample
  uint32_t _t_change_us;   // time when sample first differed from state
  bool     changed;        // latched when state changes (cleared by consumer)
} PBHandle;

// Initialize the pushbutton. If use_pullup=true, pin is INPUT_PULLUP and pressed==LOW.
void pb_init(PBHandle* pb, uint8_t pin, bool use_pullup, uint32_t debounce_us);

// Call often (e.g., each loop). Pass a common timestamp (micros()) for consistency.
void pb_update(PBHandle* pb, uint32_t now_us);

// Optional: immediate raw read (non-debounced), using current pin level.
PBState pb_read_raw(PBHandle* pb);

// Helper: returns true if a debounced state change occurred since last call and clears the latch.
static inline bool pb_consume_change(PBHandle* pb, PBState* out_state) {
  if (!pb->changed) return false;
  pb->changed = false;
  if (out_state) *out_state = pb->state;
  return true;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif // PUSHBUTTON_H
