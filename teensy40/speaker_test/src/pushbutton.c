#include "pushbutton.h"

static inline PBState sample_pressed(const PBHandle* pb) {
  int level = digitalReadFast(pb->pin);          // HIGH or LOW
  bool pressed = pb->invert ? (level == LOW)     : (level == HIGH);
  return pressed ? PB_PRESSED : PB_RELEASED;
}

void pb_init(PBHandle* pb, uint8_t pin, bool use_pullup, uint32_t debounce_us) {
  pb->pin = pin;
  pb->invert = use_pullup;       // pressed == LOW when using internal pullup
  pb->debounce_us = debounce_us ? debounce_us : 50000u; // default 50ms if zero
  if (use_pullup) pinMode(pin, INPUT_PULLUP);
  else            pinMode(pin, INPUT);
  uint32_t now = micros();
  PBState s = sample_pressed(pb);
  pb->state = s;
  pb->_last_sampled = s;
  pb->_t_change_us = now;
  pb->changed = false;
}

void pb_update(PBHandle* pb, uint32_t now_us) {
  PBState inst = sample_pressed(pb);

  if (inst != pb->_last_sampled) {
    // New instantaneous sample diverged; restart debounce timer
    pb->_last_sampled = inst;
    pb->_t_change_us = now_us;
    return;
  }

  // If the instantaneous sample has matched long enough and differs from state, commit
  if (inst != pb->state) {
    uint32_t dt = (uint32_t)(now_us - pb->_t_change_us);
    if (dt >= pb->debounce_us) {
      pb->state = inst;
      pb->changed = true;  // latch the event
    }
  }
}

PBState pb_read_raw(PBHandle* pb) {
  return sample_pressed(pb);
}
