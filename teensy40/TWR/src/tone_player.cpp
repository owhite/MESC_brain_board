#include "tone_player.h"
#include <IntervalTimer.h>
#include "tones.h"

// Single speaker: one IntervalTimer + one pin.
static IntervalTimer s_tpTimer;
static uint8_t       s_tpPin   = 255;
static TonePlayer*   s_default_tp = nullptr;
static volatile bool s_level   = false;
static volatile bool s_half_volume_mode = false;
static volatile uint8_t s_pwm_phase = 0u; // 0..3, used only in half-volume mode

static constexpr uint32_t ZERO_CROSS_BEEP_HZ = 2800u;
static constexpr uint32_t ZERO_CROSS_BEEP_MS = 10u;
static constexpr uint32_t ZERO_CROSS_BEEP_GAP_MS = 0u;
static constexpr uint8_t ZERO_CROSS_BEEP_VOLUME_PCT = 50u;

static constexpr uint32_t CAL_SONG_NOTE_MS = 120u;
static constexpr uint32_t CAL_SONG_GAP_MS = 80u;
static constexpr uint32_t CAL_SONG_NOTES[] = {
  NOTE_C4, NOTE_C4, NOTE_C4, NOTE_F4
};

static constexpr uint32_t IDLE_LONG_SONG_NOTE_MS = 120u;
static constexpr uint32_t IDLE_LONG_SONG_GAP_MS = 80u;
static constexpr uint32_t IDLE_LONG_SONG_NOTES[] = {
  NOTE_F4, NOTE_F4, NOTE_F4, NOTE_C4
};

static bool s_cal_song_active = false;
static uint8_t s_cal_song_index = 0u;
static bool s_idle_long_song_active = false;
static uint8_t s_idle_long_song_index = 0u;

static void tp_isr_toggle() {
  if (!s_half_volume_mode) {
    s_level = !s_level;
    digitalWriteFast(s_tpPin, s_level ? HIGH : LOW);
    return;
  }

  // Half-volume mode: preserve nominal frequency with 25% duty.
  // For a full wave split into 4 equal phases, output HIGH for phase 0 only.
  s_pwm_phase = (uint8_t)((s_pwm_phase + 1u) & 0x03u);
  digitalWriteFast(s_tpPin, (s_pwm_phase == 0u) ? HIGH : LOW);
}

static void tp_start_osc(uint32_t freq_hz, bool half_volume) {
  if (freq_hz == 0 || s_tpPin == 255) {
    s_tpTimer.end();
    digitalWriteFast(s_tpPin, LOW);
    s_level = false;
    s_half_volume_mode = false;
    s_pwm_phase = 0u;
    return;
  }
  const uint32_t divisor = half_volume ? 250000UL : 500000UL;
  uint32_t period_us = divisor / (freq_hz ? freq_hz : 1);
  if (period_us == 0) period_us = 1;

  s_level = false;
  s_half_volume_mode = half_volume;
  s_pwm_phase = 0u;
  digitalWriteFast(s_tpPin, LOW);

  // (re)start the PIT channel and set a solid priority
  if (s_tpTimer.begin(tp_isr_toggle, period_us)) {
    s_tpTimer.priority(64);   // 0 = highest, 255 = lowest; 64 is safely above USB
  }
}

static void tp_stop_osc() {
  s_tpTimer.end();
  digitalWriteFast(s_tpPin, LOW);
  s_level = false;
  s_half_volume_mode = false;
  s_pwm_phase = 0u;
}

void tone_init(TonePlayer* tp, uint8_t pin) {
  tp->pin = pin;
  tp->state = TONE_IDLE;
  tp->play_end_us = 0;
  tp->silence_end_us = 0;
  tp->freq_hz = 0;
  s_default_tp = tp;

  s_tpPin = pin;
  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, LOW);
}

void tone_start_volume(TonePlayer* tp, uint32_t freq_hz, uint32_t dur_ms, uint32_t silence_ms, uint8_t volume_pct) {
  const uint8_t vol = (volume_pct == 0u) ? 1u : (volume_pct > 100u ? 100u : volume_pct);
  const bool half_volume = (vol <= 50u);
  tp->freq_hz = freq_hz;

  const uint32_t now = micros();
  if (freq_hz > 0 && dur_ms > 0) {
    // Start the tone immediately
    tp->play_end_us = now + (dur_ms * 1000UL);
    tp_start_osc(freq_hz, half_volume);
    tp->state = TONE_PLAYING;
  } else {
    // No tone: go straight to silence if requested
    tp_stop_osc();
    if (silence_ms > 0) {
      tp->silence_end_us = now + (silence_ms * 1000UL);
      tp->state = TONE_SILENCE;
    } else {
      tp->state = TONE_IDLE;
    }
  }

  // Always program the post-tone silence (even if 0)
  tp->silence_end_us = now + (dur_ms * 1000UL) + (silence_ms * 1000UL);
}

void tone_start(TonePlayer* tp, uint32_t freq_hz, uint32_t dur_ms, uint32_t silence_ms) {
  tone_start_volume(tp, freq_hz, dur_ms, silence_ms, 100u);
}

void tone_stop(TonePlayer* tp) {
  tp_stop_osc();
  tp->state = TONE_IDLE;
  tp->play_end_us = 0;
  tp->silence_end_us = 0;
  tp->freq_hz = 0;
}

void tone_update(TonePlayer* tp, uint32_t now_us) {
  switch (tp->state) {
    case TONE_IDLE:
      // nothing to do
      break;

    case TONE_PLAYING:
      if ((int32_t)(now_us - tp->play_end_us) >= 0) {
        // End of tone -> start/post silence (or go idle if silence already elapsed)
        tp_stop_osc();
        if ((int32_t)(now_us - tp->silence_end_us) >= 0) {
          tp->state = TONE_IDLE;
        } else {
          tp->state = TONE_SILENCE;
        }
      }
      break;

    case TONE_SILENCE:
      if ((int32_t)(now_us - tp->silence_end_us) >= 0) {
        tp->state = TONE_IDLE;
      }
      break;
  }
}

void start_calibrate_song(TonePlayer* tp) {
  tone_stop(tp);
  s_cal_song_active = true;
  s_cal_song_index = 0u;
  tone_start(tp,
             CAL_SONG_NOTES[s_cal_song_index],
             CAL_SONG_NOTE_MS,
             CAL_SONG_GAP_MS);
  s_cal_song_index++;
}

void start_calibrate_song() {
  if (s_default_tp == nullptr) return;
  start_calibrate_song(s_default_tp);
}

void start_idle_long_hold_song(TonePlayer* tp) {
  tone_stop(tp);
  s_idle_long_song_active = true;
  s_idle_long_song_index = 0u;
  tone_start(tp,
             IDLE_LONG_SONG_NOTES[s_idle_long_song_index],
             IDLE_LONG_SONG_NOTE_MS,
             IDLE_LONG_SONG_GAP_MS);
  s_idle_long_song_index++;
}

void start_idle_long_hold_song() {
  if (s_default_tp == nullptr) return;
  start_idle_long_hold_song(s_default_tp);
}

void update_idle_long_hold_song(TonePlayer* tp) {
  if (!s_idle_long_song_active) return;
  if (!tone_is_idle(tp)) return;

  if (s_idle_long_song_index >= (sizeof(IDLE_LONG_SONG_NOTES) / sizeof(IDLE_LONG_SONG_NOTES[0]))) {
    s_idle_long_song_active = false;
    return;
  }

  tone_start(tp,
             IDLE_LONG_SONG_NOTES[s_idle_long_song_index],
             IDLE_LONG_SONG_NOTE_MS,
             IDLE_LONG_SONG_GAP_MS);
  s_idle_long_song_index++;
}

void update_idle_long_hold_song() {
  if (s_default_tp == nullptr) return;
  update_idle_long_hold_song(s_default_tp);
}

void update_calibrate_song(TonePlayer* tp) {
  if (!s_cal_song_active) return;
  if (!tone_is_idle(tp)) return;

  if (s_cal_song_index >= (sizeof(CAL_SONG_NOTES) / sizeof(CAL_SONG_NOTES[0]))) {
    s_cal_song_active = false;
    return;
  }

  tone_start(tp,
             CAL_SONG_NOTES[s_cal_song_index],
             CAL_SONG_NOTE_MS,
             CAL_SONG_GAP_MS);
  s_cal_song_index++;
}

void update_calibrate_song() {
  if (s_default_tp == nullptr) return;
  update_calibrate_song(s_default_tp);
}

void balance_zero_cross_tweet(TonePlayer* tp) {
  tone_start_volume(tp,
                    ZERO_CROSS_BEEP_HZ,
                    ZERO_CROSS_BEEP_MS,
                    ZERO_CROSS_BEEP_GAP_MS,
                    ZERO_CROSS_BEEP_VOLUME_PCT);
}

void balance_zero_cross_tweet() {
  if (s_default_tp == nullptr) return;
  balance_zero_cross_tweet(s_default_tp);
}
