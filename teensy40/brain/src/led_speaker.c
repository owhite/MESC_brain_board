#include "led_speaker.h"
#include <string.h>

// ---------------- LED ----------------
static void LEDTask(void *arg) {
  LEDHandle *h = (LEDHandle*)arg;
  const TickType_t fast_half = pdMS_TO_TICKS(LED_FAST_PERIOD_MS / 2);
  const TickType_t slow_half = pdMS_TO_TICKS(LED_SLOW_PERIOD_MS / 2);

  pinMode(h->pin, OUTPUT);
  digitalWrite(h->pin, LOW);

  for (;;) {
    LEDState s = h->state;
    switch (s) {
      case LED_OFF:
        digitalWrite(h->pin, LOW);
        vTaskDelay(pdMS_TO_TICKS(20));  // small idle delay
        break;

      case LED_ON_CONTINUOUS:
        digitalWrite(h->pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(20));
        break;

      case LED_FAST_BLINK:
        digitalWrite(h->pin, HIGH); vTaskDelay(fast_half);
        digitalWrite(h->pin, LOW ); vTaskDelay(fast_half);
        break;

      case LED_SLOW_BLINK:
        digitalWrite(h->pin, HIGH); vTaskDelay(slow_half);
        digitalWrite(h->pin, LOW ); vTaskDelay(slow_half);
        break;

      default:
        vTaskDelay(pdMS_TO_TICKS(20));
        break;
    }
  }
}

LEDHandle* init_LED(uint8_t pin, LEDState initial_state,
                    UBaseType_t task_prio, uint16_t task_stack_words)
{
  LEDHandle *h = (LEDHandle*)pvPortMalloc(sizeof(LEDHandle));
  if (!h) return NULL;
  memset(h, 0, sizeof(*h));
  h->pin = pin;
  h->state = initial_state;

  BaseType_t ok = xTaskCreate(LEDTask, "LED", task_stack_words, h, task_prio, &h->task);
  if (ok != pdPASS) {
    vPortFree(h);
    return NULL;
  }
  return h;
}

// ---- Local "tone" replacement (no Arduino tone/PWM) ----
static volatile bool      spk_enabled = false;
static volatile uint32_t  spk_half_ticks = 0;   // half-period in RTOS ticks
static volatile uint8_t   spk_pin = 255;

// ---------------- Speaker ----------------
static inline uint16_t sound_to_freq(SpeakerSound s) {
  switch (s) {
    case WARNING1:      return WARNING1_FREQ_HZ;
    case WARNING2:      return WARNING2_FREQ_HZ;
    case TRACK_BALANCE: return TRACK_FREQ_HZ;
    default:            return 0;
  }
}

static inline TickType_t sound_to_dur_ticks(SpeakerSound s) {
  switch (s) {
    case WARNING1:      return pdMS_TO_TICKS(WARNING1_DUR_MS);
    case WARNING2:      return pdMS_TO_TICKS(WARNING2_DUR_MS);
    case TRACK_BALANCE: return pdMS_TO_TICKS(TRACK_DUR_MS);
    default:            return 0;
  }
}

static inline TickType_t ms_to_ticks_clamped(float ms) {
  TickType_t t = (TickType_t)(ms * (configTICK_RATE_HZ / 1000.0f) + 0.5f);
  return (t == 0) ? 1 : t; // never 0 (ensures at least 1 tick)
}

static void speaker_start(uint8_t pin, uint16_t freq_hz) {
  if (freq_hz == 0) { spk_enabled = false; return; }
  spk_pin = pin;
  pinMode(spk_pin, OUTPUT);
  // half period (ms) = 1000 / (2 * f)
  float half_ms = 1000.0f / (2.0f * (float)freq_hz);
  spk_half_ticks = ms_to_ticks_clamped(half_ms);
  spk_enabled = true;
}

static void speaker_stop(uint8_t pin) {
  (void)pin;
  spk_enabled = false;
  if (spk_pin != 255) {
    digitalWrite(spk_pin, LOW);
  }
}

static void SpeakerTask(void *arg) {
  SpeakerHandle *h = (SpeakerHandle*)arg;
  pinMode(h->pin, OUTPUT);
  digitalWrite(h->pin, LOW);

  // Track last applied mode/sound to avoid redundant restarts
  SpeakerMode  last_mode  = (SpeakerMode)(-1);
  SpeakerSound last_sound = (SpeakerSound)(-1);

  // Toggle bookkeeping
  TickType_t last_wake = xTaskGetTickCount();
  TickType_t next_toggle = 0;
  bool level = false;

  for (;;) {
    SpeakerMode m = h->mode;
    SpeakerSound s = h->state;

    // Compute desired frequency & duration (same helpers as before)
    uint16_t f = sound_to_freq(s);
    TickType_t dur = sound_to_dur_ticks(s);

    // Handle OFF
    if (m == SPEAKER_OFF) {
      if (last_mode != SPEAKER_OFF) {
        speaker_stop(h->pin);
        level = false; digitalWrite(h->pin, LOW);
      }
      last_mode = SPEAKER_OFF;
      last_sound = s;

      // idle tick
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
      continue;
    }

    // CONTINUOUS: ensure generator is running at correct frequency
    if (m == SPEAKER_CONTINUOUS) {
      if (last_mode != SPEAKER_CONTINUOUS || last_sound != s) {
        if (f) speaker_start(h->pin, f); else speaker_stop(h->pin);
        last_mode = SPEAKER_CONTINUOUS;
        last_sound = s;
        // reset toggle schedule
        next_toggle = xTaskGetTickCount();
        level = false; digitalWrite(h->pin, LOW);
      }

      // Toggle if enabled
      TickType_t now = xTaskGetTickCount();
      if (spk_enabled && spk_half_ticks > 0 && now - next_toggle >= spk_half_ticks) {
        level = !level;
        digitalWrite(h->pin, level ? HIGH : LOW);
        next_toggle += spk_half_ticks;
      }

      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
      continue;
    }

    // ONESHOT: start the square wave, wait duration (while toggling), then stop
    if (m == SPEAKER_ONESHOT) {
      if (f && dur > 0) {
        speaker_start(h->pin, f);
        // schedule toggles for the duration
        TickType_t start = xTaskGetTickCount();
        next_toggle = start;
        level = false; digitalWrite(h->pin, LOW);

        while ((xTaskGetTickCount() - start) < dur) {
          // Toggle if enabled
          TickType_t now = xTaskGetTickCount();
          if (spk_enabled && spk_half_ticks > 0 && now - next_toggle >= spk_half_ticks) {
            level = !level;
            digitalWrite(h->pin, level ? HIGH : LOW);
            next_toggle += spk_half_ticks;
          }
          vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
        }
        speaker_stop(h->pin);
      } else if (f) {
        // Zero duration fallback: short blip (~50ms)
        speaker_start(h->pin, f);
        vTaskDelay(pdMS_TO_TICKS(50));
        speaker_stop(h->pin);
      }

      // Auto-return to OFF after oneshot
      h->mode = SPEAKER_OFF;
      last_mode = SPEAKER_OFF;
      last_sound = s;
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
      continue;
    }

    // Fallback
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
  }
}

SpeakerHandle* init_speaker(uint8_t pin, SpeakerMode initial_mode, SpeakerSound initial_state,
                            UBaseType_t task_prio, uint16_t task_stack_words)
{
  SpeakerHandle *h = (SpeakerHandle*)pvPortMalloc(sizeof(SpeakerHandle));
  if (!h) return NULL;
  memset(h, 0, sizeof(*h));
  h->pin = pin;
  h->mode = initial_mode;
  h->state = initial_state;

  BaseType_t ok = xTaskCreate(SpeakerTask, "Speaker", task_stack_words, h, task_prio, &h->task);
  if (ok != pdPASS) {
    vPortFree(h);
    return NULL;
  }
  return h;
}
