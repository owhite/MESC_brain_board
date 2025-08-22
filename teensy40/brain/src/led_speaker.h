#ifndef LED_SPEAKER_H
#define LED_SPEAKER_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>    // pinMode, digitalWrite, tone, noTone
#include <FreeRTOS.h>
#include <task.h>

// ===== LED states =====
typedef enum {
  LED_OFF = 0,
  LED_ON_CONTINUOUS,
  LED_FAST_BLINK,
  LED_SLOW_BLINK
} LEDState;

// ===== Speaker modes and sounds =====
typedef enum {
  SPEAKER_OFF = 0,
  SPEAKER_ONESHOT,
  SPEAKER_CONTINUOUS
} SpeakerMode;

typedef enum {
  WARNING1 = 0,
  WARNING2,
  TRACK_BALANCE
} SpeakerSound;

// ===== Tunables (defaults) =====
#ifndef LED_FAST_PERIOD_MS
#define LED_FAST_PERIOD_MS   200   // total period; 50% duty -> 100ms on, 100ms off
#endif

#ifndef LED_SLOW_PERIOD_MS
#define LED_SLOW_PERIOD_MS  1000   // total period; 50% duty
#endif

#ifndef WARNING1_FREQ_HZ
#define WARNING1_FREQ_HZ    2000
#endif
#ifndef WARNING2_FREQ_HZ
#define WARNING2_FREQ_HZ    3000
#endif
#ifndef TRACK_FREQ_HZ
#define TRACK_FREQ_HZ       1000
#endif

#ifndef WARNING1_DUR_MS
#define WARNING1_DUR_MS      200
#endif
#ifndef WARNING2_DUR_MS
#define WARNING2_DUR_MS      300
#endif
#ifndef TRACK_DUR_MS
#define TRACK_DUR_MS         250   // used only for ONESHOT; CONTINUOUS ignores
#endif

// ===== Handles you can modify directly =====
typedef struct LEDHandle {
  uint8_t  pin;
  volatile LEDState state;     // <-- you change this at runtime
  TaskHandle_t task;
} LEDHandle;

typedef struct SpeakerHandle {
  uint8_t        pin;
  volatile SpeakerMode  mode;   // <-- you change this at runtime
  volatile SpeakerSound state;  // <-- “state” = which sound
  TaskHandle_t   task;
} SpeakerHandle;

// ===== API =====
#ifdef __cplusplus
extern "C" {
#endif

LEDHandle*     init_LED(uint8_t pin, LEDState initial_state,
                        UBaseType_t task_prio, uint16_t task_stack_words);

SpeakerHandle* init_speaker(uint8_t pin, SpeakerMode initial_mode, SpeakerSound initial_state,
                            UBaseType_t task_prio, uint16_t task_stack_words);

#ifdef __cplusplus
}
#endif

#endif // LED_SPEAKER_H
