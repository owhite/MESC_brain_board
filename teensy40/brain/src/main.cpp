#include <Arduino.h>
#include "main.h"
#include "esc.h"
#include "supervisor.h" 
#include "led_speaker.h"   

#define userSerial Serial

Supervisor_typedef g_supervisor;

// Two ESC instances
ESC_motor_typedef esc1;
ESC_motor_typedef esc2;

LEDHandle* redLed;
LEDHandle* greenLed;
SpeakerHandle* speaker;

void setup() {
  redLed   = init_LED(LED1_PIN, LED_SLOW_BLINK, tskIDLE_PRIORITY + 1, 256);
  greenLed = init_LED(LED2_PIN, LED_FAST_BLINK, tskIDLE_PRIORITY + 1, 256);
  speaker  = init_speaker(SPEAKER_PIN, SPEAKER_OFF, WARNING1, tskIDLE_PRIORITY + 1, 256);

  init_ESC(&esc1, "ESC1", 1);
  init_ESC(&esc2, "ESC2", 2);

  static const char *ESC_NAMES[] = { "LeftESC", "RightESC" };
  static const uint16_t ESC_IDS[] = { 1, 2 };
  const uint16_t ESC_COUNT = 2;

  init_supervisor(&g_supervisor, ESC_COUNT, ESC_NAMES, ESC_IDS);

  vTaskStartScheduler();
}

void loop() {
}


