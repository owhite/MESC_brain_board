#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <IntervalTimer.h>
#include <FlexCAN_T4.h>

// ================= Pins from your board =================
#define LED1_PIN        2
#define LED2_PIN        3
#define PUSHBUTTON_PIN  17
#define SPEAKER_PIN     13

// ================= RC PWM (4 channels) =================
#define RC_CH_COUNT     4
#define RC_INPUT1       9
#define RC_INPUT2       8
#define RC_INPUT3       7
#define RC_INPUT4       6

#define CAN_TX          24
#define CAN_RX          25

// ================= Control loop timing =================
#ifndef CONTROL_HZ
#define CONTROL_HZ      1000u                 // 1 kHz control loop
#endif
#define CONTROL_PERIOD_US (1000000u / CONTROL_HZ)

// ================= CAN config =================
// NOTE: Verify which CAN controller maps to your 24/25 pins on Teensy 4.0.
// Many shields use CAN2 for that pair. Change CAN_CONTROLLER to CAN1/CAN3 if needed.
#define CAN_CONTROLLER  CAN2
#ifndef CAN_BITRATE
#define CAN_BITRATE     1000000u              // 1 Mbit/s CAN 2.0
#endif

// ================= Types =================
typedef struct {
  uint32_t id;
  uint8_t  len;
  uint8_t  buf[8];
  uint32_t t_us;     // timestamp when captured
} CanFrame;

// Your control math (called at CONTROL_HZ, outside the ISR)
void control_step(uint32_t now_us);

#endif // MAIN_H
