#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <IntervalTimer.h>
#include <FlexCAN_T4.h>

#define TELEMETRY_WRITE 0
#define SERIAL_WRITE 1


// MESC CAN ID for Iq request
#define CAN_ID_IQREQ  0x001

// IDs
#define TEENSY_NODE_ID  0x03   // sender (this Teensy)
#define ESC_NODE_ID     0x0B 

// ================= Pins from your board =================
#define LED1_PIN        2
#define LED2_PIN        3
#define PUSHBUTTON_PIN  19
#define SPEAKER_PIN     14
#define CS_PIN          10
#define INT_PIN         19
#define TEST_PIN        14

#define IMU_SPI_HS_CLOCK 10000000


// ================= RC PWM (4 channels) =================
#define RC_INPUT1       9
#define RC_INPUT2       8
#define RC_INPUT3       7
#define RC_INPUT4       6

#define CAN_TX          24
#define CAN_RX          25

#endif // MAIN_H
