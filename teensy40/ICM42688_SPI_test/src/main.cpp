#include <Arduino.h>
#include <SPI.h>
#include "ICM42688.h"

// a quick check to make sure the ICM42688 works
// on brain board V1.2 that uses SPI communications

static constexpr uint8_t PIN_CS  = 10;
static constexpr uint8_t PIN_INT = 20;
static constexpr uint32_t IMU_SPI_HS_CLOCK = 10000000;

ICM42688 imu(SPI, PIN_CS, IMU_SPI_HS_CLOCK);

volatile bool imu_drdy = false;

void imuISR() {
  imu_drdy = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_INT, INPUT);

  SPI.begin();

  int ret = imu.begin();
  if (ret < 0) {
    Serial.print("imu.begin() failed: ");
    Serial.println(ret);
    while (1) delay(1000);
  }
  Serial.println("imu.begin() OK");

  ret = imu.enableDataReadyInterrupt();
  if (ret < 0) {
    Serial.print("enableDataReadyInterrupt() failed: ");
    Serial.println(ret);
    while (1) delay(1000);
  }
  Serial.println("DRDY interrupt enabled");

  // Clear any pending latch
  imu.getRawAGT();
  imu_drdy = false;

  attachInterrupt(digitalPinToInterrupt(PIN_INT), imuISR, RISING);

  Serial.println("Interrupt attached; reading on DRDY...");
}

void loop() {
  if (!imu_drdy) return;
  imu_drdy = false;

  int ret = imu.getRawAGT();
  if (ret < 0) {
    Serial.print("getRawAGT() failed: ");
    Serial.println(ret);
    return;
  }

  imu.getAGT();

  /* 
  static uint32_t count = 0;
  static uint32_t t0 = millis();

  count++;
  if (millis() - t0 >= 1000) {
    Serial.print("DRDY Hz = ");
    Serial.println(count);
    count = 0;
    t0 += 1000;
  }
  */ 

  Serial.print("A(g) [");
  Serial.print(imu.accX(), 6); Serial.print(", ");
  Serial.print(imu.accY(), 6); Serial.print(", ");
  Serial.print(imu.accZ(), 6); Serial.print("]  ");

  Serial.print("G(dps) [");
  Serial.print(imu.gyrX(), 6); Serial.print(", ");
  Serial.print(imu.gyrY(), 6); Serial.print(", ");
  Serial.print(imu.gyrZ(), 6); Serial.println("]");
}
