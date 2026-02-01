#include "ICM42688.h"

static const uint8_t CS_PIN  = 18;
static const uint8_t INT_PIN = 19;

ICM42688 imu(SPI, CS_PIN);
volatile bool dataReady = false;

// count interrupts per second
volatile uint32_t isrCount = 0;

void setImuFlag() {
  dataReady = true;
  isrCount++;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  int status = imu.begin();
  if (status < 0) {
    Serial.println("IMU init failed");
    Serial.println(status);
    while (1) {}
  }

  pinMode(INT_PIN, INPUT);
  attachInterrupt(INT_PIN, setImuFlag, RISING);

  imu.setAccelODR(ICM42688::odr12_5);
  imu.setGyroODR(ICM42688::odr12_5);
  imu.enableDataReadyInterrupt();

  Serial.println("ax ay az gx gy gz temp | ISR Hz");
}

uint32_t lastPrint = 0;

void loop() {
  // print ISR frequency once per second
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    uint32_t count = isrCount;
    isrCount = 0;

    Serial.print("ISR Frequency: ");
    Serial.print(count);
    Serial.println(" Hz");
  }

  // process IMU read when ready
  if (!dataReady) return;
  dataReady = false;

  imu.getAGT();

  Serial.print(imu.accX(), 6); Serial.print('\t');
  Serial.print(imu.accY(), 6); Serial.print('\t');
  Serial.print(imu.accZ(), 6); Serial.print('\t');
  Serial.print(imu.gyrX(), 6); Serial.print('\t');
  Serial.print(imu.gyrY(), 6); Serial.print('\t');
  Serial.print(imu.gyrZ(), 6); Serial.print('\t');
  Serial.print(imu.temp(), 6); Serial.println('\r');
  // ISR frequency printed separately
}
