#include <Arduino.h>

#define TEST_INTERVAL_MS 1000

// A simple binary test packet
struct TestPacket {
  uint32_t counter;
  uint32_t millisStamp;
  uint8_t  payload[16];
};

uint32_t lastSend = 0;
uint32_t txCounter = 0;

void sendTestPacket()
{
  TestPacket pkt;

  pkt.counter = txCounter++;
  pkt.millisStamp = millis();

  for (int i = 0; i < 16; i++) {
    pkt.payload[i] = i + pkt.counter;
  }

  Serial1.write((uint8_t*)&pkt, sizeof(pkt));

  Serial.print("Sent packet #");
  Serial.print(pkt.counter);
  Serial.print("  size=");
  Serial.println(sizeof(pkt));
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

  Serial.println("\nTeensy Serial1 <-> ESP32 link test");

  Serial1.begin(115200);
}

void loop()
{
  // Send a packet once per second
  if (millis() - lastSend >= TEST_INTERVAL_MS) {
    lastSend = millis();
    sendTestPacket();
  }

  // Check for data coming back from ESP32
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    Serial.print("RX byte from ESP32: 0x");
    if (b < 16) Serial.print("0");
    Serial.println(b, HEX);
  }
}
