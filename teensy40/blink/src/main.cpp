#include <Arduino.h>

#define SPEAKER_PIN 13

void setup() {
  Serial.begin(115200);
  // pinMode(SPEAKER_PIN, OUTPUT);
}

void loop() {
  Serial.println("UP");
  // Generate a 1 kHz square wave manually
  digitalWriteFast(SPEAKER_PIN, HIGH);
  delayMicroseconds(500);   // half period (500 µs → 1 kHz)
  digitalWriteFast(SPEAKER_PIN, LOW);
  delayMicroseconds(500);
}


