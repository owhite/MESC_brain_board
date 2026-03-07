#include <Arduino.h>

#define LED_PIN1 2
#define LED_PIN2 3
#define LED_PIN3 13

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);

}

void loop() {
  Serial.println("UP");

  digitalWriteFast(LED_PIN1, HIGH);
  digitalWriteFast(LED_PIN2, HIGH);
  digitalWriteFast(LED_PIN3, HIGH);
  delay(100);
  digitalWriteFast(LED_PIN1, LOW);
  digitalWriteFast(LED_PIN2, LOW);
  digitalWriteFast(LED_PIN3, LOW);
  delay(100);
}


