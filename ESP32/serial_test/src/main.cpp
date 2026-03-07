#include <Arduino.h>


#define RXD2 16
#define TXD2 17
#define LED_PIN 2

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    Serial.println("ESP32 serial monitor started");
}

void loop() {

    if (Serial2.available()) {

        // Read incoming byte
        char c = Serial2.read();

        // Blink LED
        digitalWrite(LED_PIN, HIGH);
        delay(20);
        digitalWrite(LED_PIN, LOW);

        // Optional: print to USB for debugging
        Serial.print(c);
    }
}
