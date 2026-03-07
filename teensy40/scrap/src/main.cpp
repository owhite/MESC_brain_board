
 

#include <Arduino.h>

void setup() {

    Serial.begin(115200);      // USB serial (computer)
    Serial1.begin(115200);     // TX1 = pin 1, RX1 = pin 0

    while (!Serial) {
        ; // wait for USB serial
    }

    Serial.println("Teensy serial bridge ready");
    Serial.println("Type characters and press send");
}

void loop() {

    if (Serial.available()) {

        char c = Serial.read();

        Serial1.write(c);      // send to ESP32
        Serial.write(c);       // echo back to terminal
    }
}
