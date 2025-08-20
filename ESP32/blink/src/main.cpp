#include <Arduino.h>
#include <WiFi.h>

// program blinks LED and establishes a wifi access point

#define LED_PIN 2 

// Set up SSID and password here
const char* ssid = "ESP32_AP";
const char* password = "12345678";   // must be at least 8 chars

void setup() {
  Serial.begin(115200);
  Serial.println("Configuring access point...");

  // Start WiFi in AP mode
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // LED on
  delay(500);
  digitalWrite(LED_PIN, LOW);   // LED off
  delay(500);
}

