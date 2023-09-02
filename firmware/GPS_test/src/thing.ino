#include <SD.h>
#include <SPI.h>
#Include <Wire.h> //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h" 

SFE_UBLOX_GPS myGPS;

long lastTime = 0; 

// SD card stuff
char SD_card_name[25];
File dataFile;
const int chipSelect = BUILTIN_SDCARD;

// GPS blink
#define GPS_BLINK_PIN 23

boolean  GPS_blinkOn = true;
boolean  GPS_blinkFlag = true;
uint32_t GPS_blinkFactor = 1;
uint32_t GPS_blinkDelta = 0;
uint32_t GPS_blinkInterval = 300; 
uint32_t GPS_blinkNow;

uint32_t GPS_checkDelta = 0;
uint32_t GPS_checkInterval = 100; 
uint32_t GPS_checkNow;

void setup() {
  Serial.begin(115200);
  Serial.println("Ublox Example");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("SD card initialized");

  Wire.begin();

  if (myGPS.begin() == false) {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setNavigationFrequency(20); 
  myGPS.setI2COutput(COM_TYPE_UBX); 
  myGPS.saveConfiguration(); 
}

void loop() {
  GPS_blinkFactor = 2;
  handleBlink();
  handleGPS();
}

void handleBlink() {
  GPS_blinkNow = millis();

  if (GPS_blinkFlag) {
    if ((GPS_blinkNow - GPS_blinkDelta) > GPS_blinkInterval / GPS_blinkFactor) {
      digitalWrite(GPS_BLINK_PIN, GPS_blinkOn);
      GPS_blinkOn = !GPS_blinkOn;
      GPS_blinkDelta = GPS_blinkNow;
    }
  }
  else {
    digitalWrite(GPS_BLINK_PIN, HIGH);
  }
}

void handleGPS() {
  GPS_checkNow = millis();
  if ((GPS_checkNow - GPS_checkDelta) < GPS_checkInterval) {
    return;
  }
  GPS_checkDelta = GPS_checkNow;

  long latitude = myGPS.getLatitude();
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000., 6);

  long longitude = myGPS.getLongitude();
  Serial.print(F(" Long: "));
  Serial.print(longitude / 10000000., 6);

  int SIV = myGPS.getSIV();
  Serial.print(F(" SIV: "));
  Serial.print(SIV);

  int day = myGPS.getDay();
  Serial.print(F(" day: "));
  Serial.print(day);

  int sec = myGPS.getSecond();
  Serial.print(F(" second: "));
  Serial.print(sec);

  if (SIV == 0) {
    GPS_blinkFlag = false; // just stays on until a satellite locks
  }
  else {
    GPS_blinkFlag = true; 
  }
  GPS_blinkFactor = 5;
  if (SIV < 5) {
    GPS_blinkFactor = SIV;
  }

  Serial.print(F(" BF: "));
  Serial.print(GPS_blinkFactor);

  Serial.println();
}
