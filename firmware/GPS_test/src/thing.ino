#include <Wire.h> //Needed for I2C to GPS
#include <SD.h>
#include <SPI.h>

#include "SparkFun_Ublox_Arduino_Library.h" 
SFE_UBLOX_GPS myGPS;

#include <MicroNMEA.h> 
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// SD card stuff
char SD_card_name[25];
File dataFile;
const int chipSelect = BUILTIN_SDCARD;

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun Ublox Example");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("SD card initialized");

  Wire.begin();

  if (myGPS.begin() == false) {
    Serial.println(F("Ublox GPS"));
    while (1);
  }

}

void loop()
{
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if(nmea.isValid() == true) {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();

    Serial.print("Latitude (deg): ");
    Serial.println(latitude_mdeg / 1000000., 6);
    Serial.print("Longitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);
  }
  else {
    Serial.print("No Fix - ");
    Serial.print("Num. satellites: ");
    Serial.println(nmea.getNumSatellites());
  }

  delay(250); //Don't pound too hard on the I2C bus
}

