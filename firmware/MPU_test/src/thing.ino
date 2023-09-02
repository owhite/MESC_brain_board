#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B

int x, y, z;

void setup() {
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  myIMU.begin();
}


void loop()
{

  x = myIMU.readFloatAccelX() * 90;
  y = myIMU.readFloatAccelY() * 90;
  z = myIMU.readFloatAccelZ() * 90;
  //Get all parameters
  Serial.println("\nAccelerometer:");
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);

  delay(1000);
}
