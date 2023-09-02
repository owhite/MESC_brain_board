#include <SD.h>
#include <SPI.h>

File myFile;


const int chipSelect = BUILTIN_SDCARD;

void setup()
{
 //UNCOMMENT THESE TWO LINES FOR TEENSY AUDIO BOARD:
 //SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
 //SPI.setSCK(14);  // Audio shield has SCK on pin 14

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");


  if (SD.exists("example2.txt")) {
    Serial.println("example2.txt exists.");
  }
  else {
    Serial.println("example2.txt doesn't exist.");
  }

  // open a new file and immediately close it:
  Serial.println("Creating example2.txt...");
  myFile = SD.open("example2.txt", FILE_WRITE);
  myFile.close();

  // Check to see if the file exists: 
  if (SD.exists("example2.txt")) {
    Serial.println("example2.txt exists.");
  }
  else {
    Serial.println("example2.txt doesn't exist.");  
  }

  // delete the file:
  Serial.println("Removing example2.txt...");
  SD.remove("example2.txt");

  if (SD.exists("example2.txt")){ 
    Serial.println("example2.txt exists.");
  }
  else {
    Serial.println("example2.txt doesn't exist.");  
  }
}

void loop()
{
  // nothing happens after setup finishes.
}
