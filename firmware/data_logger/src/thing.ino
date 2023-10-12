#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include <tones.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <SparkFunLSM6DS3.h>

// serial stuff
#define compSerial Serial // data from computer keyboard to teensy USB
#define MP2Serial Serial2 // data from MP2 to teensy
#define BTSerial  Serial3 // data from ESP32 to teensy UART
#define BAUDRATE 115200
#define COMP 0
#define BT   1
#define MP2  2
uint8_t cmdState = COMP;

uint8_t serialBuf[1000];
uint8_t chr;

//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) compSerial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) compSerial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif


// GPS STUFF
SFE_UBLOX_GNSS myGNSS;

#define GPS_BLINK_PIN 23

boolean  GPS_blinkOn = true;
boolean  GPS_blinkFlag = true;
uint32_t GPS_blinkFactor = 1;
uint32_t GPS_blinkDelta = 0;
uint32_t GPS_blinkInterval = 300; 
uint32_t GPS_blinkNow;

uint32_t GPS_checkDelta = 0;
uint32_t GPS_checkInterval = 500; 
uint32_t GPS_checkNow;

unsigned long GPSlastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long GPSstartTime = 0; //Used to calc the actual update rate.
unsigned long GPSupdateCount = 0; //Used to calc the actual update rate.

long latitude;
long longitude;
uint16_t SIV;
uint16_t day;
uint16_t sec;
boolean GPS_active;

// IMU
LSM6DS3 myIMU; 
long IMU_x;
long IMU_y;
long IMU_z;

// sound
#define SPK_PIN 9
int melody[] = { NOTE_G5, NOTE_G5, NOTE_C6, NOTE_C6, NOTE_C6, NOTE_G5 };
int noteDurations[] = { 125, 125, 64, 125, 125, 64 };

// single letter commands
#define BLINK         'b'
#define GET           'g'
#define COMMAND       'c'
#define END           'e' // stops record
#define RECORD        'r'
#define TONE          't' // play a sound
#define FILES         'f' // show files
#define MPU           'm'
#define GPS           'p' // p, for planet!
#define SD_CHECK      's' 
#define HELP          'h' 

// states
#define IDLE               0
#define INIT_RECORD        1
#define RECORD_GET_RESULTS 2
#define RECORD_JSON        3
#define STOP_RECORD        4
#define SET_MPU            5
#define CHECK_MPU          6
#define CHECK_GPS          7
#define CHECK_SD_CARD      8
#define BT_RECV_LINES      9
#define SHOW_FILES         10

uint8_t state = IDLE;

// recording stuff
int lineCount = 0;
uint32_t recordTime = 0;

// serial stuff
bool enterStrCapture = true;
bool printFlag = false;
uint32_t start[3];
bool startTimer[3] = {false, false, false};

// string handling
const uint16_t maxReceiveLength = 400;
char receivedChars[4][maxReceiveLength] = {'\0', '\0', '\0', '\0'};
char oldStr[maxReceiveLength] = {};
int  currentIndex[4] = {0, 0, 0, 0};
bool lineWasRecvd[4] = {false, false, false, false};
uint32_t serInterval = 300;  // timeout to receive character from MP2

// blinking stuff
#define EXT_BLINK_PIN 16
#define BT_BLINK_PIN 5
boolean blinkOn = false;
boolean blinkFlag = false;
uint32_t blinkDelta = 0;
uint32_t blinkInterval = 200; 
uint32_t recordInterval = 100; 
uint32_t blinkNow;

// SD card stuff
char SD_file_name[25];
File dataFile;
const int chipSelect = BUILTIN_SDCARD;

elapsedMillis loopTime;

void setup() {
  compSerial.begin(BAUDRATE);
  BTSerial.begin(BAUDRATE);
  MP2Serial.begin(BAUDRATE);

  // crank it up
  MP2Serial.addMemoryForRead(serialBuf, sizeof(serialBuf));

  pinMode(EXT_BLINK_PIN, OUTPUT); 
  pinMode(GPS_BLINK_PIN, OUTPUT); 

  if (!SD.begin(chipSelect)) {
    compSerial.println("Card failed, or not present");
    BTSerial.println("Card failed, or not present");
  }
  else {
    compSerial.println("SD card initialized");
    BTSerial.println("SD card initialized");
  }

  Wire.begin();
  Wire.setClock(400000);

  myIMU.begin();

  if (myGNSS.begin() == false) {
    compSerial.println(F("GPS not detected"));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); 
  myGNSS.setNavigationFrequency(5); 


  clrSerialString(COMP);
  clrSerialString(BT);
}

void loop() {
  loopTime = 0;
  if (state != RECORD_GET_RESULTS) {
    handleBlink();
    handleIMU();
    GPSloop();
  }

  if (recvSerialData(compSerial, COMP) != 0) {
    processCommand(COMP); 
  }

  if (recvSerialData(BTSerial, BT) != 0) {
    processCommand(BT); 
  }
  
  switch (state) {
  case CHECK_MPU:
    {
      printCurrentSerial("Accelerometer: ");
      printCurrentSerial(IMU_x);
      printCurrentSerial(" :: ");
      printCurrentSerial(IMU_y);
      printCurrentSerial(" :: ");
      printCurrentSerial(IMU_z);
      printCurrentSerial("\n");
      delay(100); // run, and block, user has to hit 'e'
    }
    break;
  case CHECK_GPS:
    {
      if (GPS_active) {
	printCurrentSerial("GPS: ");
	printCurrentSerial(latitude);
	printCurrentSerial(" :: ");
	printCurrentSerial(longitude);
	printCurrentSerial(" :: ");
	printCurrentSerial(SIV);
	printCurrentSerial("\n");
      }
      else {
	printCurrentSerial("no satellites\n");
      }
      state = IDLE;
    }
    break;
  case BT_RECV_LINES: // basically gets output until timeout, forgot why it's needed
    {
      uint8_t r = recvSerialData(MP2Serial, MP2);
      if (r == 1) {
	BTSerial.println(receivedChars[MP2]);
	clrSerialString(MP2);
      }
      if (r == 2) {
	BTSerial.println(receivedChars[MP2]);
	clrSerialString(MP2);
	state = IDLE;
      }
    }
    break;
  case INIT_RECORD:
    {
      // remove and then write
      if (SD.exists(SD_file_name)) { SD.remove(SD_file_name); }
      dataFile = SD.open(SD_file_name, FILE_WRITE);

      lineCount = 0;
      recordTime = millis();
      blinkFlag = true;

      if (!dataFile) {
	printCurrentSerial("error opening: "); 
	printCurrentSerial(SD_file_name);
	printCurrentSerial("\n");
	state = IDLE;
      }
      else {
	// note, only sending to serial, not BT, for debugging
	DEBUG_PRINTLN("SD :: {\nSD :: \"blob\": \"");
	dataFile.println("{\n\"blob\": \"");

	startTone(); // this is blocking! 

	MP2Serial.write("status stop\r\n"); // send this just in case 
	MP2Serial.write("get\r\n");
	printCurrentSerial("INIT_RECORD\n");
	clrSerialString(MP2);
	state = RECORD_GET_RESULTS;
      }
    }
    break;
  case RECORD_GET_RESULTS:
    {
      uint8_t r = recvSerialData(MP2Serial, MP2);
      if (r == 1) {
	// same story, only sending to serial, not BT, for debugging
	DEBUG_PRINT("SD :: ");
	DEBUG_PRINTLN(receivedChars[MP2]);
	if (dataFile) { dataFile.println(receivedChars[MP2]); }  // SD write
	clrSerialString(MP2);
      }
      if (r == 2) {
	DEBUG_PRINTLN("TIMEOUT");
	DEBUG_PRINTLN("SD :: \",");
	DEBUG_PRINTLN("SD :: \"data\": [");
	if (dataFile) {
	  dataFile.println("\",");
	  dataFile.println("\"data\": [");
	}

	// note this is the term prompt string, so dump it
	clrSerialString(MP2);
	MP2Serial.write("status json\r\n");
	state = RECORD_JSON;
      }
    }
    break;
  case RECORD_JSON:
    {
      if (recvSerialData(MP2Serial, MP2)) {
	if (strstr(receivedChars[MP2], "status json") != NULL) {
	  clrSerialString(MP2);
	  break;
	}
	char *str = receivedChars[MP2];
	char *ptr = strchr(str, '{');
	int idx = 0;
	if (ptr != NULL) {
	  idx = (int)(ptr - str);
	  strcpy(receivedChars[MP2], str + idx);
	}

	if (strlen(receivedChars[MP2]) > 0) {
	  addFloatElementToJSON(receivedChars[MP2], "IMU_x", IMU_x);
	  addFloatElementToJSON(receivedChars[MP2], "IMU_y", IMU_y);
	  addFloatElementToJSON(receivedChars[MP2], "IMU_z", IMU_z);

	  if (GPS_active) {
	    addFloatElementToJSON(receivedChars[MP2], "lat", latitude/ 10000000.);
	    addFloatElementToJSON(receivedChars[MP2], "long", longitude / 10000000.);
	  }
	  else {
	    addFloatElementToJSON(receivedChars[MP2], "lat", 0.0);
	    addFloatElementToJSON(receivedChars[MP2], "long", 0.0);
	  }

	  // write the old string
	  strcpy(oldStr, receivedChars[MP2]);
	  DEBUG_PRINT("SD :: ");
	  DEBUG_PRINT(oldStr);
	  DEBUG_PRINTLN(",");

	  if (dataFile) { // SD write
	    dataFile.print(oldStr);
	    dataFile.println(",");
	  } 
	  clrSerialString(MP2);
	}

	lineCount++;
	if (lineCount > 10) {
	  BTSerial.println("...");
	  clrSerialString(BT);
	  lineCount = 0;
	}
	compSerial.print("looptime :: ");
	compSerial.println(loopTime);
      }
    }
    break;
  case STOP_RECORD:
    {
      MP2Serial.write("status stop\r\n");
      compSerial.println("process :: status stop");

      dataFile.print(oldStr);
      dataFile.println("\n]\n}");
      DEBUG_PRINT("SD :: ");
      DEBUG_PRINT(oldStr);
      DEBUG_PRINTLN("\n]\nSD :: }");


      printCurrentSerial("...stop record...");
      if (dataFile) {
	dataFile.close();
	printCurrentSerial("...file closed");
      }
      printCurrentSerial("\n");
      stopTone(); // this is blocking! 
      blinkFlag = false;
      state = IDLE;
    }
    break;
  case SHOW_FILES:
    {
      dataFile = SD.open("/");
      printDirectory(dataFile, 0);
      state = IDLE;
    }
    break;
  case IDLE:
    break;
  default:
    break;
  }
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      compSerial.print('\t');
    }
    compSerial.print(entry.name());
    if (entry.isDirectory()) {
      compSerial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      compSerial.print("\t\t");
      compSerial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void startTone() {
  for (int thisNote = 0; thisNote < 3; thisNote++) {
    int noteDuration = noteDurations[thisNote];
    tone(SPK_PIN, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(SPK_PIN);
  }
}

void stopTone() {
  for (int thisNote = 3; thisNote < 6; thisNote++) {
    int noteDuration = noteDurations[thisNote];
    tone(SPK_PIN, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(SPK_PIN);
    delay(10);
  }
}

void clrSerialString(int serNum) {
  currentIndex[serNum] = 0;
  receivedChars[serNum][0] = '\0';
}

void printCurrentSerial (char * msg) {
  if (cmdState == COMP) {
    compSerial.print(msg);
  }
  if (cmdState == BT) {
    BTSerial.print(msg);
  }
}

void processCommand(int serNum) {

  char str2[maxReceiveLength];  // seriously, I write bad code
  strcpy(str2, receivedChars[serNum]);
  int l = strlen(str2);
  str2[l] = '\n';
  str2[l + 1] = '\0';

  char commandString[maxReceiveLength]; 
  commandString[0] = '\0';

  cmdState = serNum; // used by printCurrentSerial to report results

  if (receivedChars[serNum][1] != ' ' && strlen(receivedChars[serNum]) > 2) {
    printCurrentSerial("command did not start with single letter\n");
    clrSerialString(serNum);
    return;
  }

  char cmd = receivedChars[serNum][0];
  printCurrentSerial("CMD :: ");
  printCurrentSerial(receivedChars[serNum]);
  printCurrentSerial("\n");

  if (strlen(receivedChars[serNum]) > 2) {
    strncpy(commandString, receivedChars[serNum] + 2, strlen(receivedChars[serNum]));
    printCurrentSerial("STR :: ");
    printCurrentSerial(commandString);
    printCurrentSerial("\n");
  }
  
  switch (cmd) {
  case BLINK:
    printCurrentSerial("process :: blink\n");
    blinkFlag = !blinkFlag;
    state = IDLE;
    break;
  case TONE:
    printCurrentSerial("process :: tone\n");
    startTone();
    state = IDLE;
    break;
  case RECORD:
    printCurrentSerial("process :: record\n");
    SD_file_name[0] = '\0';
    if (strlen(commandString) > 0 && strlen(commandString) < 20) {
      strcpy(SD_file_name, commandString);
      strcat(SD_file_name, ".json");
    }
    else {
      strcpy(SD_file_name, "default.json");
    }
    printCurrentSerial("name :: ");
    printCurrentSerial(SD_file_name);
    printCurrentSerial("\n");
    state = INIT_RECORD;
    break;
  case GET:
    printCurrentSerial("process :: get\n");
    MP2Serial.write("get\r\n");
    state = BT_RECV_LINES;
    break;
  case FILES:
    printCurrentSerial("process :: showfiles\n");
    state = SHOW_FILES;
    break;
  case SD_CHECK:
    printCurrentSerial("process :: check SD card\n");
    state = CHECK_SD_CARD;
    break;
  case MPU:
    printCurrentSerial("process :: showing MPU\n");
    state = CHECK_MPU;
    delay(400);
    break;
  case GPS:
    printCurrentSerial("process :: showing GPS\n");
    state = CHECK_GPS;
    break;
  case COMMAND:
    if (strlen(commandString) > 0) {
      printCurrentSerial("sending:");
      printCurrentSerial(commandString);
      printCurrentSerial("\n");
      MP2Serial.write(commandString);
      MP2Serial.write("\r\n");
      state = BT_RECV_LINES;
    }
    else {
      printCurrentSerial("Please send command\n");
      state = IDLE;
    }
    break;
  case END:
    if (state == RECORD_JSON || state == RECORD_GET_RESULTS) {
      state = STOP_RECORD;
    }
    else {
      state = IDLE;
    }
    break;
  case HELP:
    printCurrentSerial("BLINK         b: blink LED on logger\n");
    printCurrentSerial("TONE          t: emit tone on logger\n");
    printCurrentSerial("RECORD        r: start recording\n");
    printCurrentSerial("END           e: stop recording \n");
    printCurrentSerial("GET           g: send get to MP2\n");
    printCurrentSerial("COMMAND       c: send command to MP2 \"status json\"");
    printCurrentSerial("SAVE          s: send save to MP2\n");
    printCurrentSerial("FILES         f: broken: see files on SD card\n");
    printCurrentSerial("HELP          h: this help message\n");
    state = IDLE;
    break;
  case IDLE:
    state = IDLE;
    break;
  default:
    break;
  }

  clrSerialString(serNum);
}

bool addData(char nextChar, int serNum) {  
  // Ignore these
  if ((nextChar == '\r') || (nextChar > 255) || (nextChar == 0)) {
    return false;
  }

  if (nextChar == '\n') {
    receivedChars[serNum][currentIndex[serNum]] = '\0';
    return true;
  }

  if (currentIndex[serNum] >= maxReceiveLength - 2) {
    receivedChars[serNum][maxReceiveLength] = '\0';
  }
  else {
    receivedChars[serNum][currentIndex[serNum]] = nextChar;
    receivedChars[serNum][currentIndex[serNum] + 1] = '\0';
    currentIndex[serNum]++;
  }

  return false;
}

uint8_t recvSerialData(Stream &ser, int serNum) {
  char in_char;
  bool dataReady;

  while ( ser.available() > 0 ) {
    start[serNum] = millis(); // used for timeout
    startTimer[serNum] = true;

    in_char= ser.read();
    dataReady = addData(in_char, serNum);  
    if ( dataReady ) {
      return 1;
    }
  }

  // Jens' term creates a prompt with no '/n'. 
  //  so throw in this timeout.
  if (strlen(receivedChars[serNum]) != 0 &&
      millis() - start[serNum] > serInterval &&
      startTimer[serNum]) {
    startTimer[serNum] = false;
    return 2;
  }

  return 0;
}

void addFloatElementToJSON(char *jsonstr, char *key, float value) {
  int len = strlen(jsonstr);

  // bail if this doesnt end with '}'
  if (jsonstr[len - 1] != '}') {
    return;
  }
  // wipe the '}' character
  jsonstr[len - 1] = '\0'; 
  char buffer[40];

  if(value > -0.00000001 && value < 0.00000001) {
    sprintf(buffer, ",\"%s\":0.0}", key);  
  }
  else {
    sprintf(buffer, ",\"%s\":%.4f}", key, value);  
  }

  strcat(jsonstr, buffer);
}

void addIntElementToJSON(char *jsonstr, char *key, int value) {
  int len = strlen(jsonstr);

  // bail if this doesnt end with '}'
  if (jsonstr[len - 1] != '}') {
    return;
  }
  // wipe the '}' character
  jsonstr[len - 1] = '\0'; 
  char buffer[40];

  sprintf(buffer, ",\"%s\":%d}", key, value);  

  strcat(jsonstr, buffer);
}

void handleBlink() {
  blinkNow = millis();
  uint32_t t = blinkInterval;
  if (state == RECORD_GET_RESULTS || state == RECORD_JSON) {
    t = recordInterval;
  }
  if (blinkFlag) {
    if ((blinkNow - blinkDelta) > t) {
      digitalWrite(EXT_BLINK_PIN, blinkOn);
      blinkOn = !blinkOn;
      blinkDelta = blinkNow;
    }
  }
  else {
    digitalWrite(EXT_BLINK_PIN, LOW);
  }

  // GPS LED
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

void handleIMU() {
  IMU_x = myIMU.readFloatAccelX() * 90.0;
  IMU_y = myIMU.readFloatAccelY() * 90.0;
  IMU_z = myIMU.readFloatAccelZ() * 90.0;
}

void GPSloop() {
  //Query module every 25 ms. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available. This is defined
  //by the update freq.

  GPS_active = false;
  if (millis() - GPSlastTime > 200)
  {
    GPSlastTime = millis(); //Update the timer
    
    latitude = myGNSS.getLatitude();
    longitude = myGNSS.getLongitude();
    // latitude /= 10000000.0;
    // longitude /= 10000000.0;
    GPS_active = true;

    GPSupdateCount++;

    //Calculate the actual update rate based on the sketch start time and the 
    //number of updates we've received.
    // Serial.print(F(" Rate: "));
    // Serial.print( GPSupdateCount / ((millis() - GPSstartTime) / 1000.0), 2);
    // Serial.println(F("Hz"));
  }
}
