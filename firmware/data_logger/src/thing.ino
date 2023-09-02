#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include "tones.h"
#include "SparkFun_Ublox_Arduino_Library.h" 

#define compSerial Serial // data from computer keyboard to teensy USB
#define MP2Serial Serial2 // data from MP2 to teensy
#define BTSerial  Serial3 // data from ESP32 to teensy UART
#define BAUDRATE 115200

#define COMP 0
#define BT   1
#define MP2  2

// GPS
SFE_UBLOX_GPS myGPS;
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
#define SAVE          's' 
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
#define BT_RECV_LINES      8
#define SHOW_FILES         9

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
char SD_card_name[25];
File dataFile;
const int chipSelect = BUILTIN_SDCARD;

void setup() {
  compSerial.begin(BAUDRATE);
  BTSerial.begin(BAUDRATE);
  MP2Serial.begin(BAUDRATE);

  pinMode(EXT_BLINK_PIN, OUTPUT); 
  pinMode(GPS_BLINK_PIN, OUTPUT); 

  if (!SD.begin(chipSelect)) {
    compSerial.println("Card failed, or not present");
  }
  compSerial.println("SD card initialized");

  Wire.begin();

  if (myGPS.begin() == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  clrSerialString(COMP);
  clrSerialString(BT);
}

void loop() {
  handleBlink();
  handleGPS();

  if (recvSerialData(compSerial, COMP) != 0) {
    processCommand(COMP); 
  }

  if (recvSerialData(BTSerial, BT) != 0) {
    processCommand(BT); 
  }

  switch (state) {
  case CHECK_MPU:
    {
      compSerial.print("pitch :: ");
      compSerial.print(" :: roll :: ");
      BTSerial.print("pitch :: ");
      BTSerial.print(" :: roll :: ");
      delay(100);
    }
    break;
  case CHECK_GPS:
    {
      state = IDLE;
    }
    break;
  case BT_RECV_LINES: // basically gets output until timeout
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
      MP2Serial.write("status stop\r\n"); // send this just in case 
      MP2Serial.write("get\r\n");
      compSerial.println("INIT_RECORD :: ");
      state = RECORD_GET_RESULTS;

      // remove and then write
      if (SD.exists(SD_card_name)) { SD.remove(SD_card_name); }
      dataFile = SD.open(SD_card_name, FILE_WRITE);
      lineCount = 0;
      recordTime = millis();
      blinkFlag = true;

      if (!dataFile) {
	compSerial.print("error opening: "); compSerial.print(dataFile);
	BTSerial.print("error opening: "); BTSerial.print((char) dataFile);
      }
      else {
	compSerial.println("SD :: {\nSD :: \"blob\": \"");
	dataFile.println("{\n\"blob\": \"");
      }
    }
    break;
  case RECORD_GET_RESULTS:
    {
      uint8_t r = recvSerialData(MP2Serial, MP2);
      if (r == 1) {
	compSerial.print("SD :: ");
	compSerial.println(receivedChars[MP2]);
	if (dataFile) { dataFile.println(receivedChars[MP2]); }  // SD write
	clrSerialString(MP2);
      }
      if (r == 2) {
	startTone(); // this is blocking! 
	compSerial.println("TIMEOUT");

	compSerial.println("SD :: \",");
	compSerial.println("SD :: \"data\": [");
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
	  // addFloatElementToJSON(receivedChars[MP2], "pitch", pitch);
	  // the roll is useful for current mount of logger to bike
	  // addFloatElementToJSON(receivedChars[MP2], "angle", roll);
	  // addIntElementToJSON(receivedChars[MP2], "time", millis() - recordTime);

	  // addFloatElementToJSON(receivedChars[MP2], "lat", latitude_mdeg / 1000000.);
	  // addFloatElementToJSON(receivedChars[MP2], "long", longitude_mdeg / 1000000.);

	  // write the old string
	  strcpy(oldStr, receivedChars[MP2]);
	  compSerial.print("SD :: ");
	  compSerial.print(oldStr);
	  compSerial.println(",");

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
      }
    }
    break;
  case STOP_RECORD:
    {
      compSerial.println("process :: status stop");
      MP2Serial.write("status stop\r\n");

      dataFile.print(oldStr);
      dataFile.println("\n]\n}");
      compSerial.print("SD :: ");
      compSerial.print(oldStr);
      compSerial.println("\n]\nSD :: }");


      compSerial.print("...STOP_RECORD");
      BTSerial.println("...STOP_RECORD");
      if (dataFile) { dataFile.close(); }
      // dataFile.close();
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

void processCommand(int serNum) {

  char str2[maxReceiveLength];  // seriously, I write bad code
  strcpy(str2, receivedChars[serNum]);
  int l = strlen(str2);
  str2[l] = '\n';
  str2[l + 1] = '\0';

  char commandString[maxReceiveLength]; 
  commandString[0] = '\0';

  if (receivedChars[serNum][1] != ' ' && strlen(receivedChars[serNum]) > 2) {
    compSerial.print("commands must start with single letter :: ");
    compSerial.print(serNum);
    compSerial.print(" :: ");
    compSerial.print(receivedChars[serNum]);
    compSerial.print(" :: ");
    compSerial.println(strlen(receivedChars[serNum]));

    clrSerialString(serNum);
    return;
  }

  char cmd = receivedChars[serNum][0];
  compSerial.print("CMD :: "); compSerial.println(cmd);

  if (strlen(receivedChars[serNum]) > 2) {
    strncpy(commandString, receivedChars[serNum] + 2, strlen(receivedChars[serNum]));
    compSerial.print("STR :: ");
    compSerial.println(commandString);
  }
  
  switch (cmd) {
  case BLINK:
    compSerial.println("process :: BLINK!");
    BTSerial.println("...blink");
    // BTSerial.println("...blink");
    blinkFlag = !blinkFlag;
    state = IDLE;
    break;
  case TONE:
    compSerial.println("process :: tone");
    BTSerial.println("...tone");
    startTone();
    state = IDLE;
    break;
  case RECORD:
    compSerial.println("process:: RECORD!");
    BTSerial.println("process:: RECORD!");
    SD_card_name[0] = '\0';
    if (strlen(commandString) > 0 && strlen(commandString) < 20) {
      strcpy(SD_card_name, commandString);
      strcat(SD_card_name, ".txt");
      compSerial.print("name :: ");
      compSerial.println(SD_card_name);
      BTSerial.print("name :: ");
      BTSerial.println(SD_card_name);
    }
    else {
      strcpy(SD_card_name, "default.txt");
      compSerial.print("name :: ");
      compSerial.println(SD_card_name);
      BTSerial.print("name :: ");
      BTSerial.println(SD_card_name);
    }
    BTSerial.println("....");
    state = INIT_RECORD;
    break;
  case GET:
    compSerial.println("process :: get");
    MP2Serial.write("get\r\n");
    state = BT_RECV_LINES;
    break;
  case FILES:
    compSerial.println("process :: showfiles");
    state = SHOW_FILES;
    break;
  case SAVE:
    compSerial.println("process :: save");
    MP2Serial.write("save\r\n");
    state = BT_RECV_LINES;
    break;
  case MPU:
    compSerial.println("process :: showing MPU");
    state = CHECK_MPU;
    compSerial.println("Level the bike");
    BTSerial.println("Level the bike");
    delay(400);
    break;
  case GPS:
    compSerial.println("process :: showing GPS");
    state = CHECK_GPS;
    compSerial.println("reading the GPS");
    BTSerial.println("reading the GPS");
    delay(400);
    break;
  case COMMAND:
    if (strlen(commandString) > 0) {
      compSerial.println("sending:");
      BTSerial.println("sending a command");
      compSerial.println(commandString);
      MP2Serial.write(commandString);
      MP2Serial.write("\r\n");
      state = BT_RECV_LINES;
    }
    else {
      compSerial.println("Please send command");
      BTSerial.println("Please send command");
      state = IDLE;
    }
    break;
  case END:
    compSerial.println(state);
    if (state == RECORD_JSON || state == RECORD_GET_RESULTS) {
      state = STOP_RECORD;
    }
    else {
      state = IDLE;
    }
    break;
  case HELP:
    compSerial.println("BLINK         b: blink LED on logger");
    compSerial.println("TONE          t: emit tone on logger");
    compSerial.println("RECORD        r: start recording");
    compSerial.println("END           e: stop recording ");
    compSerial.println("GET           g: send get to MP2");
    compSerial.println("COMMAND       c: send command to MP2 \"c status json\"");
    compSerial.println("SAVE          s: send save to MP2");
    compSerial.println("FILES         f: broken: see files on SD card");
    compSerial.println("HELP          h: this help message");

    BTSerial.println("BLINK         b: blink LED on logger");
    BTSerial.println("TONE          t: emit tone on logger");
    BTSerial.println("RECORD        r: start recording");
    BTSerial.println("END           e: stop recording ");
    BTSerial.println("GET           g: send get to MP2");
    BTSerial.println("COMMAND       c: send command to MP2 \"c status json\"");
    BTSerial.println("SAVE          s: send save to MP2");
    BTSerial.println("FILES         f: broken: see files on SD card");
    BTSerial.println("HELP          h: this help message");
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

  sprintf(buffer, ",\"%s\":%.6f}", key, value);  

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

void handleGPS() {
  GPS_checkNow = millis();
  if ((GPS_checkNow - GPS_checkDelta) < GPS_checkInterval) {
    return;
  }
  GPS_checkDelta = GPS_checkNow;

  long latitude = myGPS.getLatitude();
  // Serial.print(F("Lat: "));
  // Serial.print(latitude / 10000000., 6);

  long longitude = myGPS.getLongitude();
  // Serial.print(F(" Long: "));
  // Serial.print(longitude / 10000000., 6);

  int SIV = myGPS.getSIV();
  // Serial.print(F(" SIV: "));
  // Serial.print(SIV);

  int day = myGPS.getDay();
  // Serial.print(F(" day: "));
  // Serial.print(day);

  int sec = myGPS.getSecond();
  // Serial.print(F(" second: "));
  // Serial.print(sec);

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

  // Serial.print(F(" BF: "));
  // Serial.print(GPS_blinkFactor);

  Serial.println();
}
