// this doesnt work that well but did transmit

#define compSerial Serial // teensy to term
#define MP2Serial Serial2 // data from MP2 to teensy

#define EXT_BLINK_PIN 23

// blinking stuff
boolean blinkOn = false;
boolean blinkFlag = false;
uint32_t blinkDelta = 0;
uint32_t blinkInterval = 200; 
uint32_t recordInterval = 100; 
uint32_t blinkNow;

void setup() {

  compSerial.begin(115200);
  MP2Serial.begin(115200);

  pinMode(EXT_BLINK_PIN, OUTPUT); 
}

void loop() {
  handleBlink();
  char chr;

  while (compSerial.available()) {
    chr = compSerial.read();
    compSerial.write(chr);
    MP2Serial.write(chr);
  }

  while (MP2Serial.available()) {
    chr = MP2Serial.read();
    compSerial.write(chr);
    MP2Serial.write(chr);
  }


}

void handleBlink() {
  blinkNow = millis();

  if ((blinkNow - blinkDelta) > blinkInterval) {
    digitalWrite(EXT_BLINK_PIN, blinkOn);
    blinkOn = !blinkOn;
    blinkDelta = blinkNow;
  }
  else {
    digitalWrite(EXT_BLINK_PIN, LOW);
  }
}
