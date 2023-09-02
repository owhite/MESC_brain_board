// this doesnt work that well but did transmit

#define compSerial Serial // teensy to term
#define BTSerial Serial3 // data from MP2 to teensy

#define BAUDRATE 115200

#define EXT_BLINK_PIN 23
#define BT_BLINK_PIN 5

// blinking stuff
boolean blinkOn = false;
boolean blinkFlag = false;
uint32_t blinkDelta = 0;
uint32_t blinkInterval = 200; 
uint32_t recordInterval = 100; 
uint32_t blinkNow;

void setup() {

  compSerial.begin(BAUDRATE);
  BTSerial.begin(BAUDRATE);

  pinMode(EXT_BLINK_PIN, OUTPUT); 
  pinMode(BT_BLINK_PIN, OUTPUT); 
}

void loop() {
  handleBlink();
  char chr;

  while (compSerial.available()) {
    chr = compSerial.read();
    compSerial.write(chr);
    BTSerial.write(chr);
  }

  while (BTSerial.available()) {
    chr = BTSerial.read();
    compSerial.write(chr);
    BTSerial.write(chr);
  }
}

void handleBlink() {
  blinkNow = millis();
  if ((blinkNow - blinkDelta) > blinkInterval) {
    digitalWrite(EXT_BLINK_PIN, blinkOn);
    digitalWrite(BT_BLINK_PIN, blinkOn);
    blinkOn = !blinkOn;
    blinkDelta = blinkNow;
  }
}
