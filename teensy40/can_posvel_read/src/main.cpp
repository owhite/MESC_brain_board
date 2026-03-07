#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// --- Constants ---
#define CAN_ID_POSVEL 0x2D0   // MESC POS/VEL ID
#define LED_PIN 2
#define CAN_STB 21

// --- Ring buffer for safe message passing ---
const int BUF_SIZE = 32;
CAN_message_t rxBuf[BUF_SIZE];
volatile int head = 0, tail = 0;

static const uint32_t LED_ACTIVE_MS = 500;   // blink this long after last RX frame
static const uint32_t LED_TOGGLE_MS = 100;   // 100ms toggle => 5 Hz blink
static volatile uint32_t led_active_until_ms = 0;

static inline void noteCanRxActivity() {
  uint32_t now = millis();
  led_active_until_ms = now + LED_ACTIVE_MS;
}

static inline void serviceRxLed() {
  static uint32_t last_toggle_ms = 0;
  static bool led_state = false;

  uint32_t now = millis();
  bool active = (int32_t)(led_active_until_ms - now) > 0;

  if (!active) {
    led_state = false;
    digitalWrite(LED_PIN, LOW);
    return;
  }

  if (now - last_toggle_ms >= LED_TOGGLE_MS) {
    last_toggle_ms = now;
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state ? HIGH : LOW);
  }
}

bool bufferPush(const CAN_message_t &msg) {
  int next = (head + 1) % BUF_SIZE;
  if (next == tail) return false; // buffer full, drop
  rxBuf[head] = msg;
  head = next;
  return true;
}

bool bufferPop(CAN_message_t &msg) {
  if (head == tail) return false; // empty
  msg = rxBuf[tail];
  tail = (tail + 1) % BUF_SIZE;
  return true;
}

// --- POSVEL handler ---
void handlePosVel(const CAN_message_t &msg) {
  uint8_t sender = (msg.id) & 0xFF;

  if (msg.len == 8) {
    float pos, vel;

    uint32_t u0 = (uint32_t)msg.buf[0] |
                  ((uint32_t)msg.buf[1] << 8) |
                  ((uint32_t)msg.buf[2] << 16) |
                  ((uint32_t)msg.buf[3] << 24);

    uint32_t u1 = (uint32_t)msg.buf[4] |
                  ((uint32_t)msg.buf[5] << 8) |
                  ((uint32_t)msg.buf[6] << 16) |
                  ((uint32_t)msg.buf[7] << 24);

    memcpy(&pos, &u0, sizeof(float));
    memcpy(&vel, &u1, sizeof(float));

    Serial.printf("{\"t_us\":%lu,\"sender\":%u,\"pos\":%.6f,\"vel\":%.6f}\r\n",
                  micros(), sender, pos, vel);

    // (kept your placeholder logic)
    if (vel > -20.0f && vel < 20.0f && (vel > 0.1f || vel < -0.1f)) {
    }
  }
}

// --- General CAN handler ---
void canHandler(const CAN_message_t &msg) {
  uint16_t mid = (msg.id >> 16) & 0x1FFF;  // unpack MESC message ID
  if (mid == CAN_ID_POSVEL) {
    handlePosVel(msg);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(CAN_STB, OUTPUT);
  digitalWrite(CAN_STB, LOW); 

  // Fixed missing quote in your JSON string
  Serial.println("{\"status\":\"POSVEL reader started\"}\r\n");

  Can1.begin();
  Can1.setBaudRate(500000);  // or 1000000 if ESC runs at 1 Mbps
  Can1.enableFIFO();
}

void loop() {
  // Service LED every loop (non-blocking)
  serviceRxLed();

  // Collector: move frames into buffer
  CAN_message_t msg;
  while (Can1.read(msg)) {
    noteCanRxActivity();
    bufferPush(msg);
  }

  // Dispatcher: process buffered frames
  while (bufferPop(msg)) {
    canHandler(msg);
  }
}
