#include <Arduino.h>
#include <FlexCAN_T4.h>

// === Teensy CAN setup ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

#define PUSHBUTTON_PIN 17
#define LED_PIN 2

// MESC CAN ID for Iq request
#define CAN_ID_IQREQ   0x001
#define CAN_ID_POSVEL  0x2D0   // MESC POS/VEL ID

// IDs
#define TEENSY_NODE_ID  0x03   // sender (this Teensy)
#define ESC_NODE_ID1    0x0B   // receiver (your ESC node_id = 11)
#define ESC_NODE_ID2    0x0C   // receiver (your ESC node_id = 12)

float max_current = 4.0f;

// --- Ring buffer for safe message passing ---
const int BUF_SIZE = 32;
CAN_message_t rxBuf[BUF_SIZE];
volatile int head = 0, tail = 0;

// ===== RX LED "flash while traffic" logic =====
// We want visible flashing even at high CAN rates.
// Strategy:
// - On ANY received frame, arm an "active" window into the future.
// - While active, blink at a human-visible rate (e.g., 5 Hz).
// - Do NOT use delay() for blinking; we service it every loop.

static const uint32_t LED_ACTIVE_MS   = 500;  // keep blinking this long after last RX frame
static const uint32_t LED_TOGGLE_MS   = 100;  // 100ms toggle => 5 Hz blink
static volatile uint32_t led_active_until_ms = 0;

static inline void noteCanRxActivity() {
  uint32_t now = millis();
  led_active_until_ms = now + LED_ACTIVE_MS;
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
  uint8_t receiver = (msg.id) & 0xFF; // receiver ID (per your packing)

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

    /*
    Serial.printf("{\"t_us\":%lu,\"sender\":%u,\"pos\":%.6f,\"vel\":%.6f}\r\n",
                  micros(), receiver, pos, vel);
    */ 
  }
}

// --- General CAN handler ---
void canHandler(const CAN_message_t &msg) {
  uint16_t mid = (msg.id >> 16) & 0x1FFF; // unpack MESC message ID
  if (mid == CAN_ID_POSVEL) {
    handlePosVel(msg);
  }
}

// Build full 29-bit CAN extended ID
uint32_t make_ext_id(uint16_t msg_id, uint8_t sender, uint8_t receiver) {
  return ((uint32_t)msg_id << 16) |
         ((uint32_t)receiver << 8) |
         sender;
}

// Encode float into buffer
void pack_float(float val, uint8_t *buf) {
  memcpy(buf, &val, sizeof(float));
}

static bool running = false; // motion enabled flag

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(PUSHBUTTON_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN, HIGH);

  Serial.println("Press any key to toggle motion START/STOP.");

  delay(2000);

  Can1.begin();
  Can1.setBaudRate(500000);

  // If you only use extended frames, you can optionally add:
  // Can1.setMBFilter(REJECT_ALL);
  // Can1.setMBFilter(ACCEPT_ALL, 0);  // depends on your FlexCAN_T4 usage
}

// #define LOOP1 0
#define LOOP2 1
// #define LOOP3 0

#if LOOP2
void loop() {
  static uint8_t state = 0;              // 0=idle, 1=forward, 2=idle, 3=reverse
  static uint32_t last_switch_ms = 0;
  const uint32_t STATE_HOLD_MS = 400;

  // Non-blocking send pacing (replaces delay(100))
  static uint32_t last_send_ms = 0;
  const uint32_t SEND_PERIOD_MS = 100;

  // Service LED every loop (non-blocking)
  // serviceRxLed();

  // --- Serial toggle control ---
  if (Serial.available()) {
    (void)Serial.read(); // clear the key
    running = !running;
    Serial.printf("Motion %s\r\n", running ? "STARTED" : "STOPPED");
    // no delay() debounce; if you want one, do a simple time-gate instead
  }

  // --- Drain CAN RX FIFO quickly ---
  CAN_message_t msg;
  while (Can1.read(msg)) {
    noteCanRxActivity();  // <-- activity latch (any received frame)
    bufferPush(msg);
  }

  // --- Process buffered frames ---
  while (bufferPop(msg)) {
    canHandler(msg);
  }

  // Rate-limit CAN TX without blocking loop
  uint32_t now_ms = millis();
  if (now_ms - last_send_ms < SEND_PERIOD_MS) {
    return;
  }
  last_send_ms = now_ms;

  if (!running) {
    // --- Send zero torque (stop command) to both ESCs ---
    float cmd = 0.0f;
    for (int i = 0; i < 2; i++) {
      uint16_t esc_id = (i == 0) ? ESC_NODE_ID1 : ESC_NODE_ID2;
      CAN_message_t tx;
      tx.id = make_ext_id(CAN_ID_IQREQ, TEENSY_NODE_ID, esc_id);
      tx.flags.extended = 1;
      tx.len = 8;
      pack_float(cmd, tx.buf);
      float zero = 0.0f;
      pack_float(zero, tx.buf + 4);
      Can1.write(tx);
      /*
      Serial.printf("STOP: Sent Iq_req=%.3f A to ESC node_id=%u (CAN ID=0x%08X)\r\n",
                    cmd, esc_id, tx.id);
      */
    }
    return;
  }

  // --- State machine: advance every STATE_HOLD_MS ---
  if (now_ms - last_switch_ms >= STATE_HOLD_MS) {
    last_switch_ms = now_ms;
    state = (state + 1) % 4;
  }

  // --- Command selection based on state ---
  float cmd = 0.0f;
  if (state == 0)       cmd =  0.0f;
  else if (state == 1)  cmd =  max_current;
  else if (state == 2)  cmd =  0.0f;
  else if (state == 3)  cmd = -max_current;

  // --- Clamp safety range ---
  if (cmd > 10.0f)  cmd = 10.0f;
  if (cmd < -10.0f) cmd = -10.0f;

  // --- Send to both ESCs ---
  for (int i = 0; i < 2; i++) {
    uint16_t esc_id = (i == 0) ? ESC_NODE_ID1 : ESC_NODE_ID2;
    CAN_message_t tx;
    tx.id = make_ext_id(CAN_ID_IQREQ, TEENSY_NODE_ID, esc_id);
    tx.flags.extended = 1;
    tx.len = 8;
    pack_float(cmd, tx.buf);
    float zero = 0.0f;
    pack_float(zero, tx.buf + 4);
    Can1.write(tx);

    /* 
    Serial.printf("State %u: Sent Iq_req=%.3f A to ESC node_id=%u (CAN ID=0x%08X)\r\n",
                  state, cmd, esc_id, tx.id);
    */ 
  }
}
#endif
