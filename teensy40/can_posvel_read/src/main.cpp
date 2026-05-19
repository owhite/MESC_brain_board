#include <Arduino.h>
#include <FlexCAN_T4.h>

/*
 * POSVEL reader / keepalive diagnostic.
 *
 * Important MESC behavior as tested on the current ESC firmware:
 * - The ESC can stop autonomous CAN_ID_POSVEL publication after about 25.56 s
 *   when no CAN_ID_IQREQ frames are received.
 * - Periodic zero-torque IQREQ frames are sufficient to keep the POSVEL path
 *   synchronized/alive in the current firmware.
 * - The TWR bench wiring uses both Teensy CAN controllers, with node routing
 *   that may place node 11 on CAN2. To avoid silent wrong-bus tests, this
 *   reader listens on CAN1 and CAN2 and sends the zero-IQREQ keepalive on both.
 *
 * This is intentionally a receiver/diagnostic workaround, not the preferred
 * long-term ESC behavior. The ESC should eventually use a wrap-safe monotonic
 * POSVEL scheduler timebase while retaining IQREQ-triggered sync.
 */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

// --- Constants ---
#define CAN_ID_IQREQ 0x001
#define CAN_ID_POSVEL 0x2D0   // MESC POS/VEL ID
#define TEENSY_NODE_ID 0x03
#define ESC_NODE_ID 11
#define LED_PIN 2
#define HEARTBEAT_LED_PIN 13
#define CAN_STB 21
// Match TWR CAN1 routing on Teensy 4.0: RX=pin 23, TX=pin 22.
// FlexCAN_T4 setRX()/setTX() takes a FLEXCAN_PINS enum, not literal GPIO pins.
#define CAN1_PINSEL DEF
// Match TWR CAN2 routing on Teensy 4.0: RX=pin 0, TX=pin 1.
#define CAN2_PINSEL DEF

// --- Ring buffer for safe message passing ---
const int BUF_SIZE = 256;
CAN_message_t rxBuf[BUF_SIZE];
volatile int head = 0, tail = 0;
volatile uint32_t rx_drop_count = 0;

static const uint32_t POSVEL_PRINT_PERIOD_US = 10000;  // 100 Hz total
static const uint32_t SUMMARY_PRINT_PERIOD_US = 1000000;  // 1 Hz
static const uint32_t IQREQ_HEARTBEAT_PERIOD_US = 2000000;  // 0.5 Hz zero torque keepalive (every 2 s)
static const bool IQREQ_HEARTBEAT_ENABLE = true;
static const bool POSVEL_VALUE_PRINT_ENABLE = true;
static const bool POSVEL_TIMER_PRINT_ENABLE = false;
static const bool SUMMARY_PRINT_ENABLE = false;
static const uint32_t HEARTBEAT_TOGGLE_MS = 250;
static volatile uint32_t posvel_active_until_ms = 0;
static uint32_t rx_total_count = 0;
static uint32_t rx_can1_count = 0;
static uint32_t rx_can2_count = 0;
static uint32_t rx_ext_count = 0;
static uint32_t rx_std_count = 0;
static uint32_t posvel_total_count = 0;
static uint32_t posvel_print_count = 0;
static uint32_t posvel_print_skip_count = 0;
static uint32_t last_rx_id = 0;
static uint32_t last_rx_mid = 0;
static uint8_t last_rx_extended = 0;

struct LatestPosVel {
  float pos = 0.0f;
  float vel = 0.0f;
  uint32_t t_us = 0;
  uint32_t dt_us = 0;
  uint32_t count = 0;
  bool valid = false;
};

static LatestPosVel latest_posvel[256];
static uint32_t latest_posvel_printed_count[256] = {0};
static uint32_t latest_posvel_timer_printed_count[256] = {0};
static uint8_t latest_posvel_print_cursor = 0;
static uint8_t latest_posvel_timer_print_cursor = 0;

static uint32_t makeExtId(uint16_t msg_id, uint8_t sender, uint8_t receiver) {
  return ((uint32_t)msg_id << 16) |
         ((uint32_t)receiver << 8) |
         sender;
}

static void packFloat(float val, uint8_t *buf) {
  memcpy(buf, &val, sizeof(float));
}

static const uint32_t LED_ACTIVE_MS = 500;   // blink this long after last RX frame
static const uint32_t LED_TOGGLE_MS = 100;   // 100ms toggle => 5 Hz blink
static volatile uint32_t led_active_until_ms = 0;

static inline void noteCanRxActivity() {
  uint32_t now = millis();
  led_active_until_ms = now + LED_ACTIVE_MS;
}

static inline void notePosVelActivity() {
  uint32_t now = millis();
  posvel_active_until_ms = now + LED_ACTIVE_MS;
}

static inline void serviceRxLed() {
  static uint32_t last_toggle_ms = 0;
  static bool led_state = false;

  uint32_t now = millis();
  bool active = (int32_t)(led_active_until_ms - now) > 0;
  active = (int32_t)(posvel_active_until_ms - now) > 0;

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

static inline void serviceHeartbeatLed() {
  static uint32_t last_toggle_ms = 0;
  static bool led_state = false;
  const uint32_t now = millis();
  if ((uint32_t)(now - last_toggle_ms) < HEARTBEAT_TOGGLE_MS) return;
  last_toggle_ms = now;
  led_state = !led_state;
  digitalWrite(HEARTBEAT_LED_PIN, led_state ? HIGH : LOW);
}

bool bufferPush(const CAN_message_t &msg) {
  int next = (head + 1) % BUF_SIZE;
  if (next == tail) {
    rx_drop_count++;
    return false;
  }
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
  posvel_total_count++;
  notePosVelActivity();

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

    const uint32_t now_us = micros();
    LatestPosVel &latest = latest_posvel[sender];
    latest.pos = pos;
    latest.vel = vel;
    latest.dt_us = latest.valid ? (uint32_t)(now_us - latest.t_us) : 0u;
    latest.t_us = now_us;
    latest.count++;
    latest.valid = true;

    // (kept your placeholder logic)
    if (vel > -20.0f && vel < 20.0f && (vel > 0.1f || vel < -0.1f)) {
    }
  }
}

void printLatestPosVelIfDue() {
  if (!POSVEL_VALUE_PRINT_ENABLE) return;

  static uint32_t last_print_us = 0;
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_print_us) < POSVEL_PRINT_PERIOD_US) return;

  for (uint16_t i = 0; i < 256; ++i) {
    const uint8_t sender = (uint8_t)(latest_posvel_print_cursor + i);
    LatestPosVel &latest = latest_posvel[sender];
    if (!latest.valid || latest.count == latest_posvel_printed_count[sender]) continue;

    char line[96];
    const int len = snprintf(line, sizeof(line),
                             "{\"t_us\":%lu,\"sender\":%u,\"pos\":%.6f,\"vel\":%.6f}\r\n",
                             latest.t_us, sender, latest.pos, latest.vel);
    if (len <= 0 || len >= (int)sizeof(line)) {
      latest_posvel_printed_count[sender] = latest.count;
      posvel_print_skip_count++;
      return;
    }

    Serial.write((const uint8_t*)line, (size_t)len);
    latest_posvel_printed_count[sender] = latest.count;
    latest_posvel_print_cursor = sender + 1u;
    last_print_us = now_us;
    posvel_print_count++;
    return;
  }
}

void printLatestPosVelTimerIfDue() {
  if (!POSVEL_TIMER_PRINT_ENABLE) return;

  static uint32_t last_print_us = 0;
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_print_us) < POSVEL_PRINT_PERIOD_US) return;

  for (uint16_t i = 0; i < 256; ++i) {
    const uint8_t sender = (uint8_t)(latest_posvel_timer_print_cursor + i);
    LatestPosVel &latest = latest_posvel[sender];
    if (!latest.valid || latest.count == latest_posvel_timer_printed_count[sender]) continue;

    char line[96];
    const int len = snprintf(line, sizeof(line),
                             "{\"timer\":1,\"t_us\":%lu,\"sender\":%u,\"count\":%lu,\"dt_us\":%lu}\r\n",
                             latest.t_us, sender, latest.count, latest.dt_us);
    if (len <= 0 || len >= (int)sizeof(line)) {
      latest_posvel_timer_printed_count[sender] = latest.count;
      posvel_print_skip_count++;
      return;
    }

    Serial.write((const uint8_t*)line, (size_t)len);
    latest_posvel_timer_printed_count[sender] = latest.count;
    latest_posvel_timer_print_cursor = sender + 1u;
    last_print_us = now_us;
    posvel_print_count++;
    return;
  }
}

void printSummaryIfDue() {
  if (!SUMMARY_PRINT_ENABLE) return;

  static uint32_t last_summary_us = 0;
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_summary_us) < SUMMARY_PRINT_PERIOD_US) return;
  last_summary_us = now_us;

  Serial.printf("{\"summary\":1,\"t_us\":%lu,\"rx_total\":%lu,\"rx_can1\":%lu,\"rx_can2\":%lu,\"rx_ext\":%lu,\"rx_std\":%lu,\"posvel_total\":%lu,\"posvel_printed\":%lu,\"posvel_skipped\":%lu,\"rx_dropped\":%lu,\"last_id\":\"0x%08lX\",\"last_mid\":\"0x%03lX\",\"last_ext\":%u,\"pos11_count\":%lu,\"pos12_count\":%lu,\"pos11\":%.6f,\"vel11\":%.6f,\"pos12\":%.6f,\"vel12\":%.6f}\r\n",
                now_us,
                rx_total_count,
                rx_can1_count,
                rx_can2_count,
                rx_ext_count,
                rx_std_count,
                posvel_total_count,
                posvel_print_count,
                posvel_print_skip_count,
                rx_drop_count,
                last_rx_id,
                last_rx_mid,
                last_rx_extended,
                latest_posvel[11].count,
                latest_posvel[12].count,
                latest_posvel[11].pos,
                latest_posvel[11].vel,
                latest_posvel[12].pos,
                latest_posvel[12].vel);
}

void sendIqreqHeartbeatIfDue() {
  if (!IQREQ_HEARTBEAT_ENABLE) return;

  static uint32_t last_tx_us = 0;
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_tx_us) < IQREQ_HEARTBEAT_PERIOD_US) return;
  last_tx_us = now_us;

  CAN_message_t msg;
  msg.id = makeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, ESC_NODE_ID);
  msg.len = 8;
  msg.flags.extended = 1;
  packFloat(0.0f, msg.buf);
  packFloat(0.0f, msg.buf + 4);
  // Send on both buses so this diagnostic follows TWR's dual-bus bench wiring
  // without needing to know which ESC node is connected to which controller.
  Can1.write(msg);
  Can2.write(msg);
}

void collectCanFrames(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can, uint32_t *bus_count) {
  CAN_message_t msg;
  while (can.read(msg)) {
    noteCanRxActivity();
    rx_total_count++;
    if (bus_count != nullptr) (*bus_count)++;
    if (msg.flags.extended) {
      rx_ext_count++;
    } else {
      rx_std_count++;
    }
    bufferPush(msg);
  }
}

void collectCanFrames(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can, uint32_t *bus_count) {
  CAN_message_t msg;
  while (can.read(msg)) {
    noteCanRxActivity();
    rx_total_count++;
    if (bus_count != nullptr) (*bus_count)++;
    if (msg.flags.extended) {
      rx_ext_count++;
    } else {
      rx_std_count++;
    }
    bufferPush(msg);
  }
}

// --- General CAN handler ---
void canHandler(const CAN_message_t &msg) {
  const bool extended = (msg.flags.extended != 0);
  uint16_t mid = extended ? ((msg.id >> 16) & 0x1FFF) : (msg.id & 0x7FF);
  last_rx_id = msg.id;
  last_rx_mid = mid;
  last_rx_extended = extended ? 1u : 0u;

  if (mid == CAN_ID_POSVEL) {
    handlePosVel(msg);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  pinMode(CAN_STB, OUTPUT);
  digitalWrite(CAN_STB, LOW); 

  // Fixed missing quote in your JSON string
  Serial.println("{\"status\":\"POSVEL reader started\"}\r\n");

  Can1.setRX(CAN1_PINSEL);
  Can1.setTX(CAN1_PINSEL);
  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.enableFIFO();

  Can2.setRX(CAN2_PINSEL);
  Can2.setTX(CAN2_PINSEL);
  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.enableFIFO();
}

void loop() {
  // Service LED every loop (non-blocking).
  // Pin 13 is unconditional heartbeat; pin 2 indicates recent CAN RX.
  serviceHeartbeatLed();
  serviceRxLed();

  // Collector: move frames into buffer
  collectCanFrames(Can1, &rx_can1_count);
  collectCanFrames(Can2, &rx_can2_count);

  // Dispatcher: process buffered frames
  CAN_message_t msg;
  while (bufferPop(msg)) {
    canHandler(msg);
  }

  sendIqreqHeartbeatIfDue();
  printLatestPosVelIfDue();
  printLatestPosVelTimerIfDue();
  printSummaryIfDue();
}
