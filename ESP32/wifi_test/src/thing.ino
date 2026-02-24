#include <WiFi.h>
#include <ESPmDNS.h>

// ======== User config ========
static const char* WIFI_SSID = "Love Factory";
static const char* WIFI_PASS = "ILoveLyra";

static const char* MDNS_NAME = "twr-repeater";   // => twr-repeater.local
static const uint16_t TCP_PORT = 9000;

// UART to Teensy (ESP32 side). Use UART2 by default.
static const int UART_NUM = 2;
static const int UART_RX_PIN = 16;               // ESP32 RX  (connect to Teensy TX1)
static const int UART_TX_PIN = 17;               // ESP32 TX  (connect to Teensy RX1)
static const uint32_t UART_BAUD = 115200;

// Buffer sizing
static const size_t BUF_SZ = 4096;

// While proving the chain, use test-drop mode (never stall UART reads).
// For true lossless forwarding under WiFi stalls, set this true AND consider adding a larger ring buffer.
static const bool LOSSLESS_MODE = false;

// Pending buffer for UART->TCP partial writes (bounded, single-chunk)
static uint8_t tcpPending[BUF_SZ];
static size_t  tcpPendingLen = 0;
static size_t  tcpPendingOff = 0;

// Shared IO buffers (avoid big stack allocations)
static uint8_t uartBuf[BUF_SZ];
static uint8_t tcpBuf[BUF_SZ];

// Debug serial (USB) baud
static const uint32_t DEBUG_BAUD = 115200;

// LED / heartbeat
static const int LED_PIN = 2;                    // GPIO2 is common onboard LED
static const uint32_t HEARTBEAT_MS = 250;

// LED RX-activity timing (when receiving data from Teensy over UART)
static const uint32_t RX_LED_HOLD_MS = 40;       // LED stays ON this long after last RX byte
// ============================

WiFiServer server(TCP_PORT);
WiFiClient client;
HardwareSerial TeensyUart(UART_NUM);

uint32_t lastBlinkMs = 0;
bool ledState = false;

// If millis() < rxLedUntilMs, force LED ON to indicate UART RX activity
uint32_t rxLedUntilMs = 0;

// ---- Debug counters ----
volatile uint32_t uartRxBytes = 0;
volatile uint32_t uartTxBytes = 0;
volatile uint32_t tcpRxBytes  = 0;
volatile uint32_t tcpTxBytes  = 0;

uint32_t lastStatsMs = 0;
uint32_t bootMs = 0;
// ------------------------

static void wifiConnect()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting to: ");
  Serial.println(WIFI_SSID);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    if (millis() - start > 15000) {
      Serial.println("\nWiFi connect timeout; retrying...");
      WiFi.disconnect(true);
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start = millis();
    }
  }

  Serial.println("\nWiFi connected.");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

static void mdnsStart()
{
  MDNS.end();

  if (!MDNS.begin(MDNS_NAME)) {
    Serial.println("mDNS start failed (continuing without it).");
    return;
  }
  MDNS.addService("twr", "tcp", TCP_PORT);

  Serial.print("mDNS: ");
  Serial.print(MDNS_NAME);
  Serial.println(".local");
}

static void ledTick()
{
  uint32_t now = millis();

  if (now < rxLedUntilMs) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  if (now - lastBlinkMs >= HEARTBEAT_MS) {
    lastBlinkMs = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}

static void acceptClientIfNeeded()
{
  if (client && client.connected()) return;

  if (client) {
    client.stop();
  }

  WiFiClient newClient = server.available();
  if (newClient) {
    client = newClient;

    // For bulk dumps, leaving Nagle enabled is usually fine (less overhead).
    // If you want lowest latency for interactive commands, set true.
    client.setNoDelay(false);

    // Reset pending state on new connection
    tcpPendingLen = 0;
    tcpPendingOff = 0;

    Serial.print("TCP client connected from: ");
    Serial.println(client.remoteIP());
  }
}

static void pumpTcpToUart()
{
  if (!client || !client.connected()) return;

  int avail = client.available();
  if (avail <= 0) return;

  int toRead = (avail > (int)BUF_SZ) ? (int)BUF_SZ : avail;
  int n = client.read(tcpBuf, toRead);
  if (n > 0) {
    tcpRxBytes += (uint32_t)n;
    TeensyUart.write(tcpBuf, n);
    uartTxBytes += (uint32_t)n;
  }
}

static void flushPendingToTcp()
{
  if (!(client && client.connected())) return;
  if (tcpPendingLen == 0) return;

  // Bound work per loop() so we don't starve other tasks
  const int maxIters = 4;
  int iters = 0;

  while (tcpPendingOff < tcpPendingLen && iters++ < maxIters) {
    size_t remaining = tcpPendingLen - tcpPendingOff;

    // Try to write; do NOT rely on availableForWrite() (it can be 0 even when writes succeed).
    // Cap chunk size to keep packets reasonable.
    size_t chunk = remaining;
    if (chunk > 2048) chunk = 2048;

    int written = client.write(tcpPending + tcpPendingOff, chunk);
    if (written <= 0) {
      // Can't write right now (backpressure). Try again next loop.
      break;
    }

    tcpPendingOff += (size_t)written;
    tcpTxBytes += (uint32_t)written;
  }

  if (tcpPendingOff >= tcpPendingLen) {
    tcpPendingLen = 0;
    tcpPendingOff = 0;
  }
}

static void pumpUartToTcp()
{
  // Flush any remainder first
  flushPendingToTcp();

  // In lossless mode, pause UART reads while pending exists (prevents overflow of tcpPending).
  // In test mode, continue draining UART and drop if TCP is stalled.
  if (LOSSLESS_MODE && tcpPendingLen != 0) {
    return;
  }

  int avail = TeensyUart.available();
  if (avail <= 0) return;

  int toRead = (avail > (int)BUF_SZ) ? (int)BUF_SZ : avail;

  // Non-blocking read
  int n = TeensyUart.read(uartBuf, toRead);
  if (n <= 0) return;

  uartRxBytes += (uint32_t)n;
  rxLedUntilMs = millis() + RX_LED_HOLD_MS;

  // Debug: show first few bytes once
  static bool showedSample = false;
  if (!showedSample) {
    showedSample = true;
    Serial.print("UART RX sample (first 16 bytes): ");
    int m = (n < 16) ? n : 16;
    for (int i = 0; i < m; i++) {
      if (uartBuf[i] < 16) Serial.print("0");
      Serial.print(uartBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // If no TCP client, drop bytes (counters prove receipt)
  if (!(client && client.connected())) {
    return;
  }

  // Attempt to write immediately; do NOT trust availableForWrite() as a gate.
  // Cap the immediate write size to avoid huge bursts.
  int toSend = n;
  if (toSend > 2048) toSend = 2048;

  int written = client.write(uartBuf, toSend);
  if (written > 0) {
    tcpTxBytes += (uint32_t)written;
  } else {
    written = 0; // treat as no write
  }

  int consumed = written;
  int remaining = n - consumed;

  if (remaining > 0) {
    if (LOSSLESS_MODE) {
      // Stash as much as we can of the remainder (bounded by BUF_SZ)
      int stash = remaining;
      if (stash > (int)BUF_SZ) stash = (int)BUF_SZ;
      memcpy(tcpPending, uartBuf + consumed, (size_t)stash);
      tcpPendingLen = (size_t)stash;
      tcpPendingOff = 0;

      // If remainder > BUF_SZ, extra is dropped (bounded buffer).
    } else {
      // test mode: drop remainder
    }
  }

  // If we only sent up to 2048 and there is still data in uartBuf, we intentionally drop it
  // in test mode, or stash part in lossless mode. Teensy will resend on the next dump anyway.
}

static void statsTick()
{
  uint32_t now = millis();
  if (now - lastStatsMs < 1000) return;
  lastStatsMs = now;

  // availableForWrite() is still useful as a diagnostic even if we don't trust it for gating
  int afw = (client && client.connected()) ? client.availableForWrite() : -1;
  size_t pend = (tcpPendingLen >= tcpPendingOff) ? (tcpPendingLen - tcpPendingOff) : 0;

  Serial.print("[STATS] up=");
  Serial.print((now - bootMs) / 1000);
  Serial.print("s  WiFi=");
  Serial.print((WiFi.status() == WL_CONNECTED) ? "OK" : "DOWN");
  Serial.print("  TCP=");
  Serial.print((client && client.connected()) ? "CONNECTED" : "none");
  Serial.print("  AFW=");
  Serial.print(afw);
  Serial.print("  PEND=");
  Serial.print((unsigned)pend);
  Serial.print("B  MODE=");
  Serial.print(LOSSLESS_MODE ? "lossless" : "testdrop");

  Serial.print("  UART_RX=");
  Serial.print(uartRxBytes);
  Serial.print("B  UART_TX=");
  Serial.print(uartTxBytes);
  Serial.print("B  TCP_RX=");
  Serial.print(tcpRxBytes);
  Serial.print("B  TCP_TX=");
  Serial.print(tcpTxBytes);
  Serial.println("B");
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(DEBUG_BAUD);
  delay(300);

  bootMs = millis();
  Serial.println();
  Serial.println("=== ESP32 TWR repeater boot ===");
  Serial.printf("Build: %s %s\n", __DATE__, __TIME__);
  Serial.printf("UART2 pins: RX=%d TX=%d baud=%lu\n", UART_RX_PIN, UART_TX_PIN, (unsigned long)UART_BAUD);
  Serial.printf("WiFi SSID: %s\n", WIFI_SSID);
  Serial.printf("TCP port: %u  mDNS: %s.local\n", TCP_PORT, MDNS_NAME);
  Serial.printf("BUF_SZ: %u  MODE: %s\n", (unsigned)BUF_SZ, LOSSLESS_MODE ? "lossless" : "testdrop");

  TeensyUart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  TeensyUart.setRxBufferSize(8 * 1024);
  Serial.println("TeensyUart started.");

  wifiConnect();
  mdnsStart();

  server.begin();
  Serial.print("TCP server listening on port ");
  Serial.println(TCP_PORT);
}

void loop()
{
  ledTick();
  statsTick();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected; reconnecting...");
    wifiConnect();
    mdnsStart();
    server.begin();
  }

  acceptClientIfNeeded();

  pumpTcpToUart();
  pumpUartToTcp();

  delay(1);
}
