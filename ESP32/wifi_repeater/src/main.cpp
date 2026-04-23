#include <WiFi.h>
#include <ESPmDNS.h>
#include <SerialTransfer.h>
#include <SPI.h>
#include <Wire.h>
#include <stdarg.h>

// ======== User config ========
static const char* WIFI_SSID = "Love Factory";
static const char* WIFI_PASS = "ILoveLyra";

static const char* MDNS_NAME = "twr-repeater";   // => twr-repeater.local
static const uint16_t TCP_PORT = 9000;

// UART to Teensy (ESP32 side). Use UART2 by default.
static const int UART_NUM = 2;
static const int UART_RX_PIN = 16;               // ESP32 RX  (connect to Teensy Serial4 TX / pin 17)
static const int UART_TX_PIN = 17;               // ESP32 TX  (connect to Teensy Serial4 RX / pin 16)
static const uint32_t UART_BAUD = 2000000;
static const bool SERIALTRANSFER_RX_MONITOR_MODE = true;

// Buffer sizing
static const size_t BUF_SZ = 4096;

// While proving the chain, use test-drop mode (never stall UART reads).
static const bool LOSSLESS_MODE = true;

// Debug serial (USB) baud
static const uint32_t DEBUG_BAUD = 115200;

static constexpr uint16_t TELEMETRY_PACKET_FOOTER = 0xDEADu;
static constexpr size_t TELEMETRY_PAYLOAD_FLOATS = 11u;

struct __attribute__((packed)) TelemetryPacket {
  uint32_t seq;
  uint32_t timestamp_us;
  uint16_t type;
  uint16_t status_flags;
  float payload[TELEMETRY_PAYLOAD_FLOATS];
  uint32_t state_bits;
  uint16_t checksum;
  uint16_t footer;
};

static_assert(sizeof(TelemetryPacket) == 64u, "TelemetryPacket must be 64 bytes");

struct __attribute__((packed)) TelemetryAck {
  uint32_t seq;
  uint16_t code;
  uint16_t reserved;
};

static bool lastTcpConnected = false;

// LED / heartbeat
static const int LED_PIN = 2;                    // GPIO2 is common onboard LED
static const uint32_t HEARTBEAT_MS = 250;

// LED RX-activity timing (when receiving data from Teensy over UART)
static const uint32_t RX_LED_HOLD_MS = 40;       // LED stays ON this long after last RX byte
// ============================

// Pending buffer for UART->TCP partial writes (bounded, single-chunk)
static uint8_t tcpPending[BUF_SZ];
static size_t  tcpPendingLen = 0;
static size_t  tcpPendingOff = 0;

// Shared IO buffers (avoid big stack allocations)
static uint8_t uartBuf[BUF_SZ];
static uint8_t tcpBuf[BUF_SZ];

WiFiServer server(TCP_PORT);
WiFiClient client;
HardwareSerial TeensyUart(UART_NUM);
SerialTransfer gTransfer;
static bool g_have_last_seq = false;
static uint32_t g_last_seq = 0u;
static uint32_t g_drop_estimate = 0u;
static uint32_t g_rx_packet_count = 0u;
static uint32_t g_tcp_forward_drop_packets = 0u;
static uint32_t g_ack_tx_count = 0u;
static uint32_t g_last_rx_timestamp_us = 0u;
static uint32_t g_last_rx_ms = 0u;
static float g_last_bus_voltage = NAN;
static int8_t g_last_transfer_status = 0;
static void flushPendingToTcp();

static bool enqueueTcpPending(const uint8_t *data, size_t len)
{
  if (data == nullptr || len == 0u) return true;
  if (len > BUF_SZ) return false;

  if (tcpPendingOff > 0u && tcpPendingOff < tcpPendingLen) {
    const size_t remain = tcpPendingLen - tcpPendingOff;
    memmove(tcpPending, tcpPending + tcpPendingOff, remain);
    tcpPendingLen = remain;
    tcpPendingOff = 0u;
  } else if (tcpPendingOff >= tcpPendingLen) {
    tcpPendingLen = 0u;
    tcpPendingOff = 0u;
  }

  if ((tcpPendingLen + len) > BUF_SZ) return false;
  memcpy(tcpPending + tcpPendingLen, data, len);
  tcpPendingLen += len;
  return true;
}

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

// ============================
// ANSI "HUD" layout settings
// Row 1: banner
// Row 2..(1+STATS_LINES): stats
// Below that: scrolling logs
// ============================
static const int BANNER_LINES = 1;
static const int STATS_LINES  = 10;                 // number of stats rows we draw
static const int HUD_LINES    = BANNER_LINES + STATS_LINES; // total reserved at top

// ANSI helpers (save/restore cursor)
static inline void ansiSaveCursor()   { Serial.print("\033[s"); }
static inline void ansiRestoreCursor(){ Serial.print("\033[u"); }

// Move cursor to 1-based row/col
static inline void ansiGotoRC(int row, int col)
{
  Serial.printf("\033[%d;%dH", row, col);
}

// Clear current line (to end)
static inline void ansiClearLine()
{
  Serial.print("\033[K");
}

// Set scrolling region (top..bottom). Many terminals accept large bottom.
static inline void ansiSetScrollRegion(int top, int bottom)
{
  Serial.printf("\033[%d;%dr", top, bottom);
}

// Reset scrolling region to full screen
static inline void ansiResetScrollRegion()
{
  Serial.print("\033[r");
}

// Clear screen + home
static inline void ansiClearScreen()
{
  Serial.print("\033[2J\033[H");
}

// --- Banner printing (top line, in place) ---
static void bannerPrintf(const char* fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  ansiSaveCursor();
  ansiGotoRC(1, 1);         // row 1 banner
  ansiClearLine();
  Serial.print(buf);
  // Pad a little to fully overwrite shorter text even if clear doesn't work perfectly
  Serial.print("          ");
  ansiRestoreCursor();
}

// --- Optional log printing (goes to scrolling region below HUD) ---
static void logPrintf(const char* fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.print(buf);
}

// Initialize HUD: print placeholders + set scroll region so logs don't overwrite HUD
static void hudInit()
{
  ansiClearScreen();

  // Print banner placeholder on row 1
  ansiGotoRC(1, 1);
  Serial.print("Banner: (booting...)");
  ansiClearLine();

  // Print STATS_LINES blank lines starting on row 2
  ansiGotoRC(2, 1);
  for (int i = 0; i < STATS_LINES; i++) {
    Serial.println(); // creates the block height
  }

  // Set scroll region so that only rows below HUD scroll
  // Use a large bottom (999) as a practical "end of screen" for most terminals.
  ansiSetScrollRegion(HUD_LINES + 1, 999);

  // Move cursor to start of log area
  ansiGotoRC(HUD_LINES + 1, 1);
}

// ============================

static void wifiConnect()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  bannerPrintf("WiFi connecting to: %s", WIFI_SSID);

  uint32_t start = millis();
  int dots = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    dots = (dots + 1) % 30;

    // Update banner with dots (in place)
    char dotbuf[40];
    int n = (dots < (int)sizeof(dotbuf)-1) ? dots : (int)sizeof(dotbuf)-1;
    for (int i = 0; i < n; i++) dotbuf[i] = '.';
    dotbuf[n] = 0;

    bannerPrintf("WiFi connecting to: %s %s", WIFI_SSID, dotbuf);

    if (millis() - start > 15000) {
      bannerPrintf("WiFi connect timeout; retrying...");
      WiFi.disconnect(true);
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start = millis();
      dots = 0;
    }
  }

  bannerPrintf("WiFi connected. IP: %s", WiFi.localIP().toString().c_str());
}

static void serialTransferRxTick()
{
  flushPendingToTcp();

  // Drain multiple packets per loop pass to avoid UART/parser backlog.
  static constexpr int MAX_PACKETS_PER_TICK = 32;
  for (int i = 0; i < MAX_PACKETS_PER_TICK; ++i) {
    if (!gTransfer.available()) {
      break;
    }

    TelemetryPacket pkt = {};
    uint16_t rec_size = 0u;
    rec_size = gTransfer.rxObj(pkt, rec_size);
    if (rec_size != sizeof(TelemetryPacket)) {
      continue;
    }

    if (pkt.footer != TELEMETRY_PACKET_FOOTER) {
      continue;
    }

    g_rx_packet_count++;
    if (g_have_last_seq && pkt.seq > (g_last_seq + 1u)) {
      g_drop_estimate += (pkt.seq - g_last_seq - 1u);
    }
    g_have_last_seq = true;
    g_last_seq = pkt.seq;
    g_last_rx_timestamp_us = pkt.timestamp_us;
    g_last_rx_ms = millis();

    // Stop-and-wait flow control: ACK immediately after ingest/validation.
    TelemetryAck ack = {};
    ack.seq = pkt.seq;
    ack.code = 1u;
    gTransfer.sendDatum(ack);
    g_ack_tx_count++;

    const float bus_voltage = pkt.payload[0];
    g_last_bus_voltage = bus_voltage;
    if (client && client.connected()) {
      const uint8_t *raw = reinterpret_cast<const uint8_t*>(&pkt);
      const size_t pkt_size = sizeof(TelemetryPacket);

      if (tcpPendingLen != 0u) {
        if (!enqueueTcpPending(raw, pkt_size)) {
          g_tcp_forward_drop_packets++;
        }
      } else {
        const int written = client.write(raw, pkt_size);
        if (written == (int)pkt_size) {
          tcpTxBytes += (uint32_t)pkt_size;
        } else if (written > 0) {
          tcpTxBytes += (uint32_t)written;
          if (!enqueueTcpPending(raw + (size_t)written, pkt_size - (size_t)written)) {
            g_tcp_forward_drop_packets++;
          }
        } else {
          if (!enqueueTcpPending(raw, pkt_size)) {
            g_tcp_forward_drop_packets++;
          }
        }
      }
    }
  }

  if (gTransfer.status < 0) {
    g_last_transfer_status = gTransfer.status;
  }
}

static void serialTransferHudTick()
{
  const uint32_t now = millis();
  if (now - lastStatsMs < 200) return;
  lastStatsMs = now;

  const int afw = (client && client.connected()) ? client.availableForWrite() : -1;
  const uint32_t rx_age_ms = (g_last_rx_ms > 0u) ? (now - g_last_rx_ms) : 0u;
  const size_t pend = (tcpPendingLen >= tcpPendingOff) ? (tcpPendingLen - tcpPendingOff) : 0u;

  ansiSaveCursor();
  int r = 2;

  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("[STATS] up=%lus", (now - bootMs) / 1000);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("WiFi=%s", (WiFi.status() == WL_CONNECTED) ? "OK" : "DOWN");
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("TCP=%s", (client && client.connected()) ? "CONNECTED" : "none");
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("AFW=%d PEND=%u", afw, (unsigned)pend);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("RX_PKT=%lu ACK_TX=%lu", (unsigned long)g_rx_packet_count, (unsigned long)g_ack_tx_count);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("SEQ_LAST=%lu", (unsigned long)g_last_seq);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("DROP_EST=%lu", (unsigned long)g_drop_estimate);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("TCP_TX=%lu TCP_FWD_DROP=%lu", (unsigned long)tcpTxBytes, (unsigned long)g_tcp_forward_drop_packets);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("BUS_V=%.3fV", g_last_bus_voltage);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("RX_AGE=%lums TS_US=%lu ST=%d",
                                                                           (unsigned long)rx_age_ms,
                                                                           (unsigned long)g_last_rx_timestamp_us,
                                                                           (int)g_last_transfer_status);

  ansiRestoreCursor();
}

static void mdnsStart()
{
  MDNS.end();

  if (!MDNS.begin(MDNS_NAME)) {
    bannerPrintf("mDNS start failed (continuing without it).");
    return;
  }
  MDNS.addService("twr", "tcp", TCP_PORT);

  bannerPrintf("mDNS: %s.local  service: twr/tcp:%u", MDNS_NAME, TCP_PORT);
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
  bool nowConnected = (client && client.connected());

  // Detect disconnect
  if (lastTcpConnected && !nowConnected) {
    bannerPrintf("TCP client disconnected");
  }

  lastTcpConnected = nowConnected;

  // Already connected → nothing to do
  if (nowConnected) return;

  // Clean up dead client
  if (client) {
    client.stop();
  }

  // Check for new client
  WiFiClient newClient = server.available();
  if (newClient) {
    client = newClient;
    client.setNoDelay(true);

    tcpPendingLen = 0;
    tcpPendingOff = 0;

    bannerPrintf("TCP client connected from: %s",
                 client.remoteIP().toString().c_str());

    lastTcpConnected = true;
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

  const int maxIters = 4;
  int iters = 0;

  while (tcpPendingOff < tcpPendingLen && iters++ < maxIters) {
    size_t remaining = tcpPendingLen - tcpPendingOff;

    size_t chunk = remaining;
    if (chunk > 2048) chunk = 2048;

    int written = client.write(tcpPending + tcpPendingOff, chunk);
    if (written <= 0) {
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
  flushPendingToTcp();

  if (LOSSLESS_MODE && tcpPendingLen != 0) {
    return;
  }

  int avail = TeensyUart.available();
  if (avail <= 0) return;

  int toRead = (avail > (int)BUF_SZ) ? (int)BUF_SZ : avail;

  int n = TeensyUart.read(uartBuf, toRead);
  if (n <= 0) return;

  uartRxBytes += (uint32_t)n;
  rxLedUntilMs = millis() + RX_LED_HOLD_MS;

  // Optional one-time debug sample to log area (won't wreck HUD)
  static bool showedSample = false;
  if (!showedSample) {
    showedSample = true;
    logPrintf("UART RX sample (first 16 bytes): ");
    int m = (n < 16) ? n : 16;
    for (int i = 0; i < m; i++) {
      if (uartBuf[i] < 16) logPrintf("0");
      logPrintf("%02X ", uartBuf[i]);
    }
    logPrintf("\n");
  }

  if (!(client && client.connected())) {
    return;
  }

  int toSend = n;
  if (toSend > 2048) toSend = 2048;

  int written = client.write(uartBuf, toSend);
  if (written > 0) {
    tcpTxBytes += (uint32_t)written;
  } else {
    written = 0;
  }

  int consumed = written;
  int remaining = n - consumed;

  if (remaining > 0) {
    if (LOSSLESS_MODE) {
      int stash = remaining;
      if (stash > (int)BUF_SZ) stash = (int)BUF_SZ;
      memcpy(tcpPending, uartBuf + consumed, (size_t)stash);
      tcpPendingLen = (size_t)stash;
      tcpPendingOff = 0;
    } else {
      // test mode: drop remainder
    }
  }
}

// Stats block starts at row 2 (one line below banner)
static void statsTick()
{
  uint32_t now = millis();
  if (now - lastStatsMs < 1000) return;
  lastStatsMs = now;

  int afw = (client && client.connected()) ? client.availableForWrite() : -1;
  size_t pend = (tcpPendingLen >= tcpPendingOff) ? (tcpPendingLen - tcpPendingOff) : 0;

  // Save wherever the cursor is (log area), then go update HUD, then restore.
  ansiSaveCursor();

  // Row 2 is the first stats line (since row 1 is banner)
  int r = 2;

  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("[STATS] up=%lus", (now - bootMs) / 1000);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("WiFi=%s", (WiFi.status() == WL_CONNECTED) ? "OK" : "DOWN");
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("TCP=%s", (client && client.connected()) ? "CONNECTED" : "none");
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("AFW=%d", afw);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("PEND=%uB", (unsigned)pend);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("MODE=%s", LOSSLESS_MODE ? "lossless" : "testdrop");
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("UART_RX=%luB", uartRxBytes);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("UART_TX=%luB", uartTxBytes);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("TCP_RX=%luB", tcpRxBytes);
  ansiGotoRC(r++, 1); Serial.printf("\r"); ansiClearLine(); Serial.printf("TCP_TX=%luB", tcpTxBytes);

  ansiRestoreCursor();
}

void setup()
{
  if (SERIALTRANSFER_RX_MONITOR_MODE) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(DEBUG_BAUD);
    delay(300);
    bootMs = millis();
    hudInit();
    bannerPrintf("SerialTransfer RX monitor booting...");

    // Must be configured before begin() to absorb UART bursts.
    TeensyUart.setRxBufferSize(4096);
    TeensyUart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    // Disable SerialTransfer debug printing; surface status via HUD instead.
    gTransfer.begin(TeensyUart, false);
    g_last_bus_voltage = NAN;

    wifiConnect();
    mdnsStart();
    server.begin();
    bannerPrintf("SerialTransfer RX + TCP forwarding on port %u  (%s.local)", TCP_PORT, MDNS_NAME);
    return;
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(DEBUG_BAUD);
  Serial.setRxBufferSize(4096);
  delay(300);

  bootMs = millis();

  // Initialize HUD + scroll region
  hudInit();

  // Put boot info in banner (single line)
  bannerPrintf("=== ESP32 TWR repeater boot ===  Build: %s %s", __DATE__, __TIME__);

  TeensyUart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  TeensyUart.setRxBufferSize(8 * 1024);

  // Show key config in the log area (optional)
  logPrintf("UART2 pins: RX=%d TX=%d baud=%lu\n", UART_RX_PIN, UART_TX_PIN, (unsigned long)UART_BAUD);
  logPrintf("WiFi SSID: %s\n", WIFI_SSID);
  logPrintf("TCP port: %u  mDNS: %s.local\n", TCP_PORT, MDNS_NAME);
  logPrintf("BUF_SZ: %u  MODE: %s\n", (unsigned)BUF_SZ, LOSSLESS_MODE ? "lossless" : "testdrop");

  wifiConnect();
  mdnsStart();

  server.begin();
  bannerPrintf("TCP server listening on port %u  (%s.local)", TCP_PORT, MDNS_NAME);
}

void loop()
{
  if (SERIALTRANSFER_RX_MONITOR_MODE) {
    ledTick();
    if (WiFi.status() != WL_CONNECTED) {
      bannerPrintf("WiFi disconnected; reconnecting...");
      wifiConnect();
      mdnsStart();
      server.begin();
      bannerPrintf("SerialTransfer RX + TCP forwarding on port %u  (%s.local)", TCP_PORT, MDNS_NAME);
    }
    acceptClientIfNeeded();
    serialTransferRxTick();
    serialTransferHudTick();
    yield();
    return;
  }

 // Serial.println("alive");

  ledTick();
  statsTick();

  if (WiFi.status() != WL_CONNECTED) {
    bannerPrintf("WiFi disconnected; reconnecting...");
    wifiConnect();
    mdnsStart();
    server.begin();
    bannerPrintf("TCP server listening on port %u  (%s.local)", TCP_PORT, MDNS_NAME);
  }

  acceptClientIfNeeded();

  pumpTcpToUart();
  pumpUartToTcp();

  delay(1);
}
