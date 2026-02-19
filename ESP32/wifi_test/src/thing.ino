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

// Debug serial (USB) baud
static const uint32_t DEBUG_BAUD = 115200;

// LED / heartbeat
static const int LED_PIN = 2;                    // GPIO2 is common onboard LED
static const uint32_t HEARTBEAT_MS = 250;

// LED RX-activity timing (when receiving data from Teensy over UART)
static const uint32_t RX_LED_HOLD_MS = 40;       // LED stays ON this long after last RX byte

// Buffers
static const size_t BUF_SZ = 1024;
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
    client.setNoDelay(true);

    Serial.print("TCP client connected from: ");
    Serial.println(client.remoteIP());
  }
}

static void pumpTcpToUart()
{
  if (!client || !client.connected()) return;

  uint8_t buf[BUF_SZ];
  int avail = client.available();
  if (avail <= 0) return;

  int toRead = (avail > (int)BUF_SZ) ? (int)BUF_SZ : avail;
  int n = client.read(buf, toRead);
  if (n > 0) {
    tcpRxBytes += (uint32_t)n;
    TeensyUart.write(buf, n);
    uartTxBytes += (uint32_t)n;

    // Debug: only print occasionally to avoid spam
    // Serial.printf("TCP->UART %d bytes\n", n);
  }
}

static void pumpUartToTcp()
{
  // IMPORTANT: We still want to confirm we are receiving UART data even if no TCP client.
  uint8_t buf[BUF_SZ];
  int avail = TeensyUart.available();
  if (avail <= 0) return;

  int toRead = (avail > (int)BUF_SZ) ? (int)BUF_SZ : avail;

  TeensyUart.setTimeout(2);
  int n = TeensyUart.readBytes(buf, toRead);

  if (n > 0) {
    uartRxBytes += (uint32_t)n;

    // Trigger LED RX activity indicator
    rxLedUntilMs = millis() + RX_LED_HOLD_MS;

    // Debug: show first few bytes of the first packet after boot
    static bool showedSample = false;
    if (!showedSample) {
      showedSample = true;
      Serial.print("UART RX sample (first 16 bytes): ");
      int m = (n < 16) ? n : 16;
      for (int i = 0; i < m; i++) {
        if (buf[i] < 16) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    // If TCP client exists, forward; otherwise just drop (but counters prove receipt)
    if (client && client.connected()) {
      client.write(buf, n);
      tcpTxBytes += (uint32_t)n;
    }
  }
}

static void statsTick()
{
  uint32_t now = millis();
  if (now - lastStatsMs < 1000) return;
  lastStatsMs = now;

  Serial.print("[STATS] up=");
  Serial.print((now - bootMs) / 1000);
  Serial.print("s  WiFi=");
  Serial.print((WiFi.status() == WL_CONNECTED) ? "OK" : "DOWN");
  Serial.print("  TCP=");
  Serial.print((client && client.connected()) ? "CONNECTED" : "none");
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
