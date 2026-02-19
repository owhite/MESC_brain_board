# System Summary --- Teensy ⇄ ESP32 ⇄ Wi-Fi ⇄ Desktop

**An ESP32-WROOM-32 (Xtensa LX6) running Arduino firmware acts as a
Wi-Fi TCP bridge between Teensy 4.0 Serial1 (GPIO16/17 via UART2) and a
desktop client, enabling reliable post-run binary data transfer over the
home network using mDNS discovery (`twr-repeater.local:9000`).**

## ESP32 hardware & architecture

**Board:** WEMOS D1 mini ESP32--class (module: **ESP32-WROOM-32**)\
**Chip:** **ESP32-D0WD-V3**\
**CPU:** Dual-core **Xtensa LX6 @ 240 MHz**\
**RAM:** 520 KB internal\
**Flash:** external (typically 4 MB)\
**USB:** via CH340 → **UART0 only** (not native USB)

Framework: **Arduino (PlatformIO compatible)**

------------------------------------------------------------------------

## UART usage on the ESP32

### Reserved

-   **UART0 → `Serial` → USB**
    -   GPIO1 = TX0\
    -   GPIO3 = RX0\
        Used for:
    -   flashing
    -   debug prints\
        **Not connected to the Teensy**

### Teensy link

Using **UART2** (remappable and safe):

  Function   ESP32 GPIO   Connects to Teensy
  ---------- ------------ ------------------------
  ESP32 RX   **GPIO16**   Teensy **TX1 (pin 1)**
  ESP32 TX   **GPIO17**   Teensy **RX1 (pin 0)**
  GND        GND          GND

Baud rate: **115200**

------------------------------------------------------------------------

## LED

**GPIO2** - 250 ms heartbeat when idle - Solid/flicker when UART data is
being received

------------------------------------------------------------------------

# Wi-Fi configuration

ESP32 runs in **STA mode** and connects to:

-   **SSID:** `Monty's Love Factory`
-   **Password:** `HotMonkeyMonty`

Network discovery via:

-   **mDNS hostname:**\
    `twr-repeater.local`

TCP server:

-   **Port:** `9000`

------------------------------------------------------------------------

# Data flow architecture

## Physical chain

Teensy 4.0\
Serial1 (115200)\
│\
▼\
ESP32 UART2 (GPIO16/17)\
│\
▼\
ESP32 Wi-Fi (STA mode)\
│\
▼\
Home network\
│\
▼\
Desktop Python client (TCP)

------------------------------------------------------------------------

## Logical behavior

### ESP32 role

The ESP32 is a **transparent binary bridge**.

It:

1.  Connects to house Wi-Fi\
2.  Advertises via mDNS (`twr-repeater.local`)\
3.  Opens TCP server on port 9000\
4.  Forwards bytes

#### Teensy → Desktop

UART2 RX → TCP socket

#### Desktop → Teensy

TCP socket → UART2 TX

No packet interpretation. No protocol logic. Pure byte pipe.

------------------------------------------------------------------------

## Current tested behavior

### Verified working

-   Teensy test packets transmitted on Serial1\
-   ESP32 receives them → confirmed by increasing `UART_RX`\
-   Desktop connects via `twr-repeater.local:9000`\
-   Desktop receives correct number of bytes over TCP\
-   End-to-end chain validated

------------------------------------------------------------------------

# Current operating mode

This is a **post-run bulk transfer system**:

-   Teensy logs data locally during the robot run\
-   After the run it sends a single stream over Serial1\
-   ESP32 forwards that stream over Wi-Fi to the desktop

So this is **not a real-time telemetry link** --- it is a **reliable
dump channel**.

------------------------------------------------------------------------

# Debug instrumentation currently in firmware

ESP32 prints once per second:

    [STATS] up=XXs
    WiFi=OK/DOWN
    TCP=CONNECTED/none
    UART_RX=bytes
    UART_TX=bytes
    TCP_RX=bytes
    TCP_TX=bytes

This allows immediate diagnosis of:

-   UART wiring\
-   Wi-Fi status\
-   Desktop connection\
-   Direction of data flow

------------------------------------------------------------------------

# Desktop side

Python connects using hostname:

``` python
socket.create_connection(("twr-repeater.local", 9000))
```

No IP hard-coding required.

Current test: - Receive raw bytes\
- Count total received

------------------------------------------------------------------------

# What this system is (conceptually)

A **binary-safe wireless serial tunnel** between:

-   a real-time controller (**Teensy 4.0**)\
-   a high-level logging/analysis computer

with:

-   no timing impact on the control loop\
-   no USB tether\
-   reliable transport via TCP

------------------------------------------------------------------------

# What has NOT been added yet (by design)

These are future features, not missing pieces:

-   Command protocol (e.g., "start dump" trigger)\
-   Length-prefixed transfer\
-   CRC / integrity check\
-   Ring buffer for late TCP connects\
-   AP fallback mode

The transport layer itself is complete and validated.

------------------------------------------------------------------------

# Minimal wiring reference

### Teensy 4.0 ⇄ ESP32

Teensy pin 1 (TX1) → ESP32 GPIO16\
Teensy pin 0 (RX1) ← ESP32 GPIO17\
GND ↔ GND

USB remains connected only to ESP32 for flashing/debug.

------------------------------------------------------------------------
# ESP CODE
```c
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

```
-------------------------------------------------
# TEENSY CODE
```c
#include <Arduino.h>

#define TEST_INTERVAL_MS 1000

// A simple binary test packet
struct TestPacket {
  uint32_t counter;
  uint32_t millisStamp;
  uint8_t  payload[16];
};

uint32_t lastSend = 0;
uint32_t txCounter = 0;

void sendTestPacket()
{
  TestPacket pkt;

  pkt.counter = txCounter++;
  pkt.millisStamp = millis();

  for (int i = 0; i < 16; i++) {
    pkt.payload[i] = i + pkt.counter;
  }

  Serial1.write((uint8_t*)&pkt, sizeof(pkt));

  Serial.print("Sent packet #");
  Serial.print(pkt.counter);
  Serial.print("  size=");
  Serial.println(sizeof(pkt));
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

  Serial.println("\nTeensy Serial1 <-> ESP32 link test");

  Serial1.begin(115200);
}

void loop()
{
  // Send a packet once per second
  if (millis() - lastSend >= TEST_INTERVAL_MS) {
    lastSend = millis();
    sendTestPacket();
  }

  // Check for data coming back from ESP32
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    Serial.print("RX byte from ESP32: 0x");
    if (b < 16) Serial.print("0");
    Serial.println(b, HEX);
  }
}

```
-------------------------------------------------
# PYTHON CODE
```c
#!/usr/bin/env python3

import socket, time
s = socket.create_connection(("twr-repeater.local", 9000), timeout=5)
s.settimeout(1.0)
t0 = time.time()
total = 0
while time.time() - t0 < 5:
    try:
        d = s.recv(4096)
        if not d: break
        total += len(d)
    except socket.timeout:
        pass
print("received", total, "bytes")
s.close()

```
