# ESP32 WiFi Repeater for Teensy Telemetry

This folder contains an ESP32 bridge that forwards Teensy telemetry from UART to TCP.

## Overview

Data path:

1. Teensy logs telemetry samples while in `SUP_MODE_BALANCE_TWR`.
2. Teensy does **not** transmit telemetry during active balance.
3. On transition `SUP_MODE_BALANCE_TWR -> SUP_MODE_IDLE`, Teensy starts a flush on `Serial4`.
4. ESP32 (`wifi_repeater`) reads UART2 and forwards bytes to a TCP client.
5. Desktop client (`connect_raw.py`) receives raw bytes over TCP.

## Important Distinction: USB Serial vs Telemetry UART

Teensy prints JSON status lines like these on **USB Serial** (`Serial`):

- `TELEM_LOG_START`
- `TELEM_FLUSH_START`
- `TELEM_FLUSH_END`

Those are **not** sent on `Serial4`.

Actual telemetry packets sent on `Serial4` are binary fixed-size packets (64 bytes each).

## Current Wire Settings

### Teensy side

- Telemetry UART: `Serial4` (pins 16/17)
- Baud: `921600`

### ESP32 side (`src/main.cpp`)

- WiFi SSID: `Love Factory`
- TCP port: `9000`
- UART: `UART2`
- RX pin: `16` (from Teensy TX / Serial4 TX)
- TX pin: `17` (to Teensy RX / Serial4 RX)
- UART baud: `921600`
- `LOSSLESS_MODE = true`

## Running the Desktop Receiver

From this directory:

```bash
cd /Users/owhite/balancing-robot-notes/ESP32/wifi_repeater
python3 connect_raw.py
```

Defaults used by the script:

- Host: `twr-repeater.local`
- Port: `9000`

If mDNS resolution fails, pass IP explicitly:

```bash
python3 connect_raw.py --host <ESP32_IP> --port 9000
```

## Raw Binary Output (Expected Behavior)

`connect_raw.py` prints raw TCP bytes to stdout. Telemetry packets are binary, so terminal output may look empty/garbled.

To verify data capture, save to file:

```bash
python3 connect_raw.py > telem_dump.bin
```

Then inspect:

```bash
wc -c telem_dump.bin
xxd -g1 -l 128 telem_dump.bin
```

## Workflow Check

1. Start ESP32 repeater.
2. Start desktop receiver (`connect_raw.py`) before ending balance session.
3. Run balance session on Teensy.
4. Exit to idle (button reset).
5. Confirm Teensy USB Serial prints:
   - `TELEM_FLUSH_START`
   - `TELEM_FLUSH_END` with remaining bytes `0`
6. Confirm `connect_raw.py` captured binary bytes (file size increases).

## Note

If no TCP client is connected during flush, ESP32 may consume UART bytes without forwarding them to a desktop receiver. Keep the TCP client connected before triggering the flush.
