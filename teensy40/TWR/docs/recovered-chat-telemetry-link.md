# Recovered Chat: telemetry_link work

If the Codex thread UI spins, use this recovered reference.

## Original thread metadata
- Thread name: `Review Serial1 output from balance_T`
- Thread id: `019cca67-dd9d-7e53-8b38-ba13372f8b98`
- Transcript file:
  `/Users/owhite/.codex/sessions/2026/03/07/rollout-2026-03-07T17-25-30-019cca67-dd9d-7e53-8b38-ba13372f8b98.jsonl`

## Key result from that chat (2026-04-21 around 23:11 UTC)
- Added `src/telemetry_link.h` and `src/telemetry_link.cpp`
- Wired telemetry into:
  - `src/main.h`
  - `src/main.cpp`
  - `src/balance_TWR_mode.cpp`
- Transport design summary from that thread:
  - fixed-size 64-byte packet
  - CRC16-CCITT + footer `0xDEAD`
  - SPSC ring buffers for fast/event channels
  - non-blocking UART pump (`availableForWrite`)
  - event-priority overflow policy

## Direct extraction commands
Run these from a terminal to pull the exact final message lines:

```bash
rg -n "Implemented\. We now have a dedicated, non-blocking Teensy→ESP32 telemetry link|telemetry_link\.h|telemetry_link\.cpp|TELEM_LINK_CFG" \
  /Users/owhite/.codex/sessions/2026/03/07/rollout-2026-03-07T17-25-30-019cca67-dd9d-7e53-8b38-ba13372f8b98.jsonl
```

```bash
sed -n '6558,6565p' \
  /Users/owhite/.codex/sessions/2026/03/07/rollout-2026-03-07T17-25-30-019cca67-dd9d-7e53-8b38-ba13372f8b98.jsonl
```
