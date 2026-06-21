# RPI Upload/Download Checklist

Reference commit: `c7eb63e1`
Date: 2026-06-21

## Goal

Implement a full run loop:

1. Desktop builds and uploads Teensy firmware via Pi bridge.
2. Teensy emits structured mock logs on USB Serial.
3. Pi listener captures and persists per-run artifacts.
4. Desktop waits for completion, fetches artifacts, and stores by run ID.
5. Offline analysis runs on desktop artifacts.

## Proposed Contracts

### Run ID

- Format: `YYYYMMDD_HHMMSS_<shortid>`
- Created on desktop before upload.
- Propagated everywhere:
  - firmware metadata (optional compile define)
  - Pi runtime paths
  - completion signal
  - fetched desktop archive path

### Serial Message Format (Teensy -> Pi)

Use line-delimited JSON for robust parsing:

```json
{"run_id":"20260621_101530_a1b2c3","seq":1,"t_ms":0,"level":"INFO","msg":"RUN_START"}
{"run_id":"20260621_101530_a1b2c3","seq":2,"t_ms":100,"level":"DATA","val":123}
{"run_id":"20260621_101530_a1b2c3","seq":999,"t_ms":5000,"level":"INFO","msg":"RUN_DONE"}
```

Rules:

- One JSON object per line.
- Include monotonic `seq` and `t_ms`.
- End run with `msg":"RUN_DONE"`.

### Completion Signal

Primary:

- Detect `{"msg":"RUN_DONE" ...}` on Pi listener.

Fallback:

- If no done marker within timeout, mark run `timed_out`.

## File Layout

### Pi paths

Base: `/home/<pi_user>/teensy_runs`

Per run:

- `<run_id>/meta.json`
- `<run_id>/serial_raw.log`
- `<run_id>/serial_parsed.jsonl`
- `<run_id>/status.json`

### Desktop paths

Base: `<project>/logs`

Per run:

- `<run_id>/meta.json`
- `<run_id>/status.json`
- `<run_id>/serial_raw.log`
- `<run_id>/serial_parsed.jsonl`

## Implementation Plan

### Phase 1: Pi serial listener service

1. Add Pi script `pi/listener/serial_listener.py`.
2. Resolve Teensy serial device (`/dev/ttyACM*`) with reconnect loop.
3. Read line-by-line with timestamping on Pi.
4. Persist `serial_raw.log` and parsed `jsonl`.
5. Set `status.json` transitions:
   - `starting` -> `running` -> `completed` or `timed_out` or `error`.
6. Add timeout support (for example 120s).

Acceptance:

- Listener survives disconnect/reconnect and keeps writing artifacts.

### Phase 2: Pi listener systemd integration

1. Add `pi/systemd/teensy-listener.service`.
2. Add install script `pi/scripts/install_listener_service.sh`.
3. Ensure run base directory permissions for non-root user.

Acceptance:

- `systemctl status teensy-listener.service` is active after reboot.

### Phase 3: Desktop run orchestration

1. Add desktop helper (new script) to:
   - create `run_id`
   - write run start marker on Pi (for example `current_run_id` file)
   - invoke `pio run -t bridge_send`
   - wait for Pi `status.json` terminal state
   - fetch artifacts via `scp`
2. Keep deterministic exit codes:
   - `0` success
   - `124` timeout
   - nonzero for transfer/flash/listener errors

Acceptance:

- One command executes full upload-run-fetch loop.

### Phase 4: Desktop analysis hook

1. Add a simple parser/report script over `serial_parsed.jsonl`.
2. Emit summary stats and plots-ready CSV if needed.

Acceptance:

- Analysis can run from local fetched artifacts without Pi access.

## Suggested Command Surface

From firmware project:

- Build + flash only:
  - `pio run -t bridge_send`

From orchestration script:

- Full cycle (build, flash, wait, fetch):
  - `./run_cycle.sh`

## Failure Modes To Handle

1. Teensy not found on serial device.
2. Flash succeeded but no serial output appears.
3. Partial JSON lines or malformed records.
4. No completion marker before timeout.
5. SSH/SCP transfer interruptions.
6. Stale previous-run files causing false success.

## Minimum Test Matrix

1. Happy path run completes with valid fetched logs.
2. Teensy unplugged before run start.
3. Teensy reset mid-run.
4. Completion marker intentionally omitted (timeout expected).
5. Network interruption during artifact fetch.

## First Coding Steps

1. Create `pi/listener/serial_listener.py` with JSONL + status writer.
2. Create `pi/systemd/teensy-listener.service` and installer script.
3. Create desktop orchestrator script to poll `status.json` and fetch artifacts.
4. Add README section with commands and troubleshooting.
