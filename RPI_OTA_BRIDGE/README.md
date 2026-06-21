# Headless Teensy OTA Bridge (Pi Zero 2 W)

Reference commit: `89e9f187`
Date: 2026-05-05
This codebase was created from the prompt in [PROMPT.md](PROMPT.md).

## To invoke upload

Documentation update reference commit: `c7eb63e1`

Working method (validated): run from your PlatformIO project that contains `bridge_upload.py` and its `platformio.ini` wiring (example project: `teensy40/blink`).

```bash
cd /path/to/teensy40/blink
pio run -t bridge_send
```

The same command with explicit overrides:

```bash
TEENSY_BRIDGE_HOST=teensybridge.local TEENSY_BRIDGE_USER=owhite TEENSY_BRIDGE_DROP_DIR=/home/owhite/teensy_drops pio run -t bridge_send
```

Help command for this integration:

```bash
pio run -t bridge_help
```

Optional wrapper commands (if `pio_bridge.sh` is present in that PlatformIO project root):

```bash
./pio_bridge.sh --bridge_help
./pio_bridge.sh --bridge_send
```

What does not work from this repository root (`RPI_OTA_BRIDGE`) unless you add those custom targets to that project:

```bash
pio run -t bridge_send
pio run -t bridge_help
```

**NOTE:** some times you have to press the button on the teensy, but usually just for the first upload of the session. 

## Upload/Download Expansion Plan

For the new Teensy -> Pi -> Desktop run-log return pipeline, use:

- [RPI_UPLOAD_DOWNLOAD_CHECKLIST.md](RPI_UPLOAD_DOWNLOAD_CHECKLIST.md)

Phase 1 implementation reference commit: `c7eb63e1`

## Construction notes

This workspace implements your spec for a headless Raspberry Pi OTA bridge:

- Desktop -> Pi transport via `scp`
- Pi-side file completion trigger via Linux inotify (Python Watchdog `on_closed`)
- Flash target via `teensy_loader_cli --mcu=TEENSY40 -w -s -v`
- Upload status persisted to `/home/<your_pi_username>/teensy_drops/last_upload_status.log`
- Processed artifacts (`.hex`) moved to `/home/<your_pi_username>/teensy_drops/processed`

## Project Layout

- `pi/udev/49-teensy.rules`: non-root Teensy bootloader access
- `pi/scripts/install_teensy_loader_cli.sh`: build/install PJRC loader on ARM
- `pi/scripts/install_udev_rules.sh`: deploy udev rule to `/etc/udev/rules.d/`
- `pi/bridge/bridge.py`: watchdog service implementation
- `pi/bridge/requirements.txt`: optional Python dependency list (`watchdog`)
- `pi/systemd/teensy-bridge.service`: persistent service unit
- `pi/scripts/install_bridge_service.sh`: install Python dependency and systemd service
- `pi/listener/serial_listener.py`: Pi-side Teensy USB serial capture worker
- `pi/systemd/teensy-listener.service`: persistent Teensy serial listener unit
- `pi/scripts/install_listener_service.sh`: install serial listener dependency and service
- `desktop/upload_to_pi.py`: PlatformIO post-upload script (SCP + SSH log follow)
- `desktop/platformio.ini.snippet`: required PlatformIO config snippet

## mounted Pi: 
[![pi mount](bridge_mount)](bridge_mount "RasPi zero mounted to bot")

## Pi Provisioning Baseline

Raspberry Pi OS Lite (64-bit), hostname `teensybridge`, SSH enabled, Wi-Fi configured.

Note: Connect using mDNS name `teensybridge.local` from your desktop.

Required packages from spec are covered by scripts:

- `build-essential`
- `libusb-dev`
- `python3-watchdog`
- `git`

Note: On Raspberry Pi OS trixie (PEP 668), the bridge installer uses `python3-watchdog` from apt instead of pip.

## Pi Installation Steps

Run these on the Pi from this repository root:

```bash
chmod +x pi/scripts/*.sh
./pi/scripts/install_teensy_loader_cli.sh
./pi/scripts/install_udev_rules.sh
./pi/scripts/install_bridge_service.sh
./pi/scripts/install_listener_service.sh
```

If your Pi login user is not `pi`, set `BRIDGE_USER` before the bridge install step:

```bash
export BRIDGE_USER=<your_pi_username>
./pi/scripts/install_bridge_service.sh
./pi/scripts/install_listener_service.sh
```

The installer renders the systemd unit with your user home and sets `TEENSY_DROP_DIR` automatically.

Then verify:

```bash
systemctl status teensy-bridge.service --no-pager
systemctl status teensy-listener.service --no-pager
journalctl -u teensy-bridge.service -n 50 --no-pager
journalctl -u teensy-listener.service -n 50 --no-pager
```

Quick listener smoke test (on Pi):

```bash
RUN_ID=$(date +%Y%m%d_%H%M%S)_smoke
echo "$RUN_ID" > /home/$USER/teensy_runs/current_run_id
sleep 2
cat /home/$USER/teensy_runs/$RUN_ID/status.json
```

Completion marker note:

- The listener marks a run `completed` on either:
	- JSON line with `"msg":"RUN_DONE"` (optional matching `run_id`), or
	- plain-text line containing `RUN_DONE` (optionally `RUN_ID=<run_id>`).

Listener growth safety defaults:

- Per-run timeout: `RUN_TIMEOUT_SEC=120`
- Per-run byte cap: `MAX_RUN_BYTES=20971520` (20 MiB)
- Per-run record cap: `MAX_RUN_RECORDS=200000`
- Old run retention cap: `KEEP_RUN_DIRS=200` (oldest run directories are pruned)

Additional safety behavior:

- Run logs are overwritten per run ID (not appended) to prevent restart replay growth.
- Completed/timed-out/error runs are not replayed on service restart.

## Desktop PlatformIO Integration

Validated current workflow (recommended):

1. Use your PlatformIO project with bridge targets already wired (example: `teensy40/blink`).
2. From that project directory run:

```bash
pio run -t bridge_send
```

3. Optional environment overrides:

```bash
export TEENSY_BRIDGE_HOST=teensybridge.local
export TEENSY_BRIDGE_USER=<your_pi_username>
export TEENSY_BRIDGE_DROP_DIR=/home/<your_pi_username>/teensy_drops
```

If your Pi hostname differs, set `TEENSY_BRIDGE_HOST` accordingly.

Alternative workflow (this repo's generic hook):

1. Copy `desktop/upload_to_pi.py` into your PlatformIO project root.
2. Add contents of `desktop/platformio.ini.snippet` to your `platformio.ini`.
3. Run `pio run -t upload` from that PlatformIO project.

## One-Command Run Cycle (Desktop)

Desktop orchestration reference commit: `c7eb63e1`

Use this script to run one full cycle:

1. write a new `run_id` on Pi (`current_run_id`)
2. build + upload firmware (`pio run -t bridge_send`)
3. wait for Pi listener terminal status (`completed`, `timed_out`, `error`, `size_limit`)
4. fetch `/home/<pi_user>/teensy_runs/<run_id>/` artifacts to local logs

From this repository root:

```bash
./desktop/run_cycle.py --firmware-dir /path/to/teensy40/blink --user owhite --host teensybridge.local
```

Optional arguments:

```bash
./desktop/run_cycle.py \
	--firmware-dir /path/to/teensy40/blink \
	--logs-dir ./logs \
	--wait-timeout 240 \
	--keep-local-runs 100 \
	--user owhite \
	--host teensybridge.local
```

Local retention behavior:

- `--keep-local-runs` (default `100`) prunes oldest local run directories in `--logs-dir` after each run.

Exit behavior:

- `0`: run completed and artifacts fetched
- `124`: listener timed out or wait timeout exceeded
- nonzero: build/upload/fetch/listener error

## Full End-to-End Cycle (Runbook)

Documentation cycle reference commit: `c7eb63e1`

Use this sequence for the complete workflow:

1. Confirm Pi services are active:

```bash
ssh owhite@teensybridge.local "systemctl is-active teensy-bridge.service teensy-listener.service"
```

2. Run one full desktop cycle (build -> flash -> wait -> fetch):

```bash
cd /Users/owhite/balancing-robot-notes/RPI_OTA_BRIDGE
./desktop/run_cycle.py --firmware-dir /Users/owhite/balancing-robot-notes/teensy40/RPI_upload_download --user owhite --host teensybridge.local
```

3. Find the newest fetched run directory:

```bash
LATEST_RUN="$(ls -1t logs | head -n 1)"
echo "$LATEST_RUN"
ls -la "logs/$LATEST_RUN"
```

4. Verify outcome and key metrics:

```bash
cat "logs/$LATEST_RUN/status.json"
```

Expected success shape:

- `state` is `completed`
- `error` is empty
- `records` and `bytes` are nonzero

5. Inspect captured logs:

```bash
head -n 20 "logs/$LATEST_RUN/serial_raw.log"
head -n 20 "logs/$LATEST_RUN/serial_parsed.jsonl"
```

Expected log semantics:

- first meaningful marker is `RUN_START`
- periodic data lines follow
- completion marker `RUN_DONE` is present

If run ends as `timed_out` but artifacts exist, transport/capture worked and completion marker was not observed before timeout.

## Runtime Behavior

On upload:

1. Desktop script clears the remote status log and ensures drop directories exist.
2. Desktop script copies firmware (`.hex`) via `scp`.
3. Pi watchdog sees a file close event (`on_closed`) for `.hex` and runs teensy loader.
4. Desktop script streams `tail -f /home/<your_pi_username>/teensy_drops/last_upload_status.log`.
5. Desktop script exits success only when `EXIT_CODE=0`; otherwise exits with error.

## Notes

- The bridge is intentionally headless and unattended.
- Processed firmware is renamed with timestamp and result suffix (`.ok.hex` or `.fail.hex`).
- Soft reboot mode (`-s`) avoids button presses when the running firmware exposes a USB path that supports reboot requests.
- If firmware is crashed or USB is unavailable, manual button press may still be required.
- Trigger semantics: upload starts on inotify `on_closed` for new `.hex` writes in the drop folder.

## Known Failure Modes

1. **No USB events on Pi (`dmesg -w` stays silent)**
	- Symptom: plugging/unplugging Teensy shows nothing in `sudo dmesg -w`.
	- Cause: wrong Pi Zero 2 W port (`PWR IN` instead of `USB` data), bad OTG adapter, or charge-only cable.
	- Fix: use the `USB` data port with a known data cable/OTG adapter; verify events appear in `dmesg -w`.

2. **SSH key login fails (`Permission denied (publickey)`)**
	- Symptom: `ssh <user>@teensybridge.local` reaches host, then auth fails.
	- Cause: wrong SSH username or mismatched key in Imager/authorized_keys.
	- Fix: connect with the correct username; verify matching key pair; use password auth for first boot if needed.

3. **`externally-managed-environment` during install**
	- Symptom: pip install fails on Raspberry Pi OS trixie.
	- Cause: PEP 668 system Python protections.
	- Fix: installer uses apt package `python3-watchdog`; avoid system pip installs for bridge dependency.

4. **Loader waits forever (`Waiting for Teensy device...`)**
	- Symptom: `teensy_loader_cli` parses firmware then waits for device.
	- Cause: Teensy bootloader not entered yet, USB path unavailable, or firmware cannot soft-reboot.
	- Fix: try `-s` soft reboot path first; if still waiting, press Teensy button; confirm USB visibility with `lsusb`/`dmesg`.

5. **HEX/BIN mismatch parse errors**
	- Symptom: errors like `error reading intel hex file ...firmware.bin`.
	- Cause: dropping `.bin` while loader expects Intel HEX.
	- Fix: bridge is `.hex`-only; ensure desktop uploader transfers `.hex` artifacts.

6. **Upload not triggered even with file in drop folder**
	- Symptom: old file present but no flash attempt.
	- Cause: trigger is event-based (`on_closed`), not presence-based.
	- Fix: copy/write a new `.hex` file into the drop directory; stale files do not retrigger.

