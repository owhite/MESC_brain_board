
# PROMPT FOR CHATGPT:
This codebase was created from the prompt in [PROMPT.md](PROMPT.md).

## Headless Teensy OTA Bridge (Pi Zero 2 W)

Reference commit: `89e9f187`
Date: 2026-05-05

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
```

If your Pi login user is not `pi`, set `BRIDGE_USER` before the bridge install step:

```bash
export BRIDGE_USER=<your_pi_username>
./pi/scripts/install_bridge_service.sh
```

The installer renders the systemd unit with your user home and sets `TEENSY_DROP_DIR` automatically.

Then verify:

```bash
systemctl status teensy-bridge.service --no-pager
journalctl -u teensy-bridge.service -n 50 --no-pager
```

## Desktop PlatformIO Integration

1. Copy `desktop/upload_to_pi.py` into your PlatformIO project root.
2. Add contents of `desktop/platformio.ini.snippet` to your `platformio.ini`.
3. Optional environment overrides:

```bash
export TEENSY_BRIDGE_HOST=teensybridge.local
export TEENSY_BRIDGE_USER=<your_pi_username>
export TEENSY_BRIDGE_DROP_DIR=/home/<your_pi_username>/teensy_drops
```

If your Pi hostname differs, set `TEENSY_BRIDGE_HOST` accordingly.

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

## Phase 2 Roadmap

Documentation update reference commit: `89e9f187`

### Scope

Phase 2 focuses on operational hardening and faster recovery in unattended deployments.

### Planned Enhancements

1. Upload Retry and Timeout Policy
	- Add bounded retry logic for transient flash failures.
	- Add explicit timeout handling and deterministic exit codes.
	- Surface retry count and timeout reason in `last_upload_status.log`.

2. Health and Diagnostics Surface
	- Add a lightweight health command/script to summarize:
	  - systemd service state
	  - last upload result and timestamp
	  - drop/processed directory status
	- Add a compact one-shot diagnostics command for support handoff.

3. One-Command Bootstrap
	- Add a single setup entry point to provision loader, udev rules, service files, and dependencies.
	- Ensure custom username compatibility remains first-class.
	- Include post-install verification checks with pass/fail output.

### Acceptance Criteria

1. A failed upload returns a clear nonzero code with reason labels (timeout, usb_unavailable, loader_error).
2. Automatic retries are bounded and logged; no infinite retry loops.
3. One command from a fresh Pi reaches an active bridge service and passing verification output.
4. Health command gives enough context to diagnose the top known failure modes in under 60 seconds.
