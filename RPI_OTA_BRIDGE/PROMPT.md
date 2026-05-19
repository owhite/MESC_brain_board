# Prompt Used To Generate This Project

I want to build a Raspberry Pi Zero 2 W Headless Teensy OTA Bridge in this workspace:
 /Users/owhite/balancing-robot-notes/RPI_OTA_BRIDGE

Implement the full project end to end without asking permission to create, edit, move, or copy files.

Project goals:
1. Raspberry Pi Zero 2 W acts as an OTA buffer and flasher for Teensy 4.0.
2. Desktop sends firmware to Pi with SCP.
3. Pi detects completed file transfer with Linux inotify via Python watchdog on_closed event.
4. Pi flashes Teensy using teensy_loader_cli and reports status back to desktop.

Technical requirements:
1. OS and provisioning assumptions
- Raspberry Pi OS Lite 64-bit.
- In Raspberry Pi Imager use hostname teensybridge.
- Access from desktop via teensybridge.local.
- SSH enabled, key auth preferred.
- Wi-Fi preconfigured.

2. Package and toolchain setup on Pi
- Install required packages with apt: build-essential, libusb-dev, git, python3-watchdog, openssh-client.
- Compile teensy_loader_cli from PJRC GitHub source on Pi for ARM compatibility.
- Install loader to /usr/local/bin/teensy_loader_cli.

3. Udev and USB permissions
- Create udev rules file at /etc/udev/rules.d/49-teensy.rules.
- Allow non-root access for PJRC USB vendor 16c0 (runtime and bootloader visibility).
- Reload udev rules in install flow.

4. Bridge service behavior
- Create bridge.py as a systemd-managed Python service.
- Watch directory: /home/<pi_user>/teensy_drops.
- Trigger only on watchdog on_closed events.
- Accept only .hex uploads.
- Flash command: teensy_loader_cli --mcu=TEENSY40 -w -s -v <file>.
- Capture stdout, stderr, exit code.
- Write full report to /home/<pi_user>/teensy_drops/last_upload_status.log.
- Move processed .hex to /home/<pi_user>/teensy_drops/processed with timestamp and ok/fail suffix.
- Do not reflash stale files on reboot.

5. Systemd
- Provide a persistent service unit.
- Service must work for custom Pi usernames, not hardcoded pi.
- Support configurable drop directory via environment variable TEENSY_DROP_DIR.

6. Desktop PlatformIO integration
- Provide desktop script upload_to_pi.py for PlatformIO post upload hook.
- Script copies firmware .hex to Pi via SCP.
- Script immediately tails remote status log over SSH.
- Script exits success only when EXIT_CODE=0 appears in log.
- On failure or timeout, print remote log and exit nonzero.
- Add required platformio.ini settings:
  extra_scripts = post:upload_to_pi.py
  upload_protocol = custom

7. Project structure and deliverables
- Create clean subdirectories for pi and desktop assets.
- Include install scripts for loader, udev, and bridge service.
- Include README with:
  - exact setup steps
  - verification commands
  - known failure modes and fixes
  - reproducibility note containing current short commit hash

8. Validation
- Run syntax checks for Python scripts.
- Ensure no obvious runtime path mismatches between scripts, service, and docs.
- Summarize created files and final run/test commands.

Important constraints:
1. Do not prompt me for file operation permission.
2. Prefer robust defaults for headless operation.
3. Keep output paths user-aware via <pi_user>.
4. Prioritize reliable hex upload flow for Teensy 4.0.

If anything is ambiguous, choose the implementation path most likely to work on Raspberry Pi OS trixie and explain the decision briefly in README.
