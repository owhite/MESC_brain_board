#!/usr/bin/env python3
import datetime as dt
import os
import shutil
import subprocess
from pathlib import Path

from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

WATCH_DIR = Path(os.environ.get("TEENSY_DROP_DIR", "/home/pi/teensy_drops"))
PROCESSED_DIR = WATCH_DIR / "processed"
STATUS_LOG = WATCH_DIR / "last_upload_status.log"
MCU = "TEENSY40"
LOADER = "/usr/local/bin/teensy_loader_cli"
ALLOWED_SUFFIXES = {".hex"}


class BinClosedHandler(FileSystemEventHandler):
    def on_closed(self, event):
        if event.is_directory:
            return

        src_path = Path(event.src_path)
        if src_path.suffix.lower() not in ALLOWED_SUFFIXES:
            return

        run_upload(src_path)


def run_upload(firmware_path: Path):
    PROCESSED_DIR.mkdir(parents=True, exist_ok=True)
    WATCH_DIR.mkdir(parents=True, exist_ok=True)

    # -s requests a software reboot into bootloader when supported by current firmware.
    command = [LOADER, f"--mcu={MCU}", "-w", "-s", "-v", str(firmware_path)]
    started = dt.datetime.now(dt.timezone.utc)

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
        )
        exit_code = result.returncode
        stdout = result.stdout
        stderr = result.stderr
    except Exception as exc:
        exit_code = 255
        stdout = ""
        stderr = f"Bridge exception: {exc}\n"

    finished = dt.datetime.now(dt.timezone.utc)
    status = "SUCCESS" if exit_code == 0 else "FAILURE"

    report = [
        "=== TEENSY OTA BRIDGE REPORT ===",
        f"START_UTC={started.isoformat()}",
        f"END_UTC={finished.isoformat()}",
        f"FIRMWARE={firmware_path}",
        f"COMMAND={' '.join(command)}",
        f"RESULT={status}",
        f"EXIT_CODE={exit_code}",
        "--- STDOUT ---",
        stdout.rstrip(),
        "--- STDERR ---",
        stderr.rstrip(),
        "=== END REPORT ===",
        "",
    ]

    STATUS_LOG.write_text("\n".join(report), encoding="utf-8")

    timestamp = finished.strftime("%Y%m%d_%H%M%S")
    suffix = "ok" if exit_code == 0 else "fail"
    target_name = f"{firmware_path.stem}_{timestamp}.{suffix}{firmware_path.suffix.lower()}"
    target = PROCESSED_DIR / target_name

    try:
        shutil.move(str(firmware_path), str(target))
    except Exception:
        # File may already be gone; status log is already written.
        pass


def main():
    WATCH_DIR.mkdir(parents=True, exist_ok=True)
    PROCESSED_DIR.mkdir(parents=True, exist_ok=True)

    observer = Observer()
    observer.schedule(BinClosedHandler(), str(WATCH_DIR), recursive=False)
    observer.start()

    try:
        while True:
            observer.join(1)
    except KeyboardInterrupt:
        observer.stop()

    observer.join()


if __name__ == "__main__":
    main()
