#!/usr/bin/env python3
import datetime as dt
import glob
import json
import os
import re
import shutil
import time
from pathlib import Path

import serial

RUN_BASE = Path(os.environ.get("TEENSY_RUN_BASE", "/home/pi/teensy_runs"))
RUN_ID_FILE = Path(os.environ.get("TEENSY_RUN_ID_FILE", str(RUN_BASE / "current_run_id")))
SERIAL_GLOB = os.environ.get("SERIAL_GLOB", "/dev/ttyACM*")
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))
RUN_TIMEOUT_SEC = int(os.environ.get("RUN_TIMEOUT_SEC", "120"))
IDLE_SLEEP_SEC = float(os.environ.get("IDLE_SLEEP_SEC", "0.5"))
LINE_TIMEOUT_SEC = float(os.environ.get("LINE_TIMEOUT_SEC", "1.0"))
MAX_RUN_BYTES = int(os.environ.get("MAX_RUN_BYTES", str(20 * 1024 * 1024)))
MAX_RUN_RECORDS = int(os.environ.get("MAX_RUN_RECORDS", "200000"))
KEEP_RUN_DIRS = int(os.environ.get("KEEP_RUN_DIRS", "200"))
RUN_ID_RE = re.compile(r"^[A-Za-z0-9_.-]{4,80}$")
TERMINAL_STATES = {"completed", "timed_out", "error", "size_limit"}


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def _atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def _load_run_id() -> str:
    if not RUN_ID_FILE.exists():
        return ""

    run_id = RUN_ID_FILE.read_text(encoding="utf-8").strip()
    if not run_id:
        return ""

    return run_id if RUN_ID_RE.match(run_id) else ""


def _current_state_for_run(run_id: str) -> str:
    status_path = RUN_BASE / run_id / "status.json"
    if not status_path.exists():
        return ""

    try:
        payload = json.loads(status_path.read_text(encoding="utf-8"))
        state = payload.get("state", "")
        return state if isinstance(state, str) else ""
    except (json.JSONDecodeError, OSError):
        return ""


def _clear_run_id_file_if_matches(run_id: str) -> None:
    try:
        current = RUN_ID_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        return

    if current == run_id:
        RUN_ID_FILE.write_text("", encoding="utf-8")


def _cleanup_old_runs(protect_run_id: str) -> None:
    if KEEP_RUN_DIRS <= 0:
        return

    run_dirs = [p for p in RUN_BASE.iterdir() if p.is_dir()]
    if len(run_dirs) <= KEEP_RUN_DIRS:
        return

    run_dirs.sort(key=lambda p: p.stat().st_mtime)
    to_delete = len(run_dirs) - KEEP_RUN_DIRS

    removed = 0
    for path in run_dirs:
        if removed >= to_delete:
            break
        if path.name == protect_run_id:
            continue
        shutil.rmtree(path, ignore_errors=True)
        removed += 1


def _wait_for_device(deadline: float) -> str:
    while time.time() < deadline:
        devices = sorted(glob.glob(SERIAL_GLOB))
        if devices:
            return devices[0]
        time.sleep(IDLE_SLEEP_SEC)
    return ""


def _parse_line(line: str) -> tuple[dict, bool]:
    try:
        parsed = json.loads(line)
        if isinstance(parsed, dict):
            return parsed, True
        return {"payload": parsed}, True
    except json.JSONDecodeError:
        return {"raw": line}, False


def _is_done_marker(payload: dict, line: str, run_id: str) -> bool:
    if isinstance(payload, dict):
        msg = payload.get("msg", "")
        payload_run_id = payload.get("run_id", "")
        if msg == "RUN_DONE" and (not payload_run_id or payload_run_id == run_id):
            return True

    # Compatibility path for plain-text firmware logs.
    # Accepts lines such as "RUN_DONE" or "RUN_DONE run_id=<id>".
    upper_line = line.upper()
    if "RUN_DONE" not in upper_line:
        return False

    marker = f"RUN_ID={run_id}".upper()
    return ("RUN_ID=" not in upper_line) or (marker in upper_line)


def _run_once(run_id: str) -> dict:
    run_dir = RUN_BASE / run_id
    raw_log = run_dir / "serial_raw.log"
    parsed_log = run_dir / "serial_parsed.jsonl"
    status_path = run_dir / "status.json"

    run_dir.mkdir(parents=True, exist_ok=True)
    started = _utc_now()
    deadline = time.time() + RUN_TIMEOUT_SEC
    status = {
        "run_id": run_id,
        "state": "starting",
        "started_utc": started,
        "updated_utc": started,
        "ended_utc": "",
        "serial_device": "",
        "serial_baud": SERIAL_BAUD,
        "records": 0,
        "bytes": 0,
        "parse_errors": 0,
        "reconnects": 0,
        "error": "",
    }
    _atomic_write_json(status_path, status)

    with raw_log.open("w", encoding="utf-8") as raw_fh, parsed_log.open("w", encoding="utf-8") as parsed_fh:
        while time.time() < deadline:
            dev = _wait_for_device(deadline)
            if not dev:
                status["state"] = "timed_out"
                status["error"] = f"No serial device found for {SERIAL_GLOB}"
                break

            status["serial_device"] = dev
            status["state"] = "running"
            status["updated_utc"] = _utc_now()
            _atomic_write_json(status_path, status)

            try:
                with serial.Serial(dev, SERIAL_BAUD, timeout=LINE_TIMEOUT_SEC) as ser:
                    while time.time() < deadline:
                        line_bytes = ser.readline()
                        if not line_bytes:
                            continue

                        line = line_bytes.decode("utf-8", errors="replace").rstrip("\r\n")
                        pi_ts = _utc_now()

                        raw_fh.write(f"{pi_ts} {line}\n")
                        raw_fh.flush()

                        payload, ok = _parse_line(line)
                        record = {
                            "pi_ts_utc": pi_ts,
                            "run_id": run_id,
                            "payload": payload,
                        }
                        parsed_fh.write(json.dumps(record, separators=(",", ":")) + "\n")
                        parsed_fh.flush()

                        status["records"] += 1
                        status["bytes"] += len(line_bytes)
                        if not ok:
                            status["parse_errors"] += 1

                        if status["bytes"] >= MAX_RUN_BYTES:
                            status["state"] = "size_limit"
                            status["error"] = f"Run exceeded MAX_RUN_BYTES={MAX_RUN_BYTES}"
                            break

                        if status["records"] >= MAX_RUN_RECORDS:
                            status["state"] = "size_limit"
                            status["error"] = f"Run exceeded MAX_RUN_RECORDS={MAX_RUN_RECORDS}"
                            break

                        if _is_done_marker(payload, line, run_id):
                            status["state"] = "completed"
                            status["error"] = ""
                            break

                        if status["records"] % 25 == 0:
                            status["updated_utc"] = _utc_now()
                            _atomic_write_json(status_path, status)
            except serial.SerialException as exc:
                status["reconnects"] += 1
                status["updated_utc"] = _utc_now()
                status["error"] = f"Serial reconnecting after exception: {exc}"
                _atomic_write_json(status_path, status)
                time.sleep(IDLE_SLEEP_SEC)
                continue

            if status["state"] in TERMINAL_STATES:
                break

        if status["state"] == "running":
            status["state"] = "timed_out"
            status["error"] = "Run timed out waiting for RUN_DONE marker"

    if status["state"] == "running":
        status["state"] = "timed_out"
        status["error"] = "Run ended before completion marker"

    status["updated_utc"] = _utc_now()
    status["ended_utc"] = status["updated_utc"]
    _atomic_write_json(status_path, status)

    return status


def main() -> None:
    RUN_BASE.mkdir(parents=True, exist_ok=True)
    seen_run_id = ""

    while True:
        run_id = _load_run_id()
        if not run_id or run_id == seen_run_id:
            time.sleep(IDLE_SLEEP_SEC)
            continue

        # Skip replaying already-finished runs across service restarts.
        if _current_state_for_run(run_id) in TERMINAL_STATES:
            seen_run_id = run_id
            _clear_run_id_file_if_matches(run_id)
            time.sleep(IDLE_SLEEP_SEC)
            continue

        meta_path = RUN_BASE / run_id / "meta.json"
        meta = {
            "run_id": run_id,
            "listener_started_utc": _utc_now(),
            "serial_glob": SERIAL_GLOB,
            "serial_baud": SERIAL_BAUD,
            "timeout_sec": RUN_TIMEOUT_SEC,
        }
        _atomic_write_json(meta_path, meta)

        _run_once(run_id)
        seen_run_id = run_id
        _clear_run_id_file_if_matches(run_id)
        _cleanup_old_runs(protect_run_id=run_id)


if __name__ == "__main__":
    main()
