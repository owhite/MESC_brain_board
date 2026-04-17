#!/usr/bin/env python3
"""
Lightweight serial logger for balance testing.

Behavior:
- Logs all JSON lines to file (overwrites each run).
- Prints all commands except BALANCE_TRACE.
- For BALANCE_TRACE:
  - prints one summary line when a trace starts
  - updates one in-place spinner character every 10 BALANCE_TRACE lines
  - if BALANCE_TRACE pauses for >=1s, flushes log and prints "logging complete"

Usage:
  python balance_live_log.py -p /dev/cu.usbmodemXXXX balance_diag.log
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from typing import Optional

import serial


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Log balance telemetry without plotting.")
    p.add_argument("-p", "--port", required=True, help="Serial port (e.g. /dev/cu.usbmodemXXXX)")
    p.add_argument("log_file", help="Path to diagnostics log file (overwritten each run)")
    p.add_argument("-b", "--baud", type=int, default=921600, help="Serial baud (default: 921600)")
    return p.parse_args()


def safe_json(line: str) -> Optional[dict]:
    line = line.strip()
    if not line or not line.startswith("{"):
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return None


def extract_trace_id(line: str) -> int:
    m = re.search(r'"trace_id"\s*:\s*(-?\d+)', line)
    if not m:
        return -1
    try:
        return int(m.group(1))
    except Exception:
        return -1


def main() -> int:
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}", file=sys.stderr)
        return 2

    print(f"Connected: {args.port} @ {args.baud}")
    print(f"Logging diagnostics to: {args.log_file}")

    active_trace_id: Optional[int] = None
    trace_line_count = 0
    spinner_chars = ("-", "\\", "|", "/")
    spinner_idx = 0
    spinner_active = False

    trace_stream_active = False
    last_trace_rx_s: Optional[float] = None
    logging_complete_announced = False

    def stop_spinner_line() -> None:
        nonlocal spinner_active
        if spinner_active:
            print("")
            spinner_active = False

    def flush_log(logf) -> None:
        logf.flush()
        try:
            os.fsync(logf.fileno())
        except Exception:
            pass

    with open(args.log_file, "w", encoding="utf-8", buffering=1) as logf:
        try:
            while True:
                now_s = time.monotonic()
                if trace_stream_active and last_trace_rx_s is not None:
                    if (now_s - last_trace_rx_s) >= 1.0 and not logging_complete_announced:
                        stop_spinner_line()
                        flush_log(logf)
                        print("logging complete\n")
                        logging_complete_announced = True
                        trace_stream_active = False
                        active_trace_id = None
                        trace_line_count = 0

                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("{"):
                    logf.write(line + "\n")

                # Suppress BALANCE_TRACE printing even if JSON is malformed.
                if '"cmd":"BALANCE_TRACE"' in line:
                    trace_id = extract_trace_id(line)
                    if active_trace_id != trace_id:
                        stop_spinner_line()
                        active_trace_id = trace_id
                        trace_line_count = 0
                        spinner_idx = 0
                        print(
                            f'{{"cmd":"BALANCE_TRACE_PROGRESS","trace_id":{trace_id},"status":"start"}}'
                        )

                    trace_line_count += 1
                    if (trace_line_count % 10) == 0:
                        ch = spinner_chars[spinner_idx % len(spinner_chars)]
                        spinner_idx += 1
                        print(f"\r{ch}", end="", flush=True)
                        spinner_active = True

                    trace_stream_active = True
                    last_trace_rx_s = time.monotonic()
                    logging_complete_announced = False
                    continue

                data = safe_json(line)
                stop_spinner_line()

                if data is None:
                    print(line)
                    continue

                # Any non-BALANCE_TRACE command prints as-is.
                print(line)

                cmd = data.get("cmd", "")
                if cmd == "BALANCE_TRACE_END":
                    active_trace_id = None
                    trace_line_count = 0

        except KeyboardInterrupt:
            stop_spinner_line()
            flush_log(logf)
            print("\nStopped.")
        finally:
            try:
                ser.close()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
