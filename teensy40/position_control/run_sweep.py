#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing dependency: pyserial. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)

PORT = "/dev/cu.usbmodem138829401"
BAUD = 115200
READ_TIMEOUT_S = 0.20
SESSION_TIMEOUT_S = 15.0
OUTFILE = Path(__file__).resolve().parent / "sweep_output.log"
# Match macOS `screen` behavior: send carriage return only.
LINE_ENDING = "\r"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one Teensy sweep and capture serial output to sweep_output.log"
    )
    parser.add_argument("sweep_deg", type=int, help="timed_sweep command in degrees (-10000..10000)")
    args = parser.parse_args()

    if args.sweep_deg < -10000 or args.sweep_deg > 10000:
        parser.error("sweep_deg must be in range -10000..10000")
    return args


def is_terminal_line(line: str) -> bool:
    return '"cmd":"TIMED_POS_DONE"' in line or '"cmd":"TIMED_POS_FAULT"' in line


def now_ms() -> int:
    return int(time.monotonic() * 1000.0)


def log_line(lines: list[str], text: str, echo: bool = True) -> None:
    lines.append(text)
    if echo:
        print(text, flush=True)


def main() -> int:
    args = parse_args()
    cmd_body = f"timed_sweep {args.sweep_deg}"
    cmd = f"{cmd_body}{LINE_ENDING}"
    cmd_bytes = cmd.encode("utf-8")

    lines: list[str] = []
    start_wall = time.strftime("%Y-%m-%d %H:%M:%S")
    log_line(lines, f"# started: {start_wall}")
    log_line(lines, f"# port: {PORT}")
    log_line(lines, f"# baud: {BAUD}")
    log_line(lines, f"# command: {cmd_body}")
    log_line(lines, f"# line_ending_repr: {LINE_ENDING.encode('unicode_escape').decode('ascii')}")
    ports = list(list_ports.comports())
    if ports:
        log_line(lines, "# detected_ports:")
        for p in ports:
            mark = "*" if p.device == PORT else " "
            log_line(lines, f"#  {mark} {p.device}  {p.description}  hwid={p.hwid}")
    else:
        log_line(lines, "# detected_ports: none")

    try:
        with serial.Serial(PORT, BAUD, timeout=READ_TIMEOUT_S, write_timeout=1) as ser:
            log_line(lines, f"# opened: name={ser.name} is_open={ser.is_open} timeout={ser.timeout}")
            log_line(lines, f"# line_status: dtr={ser.dtr} rts={ser.rts}")

            # Opening the Teensy serial port can reset it; allow boot prints to appear.
            time.sleep(1.0)
            log_line(lines, "# settle_after_open_s: 1.0")

            pre_start_ms = now_ms()
            pre_lines = 0
            while (now_ms() - pre_start_ms) < 600:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                pre_lines += 1
                log_line(lines, f"PRE {line}")
            log_line(lines, f"# pre_read_lines: {pre_lines}")

            ser.reset_input_buffer()
            ser.reset_output_buffer()
            log_line(lines, "# buffers: input/output reset")

            n_written = ser.write(cmd_bytes)
            ser.flush()
            log_line(lines, f"# tx: wrote_bytes={n_written} expected_bytes={len(cmd_bytes)}")
            log_line(lines, f"# tx_hex: {cmd_bytes.hex()}")

            log_line(lines, f"# sent: {cmd_body} on {PORT}")

            t0 = time.monotonic()
            saw_start = False
            saw_start_banner = False
            saw_terminal = False
            rx_lines = 0

            while (time.monotonic() - t0) < SESSION_TIMEOUT_S:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                rx_lines += 1
                dt_ms = int((time.monotonic() - t0) * 1000.0)
                log_line(lines, f"RX +{dt_ms:05d}ms {line}")

                if "starting timed_pos_control mode with timed_sweep" in line:
                    saw_start_banner = True

                if '"cmd":"TIMED_POS_START"' in line:
                    saw_start = True

                if saw_start and is_terminal_line(line):
                    saw_terminal = True
                    break

            log_line(lines, f"# rx_lines: {rx_lines}")
            log_line(lines, f"# saw_start_banner: {int(saw_start_banner)}")
            log_line(lines, f"# saw_timed_pos_start: {int(saw_start)}")
            log_line(lines, f"# saw_terminal: {int(saw_terminal)}")

            if not saw_start:
                log_line(lines, "# warning: no TIMED_POS_START seen")
                print("warning: no TIMED_POS_START seen before timeout", file=sys.stderr, flush=True)
            elif not saw_terminal:
                log_line(lines, "# warning: no TIMED_POS_DONE/TIMED_POS_FAULT seen")
                print("warning: run timed out before TIMED_POS_DONE/TIMED_POS_FAULT", file=sys.stderr, flush=True)

    except serial.SerialException as exc:
        log_line(lines, f"# serial_error: {exc}")
        OUTFILE.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"serial error: {exc}", file=sys.stderr)
        return 2

    OUTFILE.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote: {OUTFILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
