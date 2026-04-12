#!/usr/bin/env python3
"""
Balance trace plotter + diagnostics logger.

Primary mode:
- Reads BALANCE_TRACE_BEGIN / BALANCE_TRACE / BALANCE_TRACE_END
- Updates plots when a full trace ends (ring-buffer dump)

Backward compatible:
- If BALANCE_CTRL appears, it can still stream that live.

Usage:
  python balance_live_plot.py -p /dev/cu.usbmodemXXXX balance_diag.log
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Dict, List, Optional

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot BALANCE_TRACE telemetry and log diagnostics.")
    p.add_argument("-p", "--port", required=True, help="Serial port (e.g. /dev/cu.usbmodemXXXX)")
    p.add_argument("log_file", help="Path to diagnostics log file (overwritten each run)")
    p.add_argument("-b", "--baud", type=int, default=921600, help="Serial baud (default: 921600)")
    p.add_argument("--max-points", type=int, default=6000, help="Point limit per trace (default: 6000)")
    return p.parse_args()


def safe_json(line: str) -> Optional[Dict]:
    line = line.strip()
    if not line or not line.startswith("{"):
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return None


def append_trim(buf: List[float], v: float, max_points: int) -> None:
    buf.append(v)
    if len(buf) > max_points:
        del buf[: len(buf) - max_points]


def make_series() -> Dict[str, List[float]]:
    return {
        "t_s": [],
        "pitch_raw_rad": [],
        "theta_eq_rad": [],
        "theta_tared_rad": [],
        "theta_dot": [],
        "x_dot": [],
        "u_nm": [],
        "dt_us": [],
        "tx_hz": [],
    }


def clear_series(s: Dict[str, List[float]]) -> None:
    for v in s.values():
        v.clear()


def copy_series(dst: Dict[str, List[float]], src: Dict[str, List[float]]) -> None:
    for k in dst.keys():
        dst[k].clear()
        dst[k].extend(src[k])


def append_trace_point(s: Dict[str, List[float]], data: Dict, max_points: int) -> None:
    t_s = float(data.get("elapsed_us", 0.0)) * 1.0e-6
    tx_hz = float(data.get("tx_hz", 0.0))
    if tx_hz <= 0.0:
        tx_period_us = float(data.get("tx_period_us", 0.0))
        tx_hz = (1.0e6 / tx_period_us) if tx_period_us > 0.0 else 0.0

    append_trim(s["t_s"], t_s, max_points)
    append_trim(s["pitch_raw_rad"], float(data.get("pitch_raw_rad", 0.0)), max_points)
    append_trim(s["theta_eq_rad"], float(data.get("theta_eq_rad", 0.0)), max_points)
    append_trim(s["theta_tared_rad"], float(data.get("theta_tared_rad", 0.0)), max_points)
    append_trim(s["theta_dot"], float(data.get("theta_dot", 0.0)), max_points)
    append_trim(s["x_dot"], float(data.get("x_dot", 0.0)), max_points)
    append_trim(s["u_nm"], float(data.get("u", 0.0)), max_points)
    append_trim(s["dt_us"], float(data.get("dt_us", 0.0)), max_points)
    append_trim(s["tx_hz"], tx_hz, max_points)


def main() -> int:
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}", file=sys.stderr)
        return 2

    print(f"Connected: {args.port} @ {args.baud}")
    print(f"Logging diagnostics to: {args.log_file}")

    logf = open(args.log_file, "w", encoding="utf-8", buffering=1)

    # Displayed trace (latest completed trace or live BALANCE_CTRL fallback)
    disp = make_series()

    # Pending trace while BALANCE_TRACE lines are arriving
    pending = make_series()
    active_trace_id: Optional[int] = None
    active_trace_reason: str = ""

    # Backward-compatible BALANCE_CTRL live timebase
    ctrl_t0_us: Optional[int] = None

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    plt.subplots_adjust(hspace=0.35, right=0.97, left=0.08, top=0.95, bottom=0.08)

    # Angles
    (ln_pitch_raw,) = ax1.plot([], [], label="pitch_raw_rad")
    (ln_theta_eq,) = ax1.plot([], [], label="theta_eq_rad")
    (ln_theta_tared,) = ax1.plot([], [], label="theta_tared_rad")
    ax1.set_title("Balance Angles")
    ax1.set_ylabel("rad")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    # Rates
    (ln_theta_dot,) = ax2.plot([], [], label="theta_dot [rad/s]")
    (ln_x_dot,) = ax2.plot([], [], label="x_dot [m/s]")
    ax2.set_title("Balance Rates")
    ax2.set_ylabel("rate")
    ax2.grid(True)
    ax2.legend(loc="upper right")

    # Control
    (ln_u,) = ax3.plot([], [], label="u [Nm]")
    ax3.set_title("Control Torque")
    ax3.set_ylabel("Nm")
    ax3.set_ylim(-2.0, 2.0)
    ax3.grid(True)
    ax3.legend(loc="upper right")

    # Timing / TX
    (ln_dt,) = ax4.plot([], [], label="dt_us")
    (ln_tx_hz,) = ax4.plot([], [], label="tx_hz")
    ax4.set_title("Control Loop / TX Timing")
    ax4.set_ylabel("timing")
    ax4.set_xlabel("time [s]")
    ax4.grid(True)
    ax4.legend(loc="upper right")

    required_ctrl = (
        "t",
        "pitch_raw_rad",
        "theta_eq_rad",
        "theta_tared_rad",
        "theta_dot",
        "x_dot",
        "u",
        "dt_us",
        "tx_hz",
    )

    def redraw_lines() -> None:
        ln_pitch_raw.set_data(disp["t_s"], disp["pitch_raw_rad"])
        ln_theta_eq.set_data(disp["t_s"], disp["theta_eq_rad"])
        ln_theta_tared.set_data(disp["t_s"], disp["theta_tared_rad"])

        ln_theta_dot.set_data(disp["t_s"], disp["theta_dot"])
        ln_x_dot.set_data(disp["t_s"], disp["x_dot"])

        ln_u.set_data(disp["t_s"], disp["u_nm"])

        ln_dt.set_data(disp["t_s"], disp["dt_us"])
        ln_tx_hz.set_data(disp["t_s"], disp["tx_hz"])

        if not disp["t_s"]:
            return

        x0 = disp["t_s"][0]
        x1 = disp["t_s"][-1]
        if x1 <= x0:
            x1 = x0 + 1.0

        for ax in (ax1, ax2, ax3, ax4):
            ax.set_xlim(x0, x1)
        for ax in (ax1, ax2, ax4):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

    def update(_frame):
        nonlocal active_trace_id, active_trace_reason, ctrl_t0_us

        while ser.in_waiting:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("{"):
                logf.write(line + "\n")

            data = safe_json(line)
            if data is None:
                continue

            cmd = data.get("cmd", "")
            if cmd in ("MODE", "BALANCE_START", "BALANCE_TARE_DONE", "BALANCE_TARE_RETRY", "BALANCE_ABORT", "BALANCE_DONE"):
                print(line)

            if cmd == "BALANCE_TRACE_BEGIN":
                active_trace_id = int(data.get("trace_id", -1))
                active_trace_reason = str(data.get("reason", ""))
                clear_series(pending)
                continue

            if cmd == "BALANCE_TRACE":
                tid = int(data.get("trace_id", -1))
                if active_trace_id is None:
                    active_trace_id = tid
                    active_trace_reason = ""
                    clear_series(pending)
                if tid != active_trace_id:
                    continue
                append_trace_point(pending, data, args.max_points)
                continue

            if cmd == "BALANCE_TRACE_END":
                tid = int(data.get("trace_id", -1))
                if active_trace_id is not None and tid == active_trace_id:
                    copy_series(disp, pending)
                    redraw_lines()
                    print(
                        f"Loaded BALANCE_TRACE trace_id={tid} samples={len(disp['t_s'])} reason={active_trace_reason or data.get('reason', '')}"
                    )
                    active_trace_id = None
                    active_trace_reason = ""
                continue

            # Backward-compatible BALANCE_CTRL handling.
            if cmd == "BALANCE_CTRL":
                if not all(k in data for k in required_ctrl):
                    continue
                t_us = int(data["t"])
                if ctrl_t0_us is None:
                    ctrl_t0_us = t_us
                t_s = (t_us - ctrl_t0_us) * 1.0e-6

                append_trim(disp["t_s"], t_s, args.max_points)
                append_trim(disp["pitch_raw_rad"], float(data["pitch_raw_rad"]), args.max_points)
                append_trim(disp["theta_eq_rad"], float(data["theta_eq_rad"]), args.max_points)
                append_trim(disp["theta_tared_rad"], float(data["theta_tared_rad"]), args.max_points)
                append_trim(disp["theta_dot"], float(data["theta_dot"]), args.max_points)
                append_trim(disp["x_dot"], float(data["x_dot"]), args.max_points)
                append_trim(disp["u_nm"], float(data["u"]), args.max_points)
                append_trim(disp["dt_us"], float(data["dt_us"]), args.max_points)
                append_trim(disp["tx_hz"], float(data["tx_hz"]), args.max_points)
                redraw_lines()

        return (
            ln_pitch_raw,
            ln_theta_eq,
            ln_theta_tared,
            ln_theta_dot,
            ln_x_dot,
            ln_u,
            ln_dt,
            ln_tx_hz,
        )

    anim = animation.FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        _ = anim
        try:
            ser.close()
        except Exception:
            pass
        try:
            logf.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
