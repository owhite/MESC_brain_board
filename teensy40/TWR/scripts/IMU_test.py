#!/usr/bin/env python3
"""
Live IMU plotter for Teensy 4.0 telemetry JSON.

Expected JSON line format (new output):
{"t":..., "pitch":..., "rate":..., "rms_raw":..., "rms_filt":..., "n_rms":..., "age_us":..., "valid":..., "exec_us":..., "ovr":...}

Units assumed:
- pitch: degrees
- rate: degrees/second
- rms_raw: radians/second   (gyro raw RMS)
- rms_filt: radians/second  (filtered RMS)
- age_us: microseconds
- exec_us: microseconds
"""

import json
import argparse
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider, RadioButtons

RAD2DEG = 180.0 / np.pi


def send_run_command(_event, ser, motor_enabled, motor_state):
    """Send either verify-angle or motor-spin command based on toggle state."""
    if bool(motor_enabled.get("enabled", False)):
        cmd = "motor\r\n"  # firmware toggles motor ON/OFF each time this command is sent
        motor_state["running"] = not bool(motor_state.get("running", False))
        expected = "ON" if motor_state["running"] else "OFF"
    else:
        cmd = "verify_angle\r\n"
        motor_state["running"] = False
        expected = "OFF"
    ser.write(cmd.encode("utf-8"))
    print(f"TX: {cmd.strip()} (expected motor {expected})")


def main():
    parser = argparse.ArgumentParser(description="Balancing robot IMU live plotter (Teensy JSON).")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g. /dev/cu.usbmodemXXXX)")
    parser.add_argument("-b", "--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--max-points", type=int, default=4000, help="Rolling plot buffer length (default: 4000)")
    parser.add_argument("--expect-n", type=int, default=200, help="Expected n_rms (default: 200)")
    parser.add_argument("--rms-ymax", type=float, default=5.0, help="RMS plot Y max (deg/s)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print(f"✅ Connected to {args.port} @ {args.baud} baud")

    # Initial axis range
    Y_LIMIT_RANGE = 60

    # ---- 3 plots: pitch, rate, rms (raw+filt) ----
    fig, (ax_pitch, ax_rate, ax_rms) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

    # Pitch plot
    (line_pitch,) = ax_pitch.plot([], [], label="Pitch [deg]")
    ax_pitch.set_title("IMU Pitch Angle")
    ax_pitch.set_ylabel("deg")
    ax_pitch.grid(True)
    ax_pitch.legend()
    ax_pitch.set_ylim(-Y_LIMIT_RANGE, Y_LIMIT_RANGE)

    # Rate plot
    (line_rate,) = ax_rate.plot([], [], label="Pitch Rate [deg/s]")
    ax_rate.set_title("IMU Pitch Rate")
    ax_rate.set_ylabel("deg/s")
    ax_rate.grid(True)
    ax_rate.legend()
    ax_rate.set_ylim(-Y_LIMIT_RANGE, Y_LIMIT_RANGE)

    # RMS plot (deg/s): BOTH raw and filtered
    (line_rms_raw,) = ax_rms.plot([], [], label="RMS Raw [deg/s]")
    (line_rms_filt,) = ax_rms.plot([], [], label="RMS Filtered [deg/s]")
    ax_rms.set_title("Pitch Rate RMS (Raw vs Filtered) (from Teensy)")
    ax_rms.set_xlabel("Time (s)")
    ax_rms.set_ylabel("deg/s")
    ax_rms.grid(True)
    ax_rms.legend()
    ax_rms.set_ylim(0, args.rms_ymax)

    # ---- Y-range slider (for pitch & rate) ----
    ax_slider = plt.axes([0.8, 0.75, 0.15, 0.03])
    slider = Slider(ax_slider, "Y-Range", 0, 360, valinit=Y_LIMIT_RANGE)

    def on_slider_change(_val):
        r = slider.val
        ax_pitch.set_ylim(-r, r)
        ax_rate.set_ylim(-r, r)
        fig.canvas.draw_idle()

    slider.on_changed(on_slider_change)

    # ---- Data buffers ----
    t_vals = []
    pitch_vals = []
    rate_vals = []
    rms_raw_vals = []
    rms_filt_vals = []
    motor_enabled = {"enabled": False}
    motor_state = {"running": False}

    # ---- Run button ----
    axbutton = plt.axes([0.8, 0.85, 0.15, 0.08])
    button = Button(axbutton, "Run")
    # ---- Motor mode selector (persistent ON/OFF state) ----
    ax_motor_mode = plt.axes([0.77, 0.54, 0.20, 0.14])
    motor_mode = RadioButtons(ax_motor_mode, ("MOTOR OFF", "MOTOR ON"), active=0)
    for lbl in motor_mode.labels:
        lbl.set_fontsize(12)

    def on_motor_mode_change(label):
        motor_enabled["enabled"] = (label == "MOTOR ON")

    motor_mode.on_clicked(on_motor_mode_change)
    button.on_clicked(lambda event: send_run_command(event, ser, motor_enabled, motor_state))
    max_points = int(args.max_points)
    t0_us = None

    def try_parse_json_line(line: str):
        """Return dict if valid JSON object, else None."""
        line = line.strip()
        if not line:
            return None
        if not line.startswith("{"):
            return None
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return None

    def update(_frame):
        nonlocal t0_us, t_vals, pitch_vals, rate_vals, rms_raw_vals, rms_filt_vals

        while ser.in_waiting:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore")
            data = try_parse_json_line(line)
            if data is None:
                continue

            # New firmware verify-angle payload:
            # {"cmd":"VERIFY_ANGLE","t":...,"pitch_deg":...,"pitch_rate_deg_s":...,
            #  "rms_raw_rad_s":...,"rms_filt_rad_s":...,"rms_n":...,"imu_valid":...}
            if data.get("cmd") != "VERIFY_ANGLE":
                continue
            required = ("t", "pitch_deg", "pitch_rate_deg_s",
                        "rms_raw_rad_s", "rms_filt_rad_s", "rms_n", "imu_valid")
            if not all(k in data for k in required):
                continue
            if int(data["imu_valid"]) != 1:
                continue

            t_us = int(data["t"])
            if t0_us is None:
                t0_us = t_us
            t_s = (t_us - t0_us) * 1e-6

            pitch_deg = float(data["pitch_deg"])          # already deg
            rate_deg_s = float(data["pitch_rate_deg_s"])  # already deg/s
            rms_raw_deg_s = float(data["rms_raw_rad_s"]) * RAD2DEG
            rms_filt_deg_s = float(data["rms_filt_rad_s"]) * RAD2DEG
            n_rms = int(data["rms_n"])
            if "motor" in data:
                motor_state["running"] = (int(data["motor"]) == 1)

            if args.expect_n and n_rms != int(args.expect_n):
                print(f"⚠️ n_rms={n_rms} (expected ~{args.expect_n}) at t={t_s:.3f}s")

            t_vals.append(t_s)
            pitch_vals.append(pitch_deg)
            rate_vals.append(rate_deg_s)
            rms_raw_vals.append(rms_raw_deg_s)
            rms_filt_vals.append(rms_filt_deg_s)

            if len(t_vals) > max_points:
                t_vals = t_vals[-max_points:]
                pitch_vals = pitch_vals[-max_points:]
                rate_vals = rate_vals[-max_points:]
                rms_raw_vals = rms_raw_vals[-max_points:]
                rms_filt_vals = rms_filt_vals[-max_points:]

        if t_vals:
            line_pitch.set_data(t_vals, pitch_vals)
            line_rate.set_data(t_vals, rate_vals)
            line_rms_raw.set_data(t_vals, rms_raw_vals)
            line_rms_filt.set_data(t_vals, rms_filt_vals)

            ax_pitch.set_xlim(t_vals[0], t_vals[-1])
            ax_rate.set_xlim(t_vals[0], t_vals[-1])
            ax_rms.set_xlim(t_vals[0], t_vals[-1])

        return (line_pitch, line_rate, line_rms_raw, line_rms_filt)

    # Keep strong reference to avoid "Animation was deleted..." warning
    anim = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

    plt.show()


if __name__ == "__main__":
    main()
