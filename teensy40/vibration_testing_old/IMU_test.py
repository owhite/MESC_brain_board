#!/usr/bin/env python3

# Test the IMU.
#  command line:
#
# $ ./IMU_test.py -p /dev/cu.usbmodem178888901
#
# Interface opens up and hit the run button

import sys, json, serial, math
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider
import numpy as np

def compute_rms(values):
    arr = np.array(values)
    return np.sqrt(np.mean(arr**2))

def send_run_command(event, ser):
    """Handle Run button click → tell Teensy to enter balance mode."""
    msg = {"cmd": "verify_angle"}
    print("TX:", msg)
    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

def main():
    parser = argparse.ArgumentParser(description="Two-Wheeled Robot live IMU plotter.")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print(f"✅ Connected to {args.port}")

    # initial y-axis range for plots
    Y_LIMIT_RANGE = 60

    # --- Setup plots (NOW 3 plots) ---
    fig, (ax_roll, ax_rate, ax_rms) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

    # IMU Roll plot -------------------------------------------
    line_roll, = ax_roll.plot([], [], color="tab:orange", label="Roll [deg]")
    ax_roll.set_title("IMU Roll Angle")
    ax_roll.set_ylabel("Angle (deg)")
    ax_roll.grid(True)
    ax_roll.legend()
    ax_roll.set_ylim(-Y_LIMIT_RANGE, Y_LIMIT_RANGE)

    # IMU Roll Rate plot ---------------------------------------
    line_rate, = ax_rate.plot([], [], color="tab:blue", label="Roll Rate [deg/s]")
    ax_rate.set_title("IMU Roll Rate")
    ax_rate.set_ylabel("Rate (deg/s)")
    ax_rate.grid(True)
    ax_rate.legend()
    ax_rate.set_ylim(-Y_LIMIT_RANGE, Y_LIMIT_RANGE)

    # RMS Plot --------------------------------------------------
    line_rms, = ax_rms.plot([], [], color="tab:green", label="Rate RMS [deg/s]")
    ax_rms.set_title("Rolling RMS (last 200 samples)")
    ax_rms.set_xlabel("Time (ms)")
    ax_rms.set_ylabel("RMS (deg/s)")
    ax_rms.grid(True)
    ax_rms.legend()
    ax_rms.set_ylim(0, 10)   # adjust as needed

    # --- Run button ---
    axbutton = plt.axes([0.8, 0.85, 0.15, 0.08])
    button = Button(axbutton, "Run")
    button.on_clicked(lambda event: send_run_command(event, ser))

    # --- Slider (0 → 360) ---
    ax_slider = plt.axes([0.8, 0.75, 0.15, 0.03])
    slider = Slider(ax_slider, "Y-Range", 0, 360, valinit=Y_LIMIT_RANGE)

    def on_slider_change(val):
        r = slider.val
        ax_roll.set_ylim(-r, r)
        ax_rate.set_ylim(-r, r)
        fig.canvas.draw_idle()

    slider.on_changed(on_slider_change)

    # --- Data buffers ---
    t_vals, pitch_vals, rate_vals, rms_vals = [], [], [], []
    max_points = 2000  # rolling window

    def update(frame):
        nonlocal t_vals, pitch_vals, rate_vals, rms_vals

        # Read serial data
        while ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8").strip()
                if not line:
                    continue
                data = json.loads(line)

                if all(k in data for k in ("t", "pitch", "pitch_rate")):
                    t_ms = (data["t"] - (t_vals[0] if t_vals else data["t"])) / 1000.0
                    t_vals.append(t_ms)
                    pitch_vals.append(data["pitch"])
                    rate_vals.append(data["pitch_rate"])

                    # Compute rolling RMS (last 200 samples)
                    if len(rate_vals) > 200:
                        rms = compute_rms(rate_vals[-200:])
                        rms_vals.append(rms)
                    else:
                        rms_vals.append(0.0)

                    # Trim buffers
                    if len(t_vals) > max_points:
                        t_vals      = t_vals[-max_points:]
                        pitch_vals   = pitch_vals[-max_points:]
                        rate_vals   = rate_vals[-max_points:]
                        rms_vals    = rms_vals[-max_points:]

            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

        # Update plots
        if t_vals:
            line_roll.set_data(t_vals, pitch_vals)
            line_rate.set_data(t_vals, rate_vals)
            line_rms.set_data(t_vals, rms_vals)

            xmin, xmax = min(t_vals), max(t_vals)
            ax_roll.set_xlim(xmin, xmax)
            ax_rate.set_xlim(xmin, xmax)
            ax_rms.set_xlim(xmin, xmax)

        return (line_roll, line_rate, line_rms)

    ani = animation.FuncAnimation(
        fig, update, interval=100, blit=False, cache_frame_data=False
    )
    plt.show()

if __name__ == "__main__":
    main()
