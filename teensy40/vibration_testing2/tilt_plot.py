#!/usr/bin/env python3

import sys, json, serial, argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# -----------------------------
# Rolling RMS helper
# -----------------------------
def compute_rms(values):
    arr = np.array(values)
    return np.sqrt(np.mean(arr**2))


# -----------------------------
# Main
# -----------------------------
def main():
    parser = argparse.ArgumentParser(description="IMU + Control Live Plotter")
    parser.add_argument("-p", "--port", required=True,
                        help="Serial port (e.g. /dev/ttyACM0)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print(f"Connected to {args.port}")

    # Rolling window
    max_points = 2000

    # -----------------------------
    # Data buffers
    # -----------------------------
    t_vals = []

    ax_vals, ay_vals, az_vals = [], [], []
    a_mag_vals = []

    gx_vals = []            # gyro X (deg/s)
    pitch_acc_vals = []     # accel angle
    pitch_filt_vals = []    # complementary filter
    pitch_rate_vals = []    # fused rate

    u_vals = []             # torque command

    # -----------------------------
    # Matplotlib figure
    # -----------------------------
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    ax1, ax2, ax3, ax4 = axs

    # --- Plot 1: Angle estimates ---
    line_pitch_acc,  = ax1.plot([], [], label="pitch_acc [deg]", color="orange")
    line_pitch_filt, = ax1.plot([], [], label="pitch_filt [deg]", color="blue")
    ax1.set_ylabel("Angle (deg)")
    ax1.grid(True)
    ax1.legend()
    ax1.set_ylim(-120, 120)

    # --- Plot 2: Rates ---
    line_rate, = ax2.plot([], [], label="pitch_rate [deg/s]", color="green")
    line_gx,   = ax2.plot([], [], label="gyro gx [deg/s]", color="red", alpha=0.5)
    ax2.set_ylabel("Rates")
    ax2.grid(True)
    ax2.legend()
    ax2.set_ylim(-400, 400)

    # --- Plot 3: Acceleration ---
    line_ax,    = ax3.plot([], [], label="ax", color="C0")
    line_ay,    = ax3.plot([], [], label="ay", color="C1")
    line_az,    = ax3.plot([], [], label="az", color="C2")
    line_a_mag, = ax3.plot([], [], label="|a|", color="black", linewidth=2)
    ax3.set_ylabel("Accel (g)")
    ax3.grid(True)
    ax3.legend()
    ax3.set_ylim(-3, 3)

    # --- Plot 4: Torque command ---
    line_u, = ax4.plot([], [], label="u (torque)", color="purple")
    ax4.set_xlabel("Time (ms)")
    ax4.set_ylabel("Torque")
    ax4.grid(True)
    ax4.legend()
    ax4.set_ylim(-4, 4)

    # -----------------------------
    # Animation update function
    # -----------------------------
    def update(frame):
        nonlocal t_vals
        nonlocal ax_vals, ay_vals, az_vals, a_mag_vals
        nonlocal gx_vals, pitch_acc_vals, pitch_filt_vals, pitch_rate_vals
        nonlocal u_vals

        # Read all available serial messages
        while ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8").strip()
                if not line:
                    continue
                data = json.loads(line)

                # Expect fields:
                # t, ax, ay, az, gx, a_mag, pitch_acc, pitch_filt, pitch_rate, u
                if "t" not in data:
                    continue

                # Time in ms (zero-based)
                t0 = t_vals[0] if t_vals else data["t"]
                t_vals.append((data["t"] - t0) / 1000.0)

                ax_vals.append(data.get("ax", 0))
                ay_vals.append(data.get("ay", 0))
                az_vals.append(data.get("az", 0))
                a_mag_vals.append(data.get("a_mag", 0))

                gx_vals.append(data.get("gx", 0))
                pitch_acc_vals.append(data.get("pitch_acc", 0))
                pitch_filt_vals.append(data.get("pitch_filt", 0))
                pitch_rate_vals.append(data.get("pitch_rate", 0))

                u_vals.append(data.get("u", 0))

                # Trim buffers
                if len(t_vals) > max_points:
                    t_vals          = t_vals[-max_points:]
                    ax_vals         = ax_vals[-max_points:]
                    ay_vals         = ay_vals[-max_points:]
                    az_vals         = az_vals[-max_points:]
                    a_mag_vals      = a_mag_vals[-max_points:]
                    gx_vals         = gx_vals[-max_points:]
                    pitch_acc_vals  = pitch_acc_vals[-max_points:]
                    pitch_filt_vals = pitch_filt_vals[-max_points:]
                    pitch_rate_vals = pitch_rate_vals[-max_points:]
                    u_vals          = u_vals[-max_points:]

            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

        if not t_vals:
            return []

        # Update plot data
        line_pitch_acc.set_data(t_vals, pitch_acc_vals)
        line_pitch_filt.set_data(t_vals, pitch_filt_vals)

        line_rate.set_data(t_vals, pitch_rate_vals)
        line_gx.set_data(t_vals, gx_vals)

        line_ax.set_data(t_vals, ax_vals)
        line_ay.set_data(t_vals, ay_vals)
        line_az.set_data(t_vals, az_vals)
        line_a_mag.set_data(t_vals, a_mag_vals)

        line_u.set_data(t_vals, u_vals)

        # Rescale x-axis
        xmin, xmax = min(t_vals), max(t_vals)
        for ax in axs:
            ax.set_xlim(xmin, xmax)

        return [
            line_pitch_acc, line_pitch_filt,
            line_rate, line_gx,
            line_ax, line_ay, line_az, line_a_mag,
            line_u
        ]

    # -----------------------------
    # Start animation loop
    # -----------------------------
    ani = animation.FuncAnimation(
        fig, update, interval=50, blit=False, cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
