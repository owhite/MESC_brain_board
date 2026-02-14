#!/usr/bin/env python3
import sys, json, serial, math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
import numpy as np

if len(sys.argv) < 2:
    print("Usage: python torque_response.py <serial_port>")
    sys.exit(1)

port = sys.argv[1]
ser = serial.Serial(port, 115200, timeout=0.1)

PARAM_FILE = "params.json"

# --- Load parameters ---
def load_params():
    try:
        with open(PARAM_FILE, "r") as f:
            return json.load(f)
    except:
        return {
            "torque_cmd": 0.1,       # raw normalized torque value [-1..1]
            "pulse_ms": 85000
        }

params = load_params()

burst_data = None

# --- Figure with 3 subplots ---
fig, (ax_torque, ax_pos, ax_vel) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

# Torque
line_torque, = ax_torque.plot([], [], color="tab:red", label="Torque Command")
ax_torque.set_title("Torque Command vs Time")
ax_torque.set_ylabel("Torque (raw)")
ax_torque.grid(True)
ax_torque.legend()

# Position
line_pos, = ax_pos.plot([], [], color="tab:green", label="Position")
ax_pos.set_title("Position vs Time")
ax_pos.set_ylabel("Position (rad)")
ax_pos.grid(True)
ax_pos.legend()

# Velocity
line_vel, = ax_vel.plot([], [], color="tab:blue", label="Velocity")
ax_vel.set_title("Velocity vs Time")
ax_vel.set_xlabel("t (ms)")
ax_vel.set_ylabel("Velocity (rad/s)")
ax_vel.grid(True)
ax_vel.legend()

# --- UI Controls ---
axbox_torque = plt.axes([0.8, 0.82, 0.15, 0.05])
axbox_pulse  = plt.axes([0.8, 0.74, 0.15, 0.05])
axbutton     = plt.axes([0.8, 0.54, 0.15, 0.07])

tb_torque = TextBox(axbox_torque, "", initial=str(params["torque_cmd"]))
axbox_torque.set_title("Torque Cmd", fontsize=10)

tb_pulse = TextBox(axbox_pulse, "", initial=str(params["pulse_ms"]))
axbox_pulse.set_title("Pulse (ms)", fontsize=10)

button = Button(axbutton, "Run")

ax_label = plt.axes([0.8, 0.40, 0.15, 0.05])
label = ax_label.text(0.0, 0.5, "π/2 Time: --- ms", transform=ax_label.transAxes, fontsize=14)
ax_label.axis("off")

start_pos = None
pi2_time_ms = None
pi2_cross_pos = None
hline = None
vline = None

def angle_diff(a, b):
    return (a - b + math.pi) % (2*math.pi) - math.pi

def save_and_run(event):
    global start_pos, pi2_time_ms, pi2_cross_pos, hline, vline

    start_pos = None
    pi2_time_ms = None
    pi2_cross_pos = None
    label.set_text("π/2 Time: --- ms")

    if hline is not None:
        hline.remove()
        hline = None
    if vline is not None:
        vline.remove()
        vline = None

    params["torque_cmd"] = float(tb_torque.text)
    params["pulse_ms"] = int(tb_pulse.text)

    with open(PARAM_FILE, "w") as f:
        json.dump(params, f, indent=2)

    msg = {
        "cmd": "torque_reps",
        "pulse_torque": params["torque_cmd"],
        "pulse_us": params["pulse_ms"] * 1000
    }

    print("SENDING:", msg)
    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

button.on_clicked(save_and_run)

def update(frame):
    global burst_data, start_pos, pi2_time_ms, pi2_cross_pos, hline, vline

    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            if line.startswith("{") and "samples" in line:
                buf = line
                while True:
                    chunk = ser.readline().decode("utf-8", errors="ignore")
                    if not chunk:
                        break
                    buf += chunk
                    try:
                        data = json.loads(buf)
                        break
                    except json.JSONDecodeError:
                        continue
                if "samples" in data:
                    burst_data = data["samples"]
            continue
        if "cmd" in data and data["cmd"] == "PRINT":
            if "note" in data:
                print("RX:", data["note"])

    if burst_data:
        t_vals = [s["t"] * 1e-3 for s in burst_data]
        torque_vals = [(s["uL"] + s["uR"]) * 0.5 for s in burst_data]

        pos_vals = [s["pos_rad"] for s in burst_data]

        vel_vals = [s["vel_rad"] for s in burst_data]

        if start_pos is None:
            start_pos = pos_vals[0]

        if pi2_time_ms is None:
            total_dist = 0.0
            last_angle = start_pos
            for i in range(1, len(pos_vals)):
                diff = angle_diff(pos_vals[i], last_angle)
                total_dist += abs(diff)
                last_angle = pos_vals[i]

                if total_dist >= math.pi/2:
                    pi2_time_ms = t_vals[i]
                    pi2_cross_pos = pos_vals[i]
                    break

        if pi2_time_ms is not None:
            label.set_text(f"π/2 Time: {pi2_time_ms:.1f} ms")
            if hline is None and pi2_cross_pos is not None:
                hline = ax_pos.axhline(y=pi2_cross_pos, color="red", linestyle="--")
            if vline is None:
                vline = ax_pos.axvline(x=pi2_time_ms, color="red", linestyle=":")
            ax_pos.legend()

        # Update plots
        line_torque.set_data(t_vals, torque_vals)
        line_pos.set_data(t_vals, pos_vals)
        line_vel.set_data(t_vals, vel_vals)

        # Rescale axes
        for ax, vals in [(ax_torque, torque_vals), (ax_pos, pos_vals), (ax_vel, vel_vals)]:
            ymin, ymax = min(vals), max(vals)
            margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
            ax.set_xlim(min(t_vals), max(t_vals))
            ax.set_ylim(ymin - margin, ymax + margin)

        burst_data = None

    return (line_torque, line_pos, line_vel, label)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.show()
