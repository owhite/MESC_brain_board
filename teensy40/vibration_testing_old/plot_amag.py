#!/usr/bin/env python3
import sys, json, serial, argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation

"""
Load this into supervisor controlLoop()

    Serial.printf(
		  "{\"t\":%lu,"
		  "\"a_mag\":%.3f,"
		  "\"accelValid\":%d,"
		  "\"pitch_rad\":%.3f,"
		  "\"pitch_rate_raw\":%.3f,"
		  "\"pitch_rate_filt\":%.3f}\r\n",
		  micros(),
		  a_mag,
		  accelValid ? 1 : 0,
		  pitch_rad * 180.0f / PI,
		  pitch_rate_raw * 180.0f / PI,
		  pitch_rate * 180.0f / PI
		  );

"""

def main():
    parser = argparse.ArgumentParser(description="Plot IMU debug JSON signals")
    parser.add_argument("-p", "--port", required=True, help="Serial port")
    args = parser.parse_args()

    # Open serial port
    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print(f"Connected to {args.port}")

    # Matplotlib setup: 6 plots
    fig, axes = plt.subplots(5, 1, figsize=(10,10), sharex=True)
    titles = [
        "a_mag",
        "accelValid",
        "pitch_rad (deg)",
        "pitch_rate_raw (deg/s)",
        "pitch_rate_filt (deg/s)"
    ]

    lines = []
    buffers = []
    WINDOW = 200

    for ax, title in zip(axes, titles):
        ax.set_title(title)
        ax.grid(True)
        line, = ax.plot([], [])
        lines.append(line)
        buffers.append([])

    # Mapping from JSON keys → buffer index
    key_map = {
        "a_mag": 0,
        "accelValid": 1,
        "pitch_rad": 2,
        "pitch_rate_raw": 3,
        "pitch_rate_filt": 4
    }

    def update(frame):
        nonlocal buffers

        # Read all available serial lines
        while ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8").strip()
                if not line:
                    continue

                data = json.loads(line)

                # Store values
                for key, idx in key_map.items():
                    if key in data:
                        buffers[idx].append(data[key])
                        # retain last WINDOW samples
                        if len(buffers[idx]) > WINDOW:
                            buffers[idx] = buffers[idx][-WINDOW:]

            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

        # Update each graph
        for idx, line in enumerate(lines):
            buf = buffers[idx]
            if len(buf) > 1:
                x = list(range(len(buf)))
                y = buf
                line.set_data(x, y)
                axes[idx].set_xlim(0, WINDOW-1)
                ymin = min(y)
                ymax = max(y)
                if ymin == ymax:
                    ymin -= 0.1
                    ymax += 0.1
                elif idx == 0:
                    axes[idx].set_ylim(-0.2, 4.2)
                elif idx == 1:
                    axes[idx].set_ylim(-0.2, 1.2)
                elif idx == 2:
                    axes[idx].set_ylim(-30, 30)
                elif idx == 3:
                    axes[idx].set_ylim(-30, 30)
                elif idx == 4:
                    axes[idx].set_ylim(-30, 30)
                else:
                    axes[idx].set_ylim(ymin - 0.1*abs(ymin), ymax + 0.1*abs(ymax))

        return lines

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False
    )

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
