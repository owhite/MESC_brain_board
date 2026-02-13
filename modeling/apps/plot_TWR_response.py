#!/usr/bin/env python3
"""
Simulate and plot LQR closed-loop response for the balancing robot JSON.

Usage:
    ./plot_TWR_response.py path/to/LQR_output.json

Notes:
  • Simulates continuous-time closed-loop: xdot = (A - B K) x
  • Computes control torque: u(t) = -K x(t)
  • Uses params.sample_rate_Hz (if present) to choose a reasonable default dt for plotting/simulation sampling
"""

import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import StateSpace, lsim


def _require_keys(d, keys, label="JSON"):
    missing = [k for k in keys if k not in d]
    if missing:
        raise KeyError(f"{label} is missing required keys: {missing}")


def simulate_closed_loop(data, t_final=1.5, n_points=None, x0=None):
    """
    Simulate xdot = (A - B K)x and compute u = -Kx.

    If n_points is None and params.sample_rate_Hz exists, uses that rate.
    """
    _require_keys(data, ["A_cont", "B_cont", "K_cont"], label="LQR output JSON")

    A = np.array(data["A_cont"], dtype=float)
    B = np.array(data["B_cont"], dtype=float)
    K = np.array(data["K_cont"], dtype=float)

    # Closed-loop dynamics
    A_cl = A - B @ K

    # Choose simulation sampling
    params = data.get("params", {})
    sample_rate = params.get("sample_rate_Hz", None)
    if n_points is None:
        if sample_rate is not None and sample_rate > 0:
            # Match generator default rate (typically 500 Hz)
            dt = 1.0 / float(sample_rate)
            n_points = int(round(t_final / dt)) + 1
        else:
            n_points = 2000

    # Initial condition
    if x0 is None:
        x0 = np.array([0.1, 0.0, 0.0, 0.0], dtype=float)  # 0.1 rad ~ 5.7°
    else:
        x0 = np.array(x0, dtype=float).reshape(-1)

    n = A.shape[0]
    if x0.size != n:
        raise ValueError(f"x0 must have length {n}, got {x0.size}")

    # StateSpace expects: xdot = A x + B u, y = C x + D u
    # We'll drive with u=0 because control is embedded in A_cl already.
    C = np.eye(n, dtype=float)
    D = np.zeros((n, B.shape[1]), dtype=float)

    sys_cl = StateSpace(A_cl, B, C, D)

    t = np.linspace(0.0, t_final, int(n_points))
    u = np.zeros((t.size, B.shape[1]), dtype=float)

    t, y, x = lsim(sys_cl, U=u, T=t, X0=x0)

    # Control torque u_cmd = -K x
    # K is (1 x n), x is (len(t) x n)
    u_cmd = -(K @ x.T).flatten()

    return t, y, u_cmd


def plot_response(t, y, u_cmd, title="LQR Closed-Loop Response (small initial tilt)"):
    """Plot state trajectories and control torque."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)

    labels = [
        "Body angle θ (rad)",
        "Body angular rate θ̇ (rad/s)",
        "Wheel position x (m)",
        "Wheel velocity ẋ (m/s)",
    ]

    n = y.shape[1]
    for i in range(n):
        lbl = labels[i] if i < len(labels) else f"State {i}"
        ax1.plot(t, y[:, i], label=lbl)

    ax1.set_ylabel("State value")
    ax1.set_title(title)
    ax1.legend()
    ax1.grid(True)

    ax2.plot(t, u_cmd, color="black", label="Control torque u (N·m)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Torque (N·m)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    if len(sys.argv) != 2:
        print("Usage: ./plot_TWR_response.py path/to/LQR_output.json")
        sys.exit(1)

    json_path = sys.argv[1]
    with open(json_path, "r") as f:
        data = json.load(f)

    # Guard against accidentally passing the *input* assembly JSON (params+meshes only)
    if "A_cont" not in data:
        raise SystemExit(
            "This file does not appear to be the generated LQR output JSON (missing 'A_cont').\n"
            "Pass the OUTPUT JSON produced by your generator (the one containing A_cont/B_cont/K_cont)."
        )

    print(f"Loaded {json_path}")

    # Optional: embed some useful title details if present
    params = data.get("params", {})
    sr = params.get("sample_rate_Hz", None)
    l = params.get("l", None)
    title = "LQR Closed-Loop Response (small initial tilt)"
    extras = []
    if sr is not None:
        extras.append(f"{sr:.0f} Hz")
    if l is not None:
        extras.append(f"l={l:.3f} m")
    if extras:
        title += " — " + ", ".join(extras)

    t, y, u_cmd = simulate_closed_loop(data, t_final=1.5)
    plot_response(t, y, u_cmd, title=title)


if __name__ == "__main__":
    main()
