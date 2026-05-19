#!/usr/bin/env python3
import sys
import serial
import json
import matplotlib.pyplot as plt
import numpy as np
import argparse
import csv
import time

"""
Spin-down capture + decay fit (single sender)

- Assumes baud rate = 115200
- Uses ONLY sender 11
- Uses the Teensy-provided t_us timestamp when present
- Wait until |vel| >= START_SPEED_RAD_S, then record until |vel| <= STOP_SPEED_RAD_S
- Fits exponential: |ω(t)| ≈ ω0 * exp(-t/τ)
- Calculates and displays:
    τ [s]
    b/J = 1/τ [1/s]
    If J provided: b = J/τ [N·m·s/rad]
    Drag torque at selected speeds: τ_drag(ω) = b*ω  (or (b/J)*ω if J not provided)
    Power at selected speeds: P(ω) = b*ω^2         (or (b/J)*ω^2 per J)

Optional:
- Also fits a 2-term model for windage:
    J dω/dt = -(b ω + c ω^2)
  reporting c if fit is possible and J is provided (units: N·m·s^2/rad^2).
"""

SENDER_ID = 11
BAUD = 115200

FALLBACK_LOG_HZ = 500.0
FALLBACK_DT = 1.0 / FALLBACK_LOG_HZ

START_SPEED_RAD_S = 80.0
STOP_SPEED_RAD_S = 0.5
OMEGA_MIN_FIT_RAD_S = 5.0  # exclude low-speed region (Coulomb / quantization)

MAX_CAPTURE_SAMPLES = 2_000_000


def open_optional_text_file(path, mode="w"):
    if not path:
        return None
    return open(path, mode, encoding="utf-8", newline="")


def safe_float(x, default=None):
    try:
        if x is None:
            return default
        return float(x)
    except (TypeError, ValueError):
        return default


def fit_exponential_decay(t_s, omega_rad_s, omega_min_fit=OMEGA_MIN_FIT_RAD_S):
    """
    Fit ln(|omega|) = a + s*t over data:
      - only after peak |omega| (true spin-down)
      - only where |omega| >= omega_min_fit

    Returns: tau_s, slope_s, r2, fit_mask (mask into original arrays used for fit)
    """
    t0 = np.asarray(t_s, dtype=float)
    w0 = np.abs(np.asarray(omega_rad_s, dtype=float))

    valid = np.isfinite(t0) & np.isfinite(w0) & (w0 > 0)
    if np.count_nonzero(valid) < 30:
        return None, None, None, None

    # Work in the "valid" reduced arrays, but keep a mapping back to original indices
    idx_valid = np.where(valid)[0]
    t = t0[idx_valid]
    w = w0[idx_valid]

    # After peak only
    i_peak = int(np.argmax(w))
    idx_after_peak = idx_valid[i_peak:]   # original indices after peak
    t_ap = t0[idx_after_peak]
    w_ap = w0[idx_after_peak]

    if len(t_ap) < 30:
        return None, None, None, None

    # Fit region within after-peak
    fit_local = np.isfinite(t_ap) & np.isfinite(w_ap) & (w_ap >= float(omega_min_fit))
    if np.count_nonzero(fit_local) < 30:
        return None, None, None, None

    t_fit = t_ap[fit_local]
    w_fit = w_ap[fit_local]

    y = np.log(w_fit)

    # Linear least squares: y = a + s*t
    A = np.vstack([np.ones_like(t_fit), t_fit]).T
    (a, s), *_ = np.linalg.lstsq(A, y, rcond=None)

    # R^2
    y_hat = a + s * t_fit
    ss_res = np.sum((y - y_hat) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else None

    if s >= 0:
        return None, s, r2, None

    tau = -1.0 / s

    # Build fit_mask in original array coordinates
    fit_mask = np.zeros_like(t0, dtype=bool)
    fit_mask[idx_after_peak[fit_local]] = True

    return tau, s, r2, fit_mask


def fit_two_term_drag(t_s, omega_rad_s):
    """
    Fit dω/dt = -a ω - k ω^2  (where a = b/J and k = c/J)
    using least squares on the spin-down region.
    Returns: a, k, r2
    """
    t = np.asarray(t_s, dtype=float)
    w = np.abs(np.asarray(omega_rad_s, dtype=float))

    if len(t) < 50:
        return None, None, None

    # Use only after peak and above a threshold
    i_peak = int(np.argmax(w))
    t = t[i_peak:]
    w = w[i_peak:]
    mask = (w >= max(OMEGA_MIN_FIT_RAD_S, 5.0)) & np.isfinite(w) & np.isfinite(t)
    t = t[mask]
    w = w[mask]
    if len(t) < 50:
        return None, None, None

    # Numerical derivative (simple, but OK at 500 Hz if data is reasonably smooth)
    # Use central differences
    dt = np.diff(t)
    if np.any(dt <= 0):
        return None, None, None

    dw = np.diff(w)
    w_mid = 0.5 * (w[:-1] + w[1:])
    t_mid = 0.5 * (t[:-1] + t[1:])
    wdot = dw / dt  # dω/dt

    # Model: -wdot = a*w + k*w^2
    y = -wdot
    X = np.vstack([w_mid, w_mid**2]).T

    # Least squares
    coeff, *_ = np.linalg.lstsq(X, y, rcond=None)
    a, k = coeff[0], coeff[1]

    y_hat = X @ coeff
    ss_res = np.sum((y - y_hat) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else None

    return a, k, r2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument(
        "--J",
        type=float,
        default=None,
        help="Flywheel inertia in kg*m^2. If provided, b = J/τ and c = J*k are reported.",
    )
    parser.add_argument(
        "--raw-capture",
        action="store_true",
        help="Show raw samples as points and print sample spacing/drop diagnostics.",
    )
    parser.add_argument(
        "--raw-jsonl",
        default=None,
        help="Write every valid JSON serial line to this JSONL file with host receive time and accept/ignore reason.",
    )
    parser.add_argument(
        "--sample-csv",
        default=None,
        help="Write accepted sender velocity samples used by the capture to this CSV file.",
    )
    args = parser.parse_args()

    ser = serial.Serial(args.port, BAUD, timeout=1)
    print(f"Opened {args.port} at {BAUD} baud. Listening for sender {SENDER_ID}...")
    print("Using incoming t_us timestamps when available.")
    print(f"Fallback logging rate: {FALLBACK_LOG_HZ:.0f} Hz (dt={FALLBACK_DT:.6f} s)")
    print(f"START when |vel| >= {START_SPEED_RAD_S} rad/s")
    print(f"STOP  when |vel| <= {STOP_SPEED_RAD_S} rad/s\n")
    if args.raw_capture:
        print("Raw capture diagnostics enabled.")
    if args.raw_jsonl:
        print(f"Raw JSONL log: {args.raw_jsonl}")
    if args.sample_csv:
        print(f"Accepted sample CSV: {args.sample_csv}")

    omega = []
    t_s = []

    started = False
    sample_idx = 0
    t0_us = None
    raw_json_count = 0
    ignored_json_count = 0
    ignored_non_json_count = 0
    accepted_sender_count = 0

    raw_jsonl_f = open_optional_text_file(args.raw_jsonl)
    sample_csv_f = open_optional_text_file(args.sample_csv)
    sample_writer = None
    if sample_csv_f is not None:
        sample_writer = csv.writer(sample_csv_f)
        sample_writer.writerow(["capture_index", "host_t_s", "t_us", "t_s", "sender", "pos", "vel"])

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            host_now_s = time.monotonic()
            if not line:
                continue

            try:
                msg = json.loads(line)
                raw_json_count += 1
            except json.JSONDecodeError:
                ignored_non_json_count += 1
                continue

            ignore_reason = None
            if msg.get("sender", None) != SENDER_ID:
                ignore_reason = "sender_mismatch_or_missing"

            v = safe_float(msg.get("vel", None))
            if ignore_reason is None and v is None:
                ignore_reason = "missing_vel"

            t_us = safe_float(msg.get("t_us", None))
            p = safe_float(msg.get("pos", None))

            if raw_jsonl_f is not None:
                raw_jsonl_f.write(json.dumps({
                    "host_t_s": host_now_s,
                    "accepted_for_sender": ignore_reason is None,
                    "ignore_reason": ignore_reason,
                    "msg": msg,
                }, separators=(",", ":")) + "\n")

            if ignore_reason is not None:
                ignored_json_count += 1
                continue

            accepted_sender_count += 1

            if not started:
                if abs(v) >= START_SPEED_RAD_S:
                    started = True
                    t0_us = t_us
                    print(f"✅ START capture at sample {sample_idx} (|vel|={abs(v):.2f} rad/s)")
                else:
                    continue

            if t_us is not None and t0_us is not None:
                sample_t_s = (t_us - t0_us) * 1.0e-6
            else:
                sample_t_s = sample_idx * FALLBACK_DT

            omega.append(v)
            t_s.append(sample_t_s)
            if sample_writer is not None:
                sample_writer.writerow([sample_idx, host_now_s, t_us, sample_t_s, SENDER_ID, p, v])
            sample_idx += 1

            if sample_idx >= MAX_CAPTURE_SAMPLES:
                print("⚠️ Reached MAX_CAPTURE_SAMPLES safety cap. Stopping.")
                break

            if abs(v) <= STOP_SPEED_RAD_S:
                print(f"✅ STOP  capture at sample {sample_idx} (|vel|={abs(v):.2f} rad/s)")
                break
    finally:
        ser.close()
        if raw_jsonl_f is not None:
            raw_jsonl_f.close()
        if sample_csv_f is not None:
            sample_csv_f.close()

    print(
        "Serial JSON diagnostics: "
        f"json={raw_json_count}, accepted_sender={accepted_sender_count}, "
        f"ignored_json={ignored_json_count}, ignored_non_json={ignored_non_json_count}"
    )

    if len(omega) < 100:
        print("Not enough data captured to fit reliably. Try a longer spin-down.")
        return

    if args.raw_capture:
        t_arr_diag = np.asarray(t_s, dtype=float)
        w_arr_diag = np.asarray(omega, dtype=float)
        dt = np.diff(t_arr_diag)
        dw = np.diff(w_arr_diag)
        if len(dt) > 0:
            print(
                "Capture spacing: "
                f"n={len(w_arr_diag)}, "
                f"dt_mean={np.mean(dt) * 1000.0:.3f} ms, "
                f"dt_min={np.min(dt) * 1000.0:.3f} ms, "
                f"dt_p95={np.percentile(dt, 95) * 1000.0:.3f} ms, "
                f"dt_max={np.max(dt) * 1000.0:.3f} ms"
            )
            nonmono_count = int(np.count_nonzero(dt <= 0))
            if nonmono_count > 0:
                print(f"⚠️ Non-monotonic or duplicate sample timestamps: {nonmono_count}")
            print(
                "Velocity step diagnostics: "
                f"|dω|_p95={np.percentile(np.abs(dw), 95):.4g} rad/s, "
                f"|dω|_max={np.max(np.abs(dw)):.4g} rad/s"
            )

    # --- Exponential fit ---
    tau_s, slope, r2, fit_mask = fit_exponential_decay(t_s, omega, omega_min_fit=OMEGA_MIN_FIT_RAD_S)
    b_over_J = (1.0 / tau_s) if (tau_s is not None and tau_s > 0) else None
    b = (args.J / tau_s) if (args.J is not None and tau_s is not None and tau_s > 0) else None

    # --- Optional two-term fit (windage) ---
    a, k, r2_2term = fit_two_term_drag(t_s, omega)  # a=b/J, k=c/J
    c = (args.J * k) if (args.J is not None and k is not None) else None

    # --- Plot (velocity only) ---
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.set_title("Spin-down velocity capture (sender 11)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity ω [rad/s]")
    ax.grid(True)

    t_arr = np.asarray(t_s, dtype=float)
    w_arr = np.asarray(omega, dtype=float)

    ax.plot(t_arr, w_arr, lw=1.5, label="ω(t)")
    if args.raw_capture:
        ax.plot(t_arr, w_arr, linestyle="none", marker=".", markersize=3.0, alpha=0.55, label="raw samples")

    # Optionally highlight fit region
    if fit_mask is not None and np.any(fit_mask):
        ax.plot(t_arr[fit_mask], w_arr[fit_mask], lw=2.5, alpha=0.6, label="fit region")

    ax.legend(loc="upper right")

    # --- Results box (upper right, inside axes) ---
    lines = ["Model: |ω(t)| ≈ ω₀·exp(-t/τ)"]
    if tau_s is None:
        lines.append("Exponential fit: FAILED (τ not available)")
        if r2 is not None:
            lines.append(f"R² = {r2:.3f}")
    else:
        lines.append(f"τ = {tau_s:.4g} s")
        lines.append(f"b/J = 1/τ = {b_over_J:.4g} 1/s")
        if r2 is not None:
            lines.append(f"R² = {r2:.3f}")

        # If J is provided, compute b and give torque/power examples
        if args.J is not None:
            lines.append(f"J = {args.J:.4g} kg·m²")
            lines.append(f"b = J/τ = {b:.4g} N·m·s/rad")

            for w_test in (100.0, 50.0, 20.0):
                tau_drag = b * w_test
                p_loss = b * (w_test ** 2)
                lines.append(f"@ ω={w_test:.0f}:  τ_drag={tau_drag:.4g} N·m,  P={p_loss:.4g} W")
        else:
            # Still provide normalized torque/power per J
            for w_test in (100.0, 50.0, 20.0):
                tau_over_J = b_over_J * w_test
                p_over_J = b_over_J * (w_test ** 2)
                lines.append(f"@ ω={w_test:.0f}:  (τ/J)={tau_over_J:.4g} rad/s²,  (P/J)={p_over_J:.4g} W/kg·m²")
            lines.append("Provide J to compute b:  run with  --J <kg*m^2>")

    # Two-term fit summary (optional)
    if a is not None and k is not None:
        lines.append("")
        lines.append("Two-term fit: dω/dt = -(a ω + k ω²)")
        lines.append(f"a = {a:.4g} 1/s  (≈ b/J)")
        lines.append(f"k = {k:.4g} 1/rad  (≈ c/J)")
        if r2_2term is not None:
            lines.append(f"R²(2-term) = {r2_2term:.3f}")
        if c is not None:
            lines.append(f"c = J·k = {c:.4g} N·m·s²/rad²")
    else:
        lines.append("")
        lines.append("Two-term fit: not enough clean data / derivative too noisy")

    ax.text(
        0.98, 0.98,
        "\n".join(lines),
        transform=ax.transAxes,
        ha="right", va="top",
        fontsize=10.5,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
