#!/usr/bin/env python3
"""
Verify the physical and control consistency of the NEW TWR LQR JSON output.

Expected JSON keys (from generate script):
  - A_cont, B_cont, A_disc, B_disc
  - K_cont, K_disc
  - params: {m_body, m_wheel, I_body, l, r, b_total, g, sample_rate_Hz}
  - (optional) eig_cont, eig_disc as [[re, im], ...]
  - (optional) com_body_mm, com_total_mm, m_total

What this script checks/reports:
  • Controllability/observability rank (4-state)
  • Open-loop & closed-loop eigenvalues (continuous and discrete)
  • Stability checks (Re(λ)<0 for continuous, |λ|<1 for discrete)
  • Characteristic time constants from discrete closed-loop poles
  • Optional: compare stored eig_* (if present) vs recomputed

Usage:
    ./verify_TWR_data.py path/to/LQR_bot_data.json
"""

import json
import numpy as np
import sys
import math


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    n = A.shape[0]
    return np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    n = A.shape[0]
    return np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(n)])


def fmt_eigs(eigs: np.ndarray) -> str:
    parts = []
    for ev in eigs:
        re = ev.real
        im = ev.imag
        if abs(im) < 1e-12:
            parts.append(f"{re:+.6g}")
        else:
            parts.append(f"{re:+.6g}{im:+.6g}j")
    return "[" + ", ".join(parts) + "]"


def pairs_to_complex(pairs):
    """Convert [[re, im], ...] to complex ndarray."""
    return np.array([complex(float(re), float(im)) for re, im in pairs], dtype=complex)


def discrete_time_constants(eigs_disc: np.ndarray, Ts: float):
    """
    For each discrete eigenvalue λ_d, map to λ_c = ln(λ_d)/Ts.
    Return (tau_s, lambda_c) where tau_s = -1/Re(λ_c) if Re(λ_c)<0 else inf.
    """
    lam_c = np.array([np.log(ev) / Ts for ev in eigs_disc], dtype=complex)
    taus = []
    for lc in lam_c:
        if lc.real < 0:
            taus.append(-1.0 / lc.real)
        else:
            taus.append(float("inf"))
    return np.array(taus, dtype=float), lam_c


def verify_twr_data(json_path: str):
    with open(json_path, "r") as f:
        data = json.load(f)

    print(f"\n🔍 Verifying TWR LQR data from: {json_path}\n")

    # --- Extract matrices ---
    A = np.array(data["A_cont"], dtype=float)
    B = np.array(data["B_cont"], dtype=float)
    A_d = np.array(data["A_disc"], dtype=float)
    B_d = np.array(data["B_disc"], dtype=float)
    K = np.array(data["K_cont"], dtype=float)
    K_d = np.array(data["K_disc"], dtype=float)

    # --- Extract params ---
    params = data.get("params", {})
    m_body = params.get("m_body", None)
    m_wheel = params.get("m_wheel", None)
    I_body = params.get("I_body", None)
    l = params.get("l", None)
    r = params.get("r", None)
    b_total = params.get("b_total", None)
    g = params.get("g", 9.81)
    sample_rate = params.get("sample_rate_Hz", None)

    # Derive Ts
    Ts = None
    if sample_rate is not None and sample_rate > 0:
        Ts = 1.0 / float(sample_rate)
    else:
        # Fallback: try to infer Ts from A_d shape? (not robust) -> assume 0.002
        Ts = 0.002

    # --- Optional: CoMs / totals ---
    com_body = data.get("com_body_mm", None)
    com_total = data.get("com_total_mm", None)
    m_total = data.get("m_total", None)

    print("📏 Physical parameters (from JSON):")
    if m_body is not None:   print(f"  m_body   = {m_body:.6g} kg")
    if m_wheel is not None:  print(f"  m_wheel  = {m_wheel:.6g} kg")
    if m_total is not None:  print(f"  m_total  = {m_total:.6g} kg")
    if I_body is not None:   print(f"  I_body   = {I_body:.6g} kg·m²")
    if l is not None:        print(f"  l        = {l:.6g} m")
    if r is not None:        print(f"  r        = {r:.6g} m")
    if b_total is not None:  print(f"  b_total  = {b_total:.6g} N·m·s/rad")
    print(f"  g        = {g:.6g} m/s²")
    print(f"  Ts       = {Ts:.6g} s  (sample_rate ≈ {1.0/Ts:.6g} Hz)")
    if com_body is not None:  print(f"  com_body_mm  = {com_body}")
    if com_total is not None: print(f"  com_total_mm = {com_total}")

    # --- Controllability / Observability ---
    print("\n🧠 Rank checks:")
    n = A.shape[0]
    C = np.eye(n)  # full-state measurement assumption
    Co = controllability_matrix(A, B)
    Ob = observability_matrix(A, C)

    rank_C = np.linalg.matrix_rank(Co)
    rank_O = np.linalg.matrix_rank(Ob)

    print(f"  Controllability rank: {rank_C}/{n}")
    print("  → " + ("System is fully controllable." if rank_C == n else "System is NOT fully controllable."))

    print(f"  Observability rank:   {rank_O}/{n}")
    print("  → " + ("System is fully observable (full-state feedback assumed)." if rank_O == n else "System is NOT fully observable."))

    # --- Eigenvalues: open-loop / closed-loop (continuous) ---
    print("\n⚙️  Continuous-time dynamics:")
    eig_ol = np.linalg.eigvals(A)
    eig_cl = np.linalg.eigvals(A - B @ K)

    print(f"  Open-loop eigenvalues:   {fmt_eigs(eig_ol)}")
    print(f"  Closed-loop eigenvalues: {fmt_eigs(eig_cl)}")
    if np.all(np.real(eig_cl) < 0):
        print("  → Closed-loop system is stable.")
    else:
        print("  → ⚠️ Closed-loop system is NOT stable (continuous).")

    # --- Eigenvalues: open-loop / closed-loop (discrete) ---
    print(f"\n⚙️  Discrete-time dynamics (Ts = {Ts:.6f} s):")
    eigd_ol = np.linalg.eigvals(A_d)
    eigd_cl = np.linalg.eigvals(A_d - B_d @ K_d)

    print(f"  Open-loop eigenvalues:   {fmt_eigs(eigd_ol)}")
    print(f"  Closed-loop eigenvalues: {fmt_eigs(eigd_cl)}")
    if np.all(np.abs(eigd_cl) < 1.0):
        print("  → Discrete-time closed-loop is stable.")
    else:
        print("  → ⚠️ Discrete-time closed-loop is NOT stable.")

    # --- Time constants from discrete closed-loop poles ---
    taus, lam_c_from_disc = discrete_time_constants(eigd_cl, Ts)
    taus_sorted = np.sort(taus[np.isfinite(taus)])

    print("\n⏱️  Characteristic time constants (s) from discrete closed-loop poles:")
    # Match screenshot-ish style: print all taus (unsorted) and a quick fast/slow summary.
    print(f"  τ (unordered): {np.array2string(taus, precision=4, separator=' ')}")
    if taus_sorted.size > 0:
        # crude grouping: "fast" = lower half, "slow" = upper half
        mid = taus_sorted.size // 2
        fast = taus_sorted[:mid] if mid > 0 else taus_sorted[:1]
        slow = taus_sorted[mid:] if mid > 0 else taus_sorted[-1:]
        fast_mean = float(np.mean(fast))
        slow_mean = float(np.mean(slow))
        print(f"  Fast modes: ~{fast_mean:.3g} s, Slow modes: ~{slow_mean:.3g} s")

    # --- Optional: compare stored eig_cont/eig_disc if present ---
    if "eig_cont" in data or "eig_disc" in data:
        print("\n🧾 Stored eigenvalues check (optional):")
        if "eig_cont" in data:
            stored = pairs_to_complex(data["eig_cont"])
            # Compare sets (order can differ). Use sorted by (real, imag).
            s1 = np.array(sorted(stored, key=lambda z: (round(z.real, 12), round(z.imag, 12))), dtype=complex)
            s2 = np.array(sorted(eig_cl,   key=lambda z: (round(z.real, 12), round(z.imag, 12))), dtype=complex)
            diff = np.max(np.abs(s1 - s2)) if s1.size == s2.size else float("inf")
            print(f"  eig_cont stored vs recomputed max |Δ| = {diff:.3e}")

        if "eig_disc" in data:
            stored = pairs_to_complex(data["eig_disc"])
            s1 = np.array(sorted(stored, key=lambda z: (round(z.real, 12), round(z.imag, 12))), dtype=complex)
            s2 = np.array(sorted(eigd_cl,  key=lambda z: (round(z.real, 12), round(z.imag, 12))), dtype=complex)
            diff = np.max(np.abs(s1 - s2)) if s1.size == s2.size else float("inf")
            print(f"  eig_disc stored vs recomputed max |Δ| = {diff:.3e}")

    # --- Summary verdict ---
    print("\n✅ Verification summary:")
    ok = True
    if rank_C != n or rank_O != n:
        ok = False
    if not np.all(np.real(eig_cl) < 0):
        ok = False
    if not np.all(np.abs(eigd_cl) < 1.0):
        ok = False

    if ok:
        print("  → Model and LQR data consistent and stable.")
    else:
        print("  → ⚠️ One or more checks failed — inspect matrices/params and JSON generation.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./verify_TWR_data.py path/to/LQR_bot_data.json")
        sys.exit(1)
    verify_twr_data(sys.argv[1])
