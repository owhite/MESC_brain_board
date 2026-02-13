#!/usr/bin/env python3
"""
Generate LQR gains and physical parameters for a torque-controlled balancing robot.

Single-input JSON version.

Adds:
  - com_body_mm and com_total_mm outputs
  - closed-loop eigenvalues (continuous and discrete) in JSON-safe format

Eigenvalues are stored as [real, imag] pairs because JSON cannot represent complex numbers.

A note about the meaning of the input json:

include_in_COM → Does gravity act on this mass as part of the pendulum body?
include_in_body_inertia → Does this mass resist body pitch acceleration about the axle?

Some day if we used a gearbox, some elements could be mass that is physically on the body, but inertia is better modeled on actuator side, and then flags might diverge

"""

import json
import numpy as np
import argparse
import os
import trimesh
from scipy.linalg import solve_continuous_are, solve_discrete_are, expm


def complex_list_to_pairs(vals):
    """Convert complex-valued array to JSON-safe list of [real, imag] pairs."""
    return [[float(v.real), float(v.imag)] for v in vals]


def calculate_total_inertia_from_mass(
    assembly_data,
    include_inertia=True,
    include_com=True,
    respect_include_in_com=True,
):
    """Compute total mass, center of mass, and scalar moment of inertia."""
    total_mass = 0.0
    weighted_com_sum = np.zeros(3)
    total_scalar_inertia = 0.0

    for name, part in assembly_data["meshes"].items():
        mass_g = part["mass"]
        mesh_path = os.path.join(assembly_data["input_path"], name)
        mesh = trimesh.load_mesh(mesh_path)

        mesh_mass_kg = mass_g * 1e-3

        # NOTE: Preserving your current inertia approach (known fragile, but intentional for now)
        I_tensor = mesh.moment_inertia * (mesh_mass_kg / mesh.mass)
        com_mm = mesh.center_mass

        d = np.linalg.norm(com_mm)
        I_axis = (np.trace(I_tensor) / 3.0) * 1e-6 + mesh_mass_kg * (d * 1e-3) ** 2

        # --- CoM accumulation ---
        if include_com:
            if (not respect_include_in_com) or part.get("include_in_COM", True):
                total_mass += mesh_mass_kg
                weighted_com_sum += com_mm * mesh_mass_kg

        # --- inertia accumulation ---
        if include_inertia and part.get("include_in_body_inertia", True):
            total_scalar_inertia += I_axis

    total_com = weighted_com_sum / total_mass if total_mass > 0 else np.zeros(3)
    return total_mass, total_com, total_scalar_inertia


def compute_lqr_for_balancing_bot(params, m_body, I_body, com_body_mm, m_wheel, axle_origin_mm, axle_vector):
    """Compute continuous/discrete state-space model, LQR gains, and closed-loop eigenvalues."""
    g = 9.81
    axle_unit = axle_vector / np.linalg.norm(axle_vector)

    # Perpendicular distance from axle axis to COM
    r_vec = com_body_mm - axle_origin_mm
    r_perp = r_vec - np.dot(r_vec, axle_unit) * axle_unit
    l = np.linalg.norm(r_perp) * 1e-3  # mm → m

    r = params["wheel_radius"] * 1e-3  # mm → m
    b_total = params["b_Nm_s_per_rad"] + params["friction_term"]

    print("\n=== PHYSICAL PARAMETERS (MODEL) ===")
    print(f"Body mass        m_b = {m_body:.4f} kg")
    print(f"Wheel mass       m_w = {m_wheel:.4f} kg")
    print(f"Body inertia     I_b = {I_body:.4e} kg·m²")
    print(f"Body CoM (mm)    = {com_body_mm}")
    print(f"Axle origin (mm) = {axle_origin_mm}")
    print(f"Axle vector      = {axle_vector}")
    print(f"Distance l       = {l:.4f} m")
    print(f"Wheel radius r   = {r:.4f} m")
    print(f"Total damping    = {b_total:.6f} N·m·s/rad")

    # --- Continuous-time state-space model ---
    A = np.array([
        [0, 1, 0, 0],
        [(m_body + m_wheel) * g * l / (I_body + m_body * l**2),
         -b_total / (I_body + m_body * l**2), 0, 0],
        [0, 0, 0, 1],
        [-m_body * g * l / (m_wheel + m_body),
         b_total / (m_wheel + m_body), 0,
         -b_total / (m_wheel + m_body)]
    ], dtype=float)

    B = np.array([
        [0],
        [1 / (I_body + m_body * l**2)],
        [0],
        [1 / (m_wheel + m_body)]
    ], dtype=float)

    # LQR weights
    Q = np.diag([50, 1, 10, 1]).astype(float)
    R = np.array([[1.0]], dtype=float)

    # --- Continuous-time LQR ---
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P

    # Closed-loop continuous eigenvalues
    eig_cont = np.linalg.eigvals(A - B @ K)

    # --- Discretize at 500 Hz ---
    Ts = 0.002
    M = np.block([[A, B], [np.zeros((1, 5))]])
    Md = expm(M * Ts)
    A_d = Md[:4, :4]
    B_d = Md[:4, 4:5]

    # --- Discrete-time LQR ---
    P_d = solve_discrete_are(A_d, B_d, Q, R)
    K_d = np.linalg.inv(B_d.T @ P_d @ B_d + R) @ (B_d.T @ P_d @ A_d)

    # Closed-loop discrete eigenvalues
    eig_disc = np.linalg.eigvals(A_d - B_d @ K_d)

    print("\n=== EIGENVALUES (closed-loop) ===")
    print("eig_cont:", eig_cont)
    print("eig_disc:", eig_disc)

    return {
        "A_cont": A.tolist(),
        "B_cont": B.tolist(),
        "A_disc": A_d.tolist(),
        "B_disc": B_d.tolist(),
        "Q": Q.tolist(),
        "R": R.tolist(),
        "K_cont": K.tolist(),
        "K_disc": K_d.tolist(),
        # JSON-safe complex pairs
        "eig_cont": complex_list_to_pairs(eig_cont),
        "eig_disc": complex_list_to_pairs(eig_disc),
        "params": {
            "m_body": float(m_body),
            "m_wheel": float(m_wheel),
            "I_body": float(I_body),
            "l": float(l),
            "r": float(r),
            "b_total": float(b_total),
            "g": float(g),
            "sample_rate_Hz": float(1 / Ts)
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Compute LQR data for a balancing robot.")
    parser.add_argument("-i", "--input", required=True, help="Single robot JSON file")
    args = parser.parse_args()

    with open(args.input) as f:
        data = json.load(f)

    params = data["params"]
    meshes = data["meshes"]

    assembly_data = {
        "meshes": meshes,
        "input_path": params["input_path"]
    }

    # BODY CoM & inertia (model): respect include_in_COM and include_in_body_inertia
    body_mass, com_body, I_body = calculate_total_inertia_from_mass(
        assembly_data,
        include_inertia=True,
        include_com=True,
        respect_include_in_com=True,
    )

    # TOTAL CoM (physical validation): include everything regardless of include_in_COM
    total_mass, com_total, _ = calculate_total_inertia_from_mass(
        assembly_data,
        include_inertia=False,
        include_com=True,
        respect_include_in_com=False,
    )

    print("\n=== CoM SUMMARY ===")
    print(f"Body CoM  (mm) = {com_body}")
    print(f"Total CoM (mm) = {com_total}")

    # "Wheel mass" here means "excluded-from-body-inertia mass" (wheels + rotors, etc.)
    m_wheel = sum(
        part["mass"] * 1e-3
        for part in meshes.values()
        if not part.get("include_in_body_inertia", True)
    )

    axle_origin_mm = np.array(params["axle_origin"], dtype=float)
    axle_vector = np.array(params["axle_vector"], dtype=float)

    result = compute_lqr_for_balancing_bot(
        params,
        body_mass,
        I_body,
        com_body,
        m_wheel,
        axle_origin_mm,
        axle_vector
    )

    # Attach CoMs + total mass for downstream validation
    result["com_body_mm"] = com_body.tolist()
    result["com_total_mm"] = com_total.tolist()
    result["m_total"] = float(total_mass)

    out_path = os.path.join(params["input_path"], params["output_file_name"])
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)

    print(f"\n✅ Output written to {out_path}")


if __name__ == "__main__":
    main()
