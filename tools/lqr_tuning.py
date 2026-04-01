#!/usr/bin/env python3
"""
Offline LQR gain computation for RC car steering control.

Kinematic bicycle model — 3-state linearization
================================================
States:  x = [e_lat (m), theta_e (rad), psi_dot (rad/s)]
Input:   u = delta (rad)   — steering angle, positive = left turn

Sign convention (matches standard path-following literature):
  e_lat   > 0  →  car is to the LEFT of the lane centre
  theta_e > 0  →  car heading is CCW (left) of lane direction
  psi_dot > 0  →  yaw rate CCW (turning left)
  delta   > 0  →  steer left

Discrete-time model (forward-Euler, step dt):
  e_lat[k+1]    = e_lat[k]  + v*dt * theta_e[k]
  theta_e[k+1]  = theta_e[k] + dt  * psi_dot[k]
  psi_dot[k+1]  = (v/L)      * delta[k]

  A = [[1,  v*dt,  0 ],       B = [[0   ],
       [0,  1,     dt],            [0   ],
       [0,  0,     0 ]]            [v/L ]]

The DARE is solved with scipy.linalg.solve_discrete_are (not the iterative
approach used in PythonRobotics) which gives a numerically exact solution.

Usage examples
--------------
  # Default Bryson weights, v=1.5 m/s, dt=50 ms
  python lqr_tuning.py

  # Custom velocity and timestep
  python lqr_tuning.py --v 2.0 --dt 0.04

  # Override individual Q weights
  python lqr_tuning.py --q1 200 --q2 5.0 --q3 0.1 --r 4.0

  # Skip the plot (CI / headless use)
  python lqr_tuning.py --no-plot
"""

import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg

# ---------------------------------------------------------------------------
# Vehicle constants
# ---------------------------------------------------------------------------
L         = 0.20          # wheelbase [m]  — RC car scale
MAX_STEER = np.deg2rad(30.0)  # physical servo limit [rad]


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_model(v: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Return (A, B) for the discrete 3-state bicycle model."""
    A = np.array([
        [1.0,  v * dt,  0.0],
        [0.0,  1.0,     dt ],
        [0.0,  0.0,     0.0],
    ])
    B = np.array([[0.0],
                  [0.0],
                  [v / L]])
    return A, B


def check_controllability(A: np.ndarray, B: np.ndarray) -> int:
    n = A.shape[0]
    C = B.copy()
    Ai = np.eye(n)
    for _ in range(n - 1):
        Ai = A @ Ai
        C = np.hstack([C, Ai @ B])
    return np.linalg.matrix_rank(C)


# ---------------------------------------------------------------------------
# LQR solver
# ---------------------------------------------------------------------------
def solve_lqr(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Solve the discrete-time LQR problem.

    Uses scipy.linalg.solve_discrete_are which solves the DARE directly
    (no iteration, unlike the PythonRobotics helper).

    Returns
    -------
    K  : (1, n) gain matrix  — u = -K @ x
    P  : (n, n) solution to the DARE
    """
    P = scipy.linalg.solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K, P


# ---------------------------------------------------------------------------
# Simulation helpers
# ---------------------------------------------------------------------------
def simulate_cl(
    A_cl: np.ndarray,
    K: np.ndarray,
    x0: np.ndarray,
    steps: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Simulate closed-loop from x0; return (state_traj, control_traj)."""
    n = A_cl.shape[0]
    states  = np.zeros((steps + 1, n))
    controls = np.zeros(steps)
    states[0] = x0
    for k in range(steps):
        u = -(K @ states[k])
        states[k + 1] = A_cl @ states[k]
        controls[k] = u[0]
    return states, controls


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def print_c_literal(K: np.ndarray, v: float, dt: float,
                    Q: np.ndarray, R: np.ndarray) -> None:
    """Print the gain matrix as a C array literal ready to paste."""
    separator = "=" * 64
    print(f"\n{separator}")
    print("/* ---- paste into source/control_algorithms.c ----")
    print(f" * LQR gain for kinematic bicycle model")
    print(f" * L = {L} m,  v0 = {v} m/s,  dt = {dt} s")
    print(f" * Q = diag({Q[0,0]:.6g}, {Q[1,1]:.6g}, {Q[2,2]:.6g})")
    print(f" * R = {R[0,0]:.6g}")
    print(f" * States: [e_lat (m), theta_e (rad), psi_dot (rad/s)]")
    print(f" * Control: delta [rad], positive = steer left")
    print(" */")
    print("static const float LQR_K[LQR_NUM_STATES] = {")
    print(f"    {K[0,0]:>14.8f}f,  /* k_e_lat   [rad/m]      */")
    print(f"    {K[0,1]:>14.8f}f,  /* k_theta_e [rad/rad]    */")
    print(f"    {K[0,2]:>14.8f}f   /* k_psi_dot [rad/(rad/s)]*/")
    print("};")
    print(separator + "\n")


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
def plot_results(A: np.ndarray, B: np.ndarray, K: np.ndarray,
                 dt: float, v: float) -> None:
    A_cl = A - B @ K
    eigs = np.linalg.eigvals(A_cl)
    stable = all(abs(e) < 1.0 for e in eigs)

    STEPS = 80
    t = np.arange(STEPS + 1) * dt

    ics = [
        (np.array([0.10, 0.0,  0.0]),  "e_lat=0.10 m",    "tab:blue"),
        (np.array([0.0,  0.20, 0.0]),  "theta_e=0.20 rad", "tab:orange"),
        (np.array([0.05, 0.10, 0.0]),  "combined",         "tab:green"),
    ]

    state_labels = ["e_lat [m]", "θ_e [rad]", "ψ̇ [rad/s]"]

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(
        f"LQR Closed-Loop Step Response\n"
        f"L={L} m,  v₀={v:.2f} m/s,  dt={dt} s,  "
        f"stable={stable},  |λ|={np.round(sorted(np.abs(eigs), reverse=True), 3)}"
    )

    for x0, label, color in ics:
        states, controls = simulate_cl(A_cl, K, x0, STEPS)
        for i in range(3):
            axes[i].plot(t, states[:, i], color=color, label=label)
        axes[3].plot(t[:-1], np.rad2deg(controls),
                     color=color, label=label)

    for i, ax in enumerate(axes[:3]):
        ax.set_ylabel(state_labels[i])
        ax.axhline(0, color="k", lw=0.6)
        ax.grid(True, alpha=0.4)
        ax.legend(fontsize=8, loc="upper right")

    axes[3].set_ylabel("δ [deg]")
    axes[3].axhline( np.rad2deg( MAX_STEER), color="r", lw=0.8,
                     linestyle="--", label="±limit")
    axes[3].axhline(-np.rad2deg( MAX_STEER), color="r", lw=0.8,
                     linestyle="--")
    axes[3].grid(True, alpha=0.4)
    axes[3].legend(fontsize=8, loc="upper right")
    axes[3].set_xlabel("Time [s]")

    plt.tight_layout()

    # Pole-zero map
    fig2, ax2 = plt.subplots(figsize=(5, 5))
    theta = np.linspace(0, 2 * np.pi, 300)
    ax2.plot(np.cos(theta), np.sin(theta), "k--", lw=0.7, label="unit circle")
    ax2.scatter(eigs.real, eigs.imag, s=80, zorder=5, label="CL poles")
    ax2.set_xlabel("Re")
    ax2.set_ylabel("Im")
    ax2.set_title("Closed-loop pole locations")
    ax2.axis("equal")
    ax2.grid(True, alpha=0.4)
    ax2.legend()
    plt.tight_layout()

    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Compute LQR gains for RC car steering control."
    )
    p.add_argument("--v",       type=float, default=1.5,
                   help="Nominal forward velocity [m/s]  (default: 1.5)")
    p.add_argument("--dt",      type=float, default=0.05,
                   help="Control timestep [s]  (default: 0.05)")
    p.add_argument("--q1",      type=float, default=None,
                   help="Q weight for e_lat   (default: Bryson 1/e_lat_max²)")
    p.add_argument("--q2",      type=float, default=None,
                   help="Q weight for theta_e (default: Bryson 1/theta_max²)")
    p.add_argument("--q3",      type=float, default=None,
                   help="Q weight for psi_dot (default: Bryson 1/psi_dot_max²)")
    p.add_argument("--r",       type=float, default=None,
                   help="R weight for delta   (default: Bryson 1/delta_max²)")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip the matplotlib plots")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    v  = args.v
    dt = args.dt

    # --- Bryson's method defaults -----------------------------------------
    # Normalise each state and input by its maximum expected value.
    e_lat_max   = 0.10                      # 10 cm lateral deviation
    theta_e_max = np.deg2rad(30.0)          # 30° heading error
    psi_dot_max = v / L * MAX_STEER         # yaw rate at max steer + v
    delta_max   = MAX_STEER

    q1 = args.q1 if args.q1 is not None else 1.0 / e_lat_max**2
    q2 = args.q2 if args.q2 is not None else 1.0 / theta_e_max**2
    q3 = args.q3 if args.q3 is not None else 1.0 / psi_dot_max**2
    r  = args.r  if args.r  is not None else 1.0 / delta_max**2

    Q = np.diag([q1, q2, q3])
    R = np.array([[r]])

    # --- Build model -------------------------------------------------------
    A, B = build_model(v, dt)

    print(f"\nVehicle:  L = {L} m,  v₀ = {v} m/s,  dt = {dt} s")
    print(f"max_steer = {np.rad2deg(MAX_STEER):.1f} °")
    print(f"\nQ = diag({q1:.4g}, {q2:.4g}, {q3:.4g})")
    print(f"R = {r:.4g}")
    print(f"\nA =\n{A}")
    print(f"B =\n{B}")

    # --- Controllability ---------------------------------------------------
    rank = check_controllability(A, B)
    ok = rank == A.shape[0]
    print(f"\nControllability rank: {rank}/{A.shape[0]}  {'✓' if ok else '✗ NOT CONTROLLABLE — ABORT'}")
    if not ok:
        sys.exit(1)

    # --- Solve LQR ---------------------------------------------------------
    K, P = solve_lqr(A, B, Q, R)

    eigs_ol = np.linalg.eigvals(A)
    eigs_cl = np.linalg.eigvals(A - B @ K)
    stable  = all(abs(e) < 1.0 for e in eigs_cl)

    print(f"\nOpen-loop  eigenvalue magnitudes : {np.sort(np.abs(eigs_ol))[::-1]}")
    print(f"Closed-loop eigenvalue magnitudes: {np.sort(np.abs(eigs_cl))[::-1]}")
    print(f"System stable: {stable}")
    print(f"\nK = {K}")

    # --- C literal ---------------------------------------------------------
    print_c_literal(K, v, dt, Q, R)

    # --- Plots -------------------------------------------------------------
    if not args.no_plot:
        plot_results(A, B, K, dt, v)
