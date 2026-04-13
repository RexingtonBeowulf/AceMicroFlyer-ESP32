"""
Drone LQR Gain Designer
=======================
Run this on your laptop (not the ESP32) to compute the optimal LQR
gain matrix K from your drone's physical parameters.

Outputs:
  - The 2x4 K matrix
  - C++ constants to paste directly into main.cpp
  - A step response simulation plot so you can preview behaviour
    before flashing

Dependencies:
    pip install numpy scipy matplotlib

Usage:
    python3 lqr_designer.py

Then tune Q_diag and R_diag until the simulated response looks right,
and paste the printed C++ block into main.cpp.
"""

import numpy as np
from scipy.linalg import solve_discrete_are
import matplotlib.pyplot as plt

# ═══════════════════════════════════════════════════════════════
#  STEP 1 — Enter your drone's physical parameters
#  Measure or estimate these. Rough estimates are fine to start.
# ═══════════════════════════════════════════════════════════════

mass       = 0.037      # kg  — weigh the drone on a kitchen scale
arm        = 0.072      # m   — centre to motor shaft (measure with ruler)
Ixx        = 0.0000959     # kg·m²  — roll moment of inertia
                        #   estimate: Ixx ≈ 0.5 * mass * arm²
                        #   for a uniform cross: Ixx ≈ mass * arm² / 4
Iyy        = 0.0000959     # kg·m²  — pitch moment of inertia (≈ Ixx for symmetric X-frame)

# Motor / ESC thrust model:
#   thrust per motor (N) = kT * duty_fraction
#   Estimate kT by hanging drone on a scale, running one motor at 50%,
#   reading the thrust in grams, converting: kT = thrust_N / 0.5
kT         = 0.1008       # N per unit throttle fraction (0-1)

# Control loop sample time
dt         = 0.01       # seconds — 100 Hz (matches firmware)

# ═══════════════════════════════════════════════════════════════
#  STEP 2 — LQR cost matrices
#  Q penalises state error: [roll_angle, roll_rate, pitch_angle, pitch_rate]
#  R penalises control effort: [roll_correction, pitch_correction]
#
#  Tuning guide:
#    Increase Q[0,0] / Q[2,2] → tighter angle hold (more aggressive)
#    Increase Q[1,1] / Q[3,3] → damps oscillation faster
#    Increase R diagonal      → softer response, less motor wear
#    Decrease R diagonal      → more aggressive, risks oscillation
# ═══════════════════════════════════════════════════════════════

Q_diag = [80.0,   # roll angle error weight
           8.0,   # roll rate error weight
          80.0,   # pitch angle error weight
           8.0]   # pitch rate error weight

R_diag = [0.8,    # roll control effort weight
           0.8]   # pitch control effort weight

# ═══════════════════════════════════════════════════════════════
#  System matrices — linearised quadcopter dynamics
#
#  State:   x = [phi, phi_dot, theta, theta_dot]
#               roll    roll-rate  pitch  pitch-rate
#
#  Control: u = [roll_correction, pitch_correction]
#               (duty units fed to applyMix)
#
#  Continuous-time: x_dot = A_c*x + B_c*u
#  Then discretised at dt using zero-order hold.
# ═══════════════════════════════════════════════════════════════

# Continuous A — angular dynamics (roll and pitch decouple for symmetric frame)
A_c = np.array([
    [0, 1,    0, 0],   # phi_dot = phi_rate
    [0, 0,    0, 0],   # phi_ddot = B input (no coupling for symmetric frame)
    [0, 0,    0, 1],   # theta_dot = theta_rate
    [0, 0,    0, 0],   # theta_ddot = B input
])

# Torque per unit correction:
#   Two motors (FL+BL or FR+BR) produce torque τ = 2 * kT * arm * u_fraction
#   Angular accel = τ / I
tau_per_u_roll  = 2.0 * kT * arm / Ixx
tau_per_u_pitch = 2.0 * kT * arm / Iyy

# Continuous B
B_c = np.array([
    [0,                  0              ],
    [tau_per_u_roll,     0              ],
    [0,                  0              ],
    [0,                  tau_per_u_pitch],
])

# Discretise using Euler method (sufficient for dt=0.01 s)
# For higher accuracy use matrix exponential: expm(A_c * dt)
A = np.eye(4) + A_c * dt
B = B_c * dt

print("=" * 60)
print("System matrices (discrete, dt={:.3f}s)".format(dt))
print(f"A =\n{A}")
print(f"B =\n{B}")
print(f"\nPhysical params:")
print(f"  mass={mass}kg  arm={arm}m  Ixx={Ixx}  Iyy={Iyy}")
print(f"  kT={kT} N/unit  tau/u roll={tau_per_u_roll:.4f}  pitch={tau_per_u_pitch:.4f}")

# ═══════════════════════════════════════════════════════════════
#  Solve the discrete-time algebraic Riccati equation
# ═══════════════════════════════════════════════════════════════

Q = np.diag(Q_diag)
R = np.diag(R_diag)

try:
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
except np.linalg.LinAlgError as e:
    print(f"\nERROR solving Riccati equation: {e}")
    print("Try increasing R values or checking your physical parameters.")
    exit(1)

print("\n" + "=" * 60)
print("Optimal LQR gain matrix K (2 x 4):")
print(K)
print(f"\nRow 0 = roll corrections:  {K[0]}")
print(f"Row 1 = pitch corrections: {K[1]}")

# Check stability — all eigenvalues of (A - B@K) should be inside unit circle
eigs = np.linalg.eigvals(A - B @ K)
stable = all(abs(e) < 1.0 for e in eigs)
print(f"\nClosed-loop eigenvalues: {[f'{abs(e):.4f}' for e in eigs]}")
print(f"System stable: {'YES' if stable else 'NO — adjust Q/R!'}")

if not stable:
    print("\nWARNING: System is not stable with these gains.")
    print("Try reducing Q values or increasing R values.")
    exit(1)

# ═══════════════════════════════════════════════════════════════
#  Print C++ block to paste into main.cpp
# ═══════════════════════════════════════════════════════════════

print("\n" + "=" * 60)
print("// ── Paste this block into main.cpp ──────────────────")
print("// LQR gain matrix K[2][4]")
print("// Row 0: roll corrections  Row 1: pitch corrections")
print("// Columns: [phi, phi_dot, theta, theta_dot]")
print("// Generated by lqr_designer.py")
print(f"// Q={Q_diag}  R={R_diag}")
print(f"// mass={mass}kg  arm={arm}m  Ixx={Ixx}  Iyy={Iyy}  kT={kT}")
print("const float K_LQR[2][4] = {")
print(f"  {{ {K[0,0]:.6f}f, {K[0,1]:.6f}f, {K[0,2]:.6f}f, {K[0,3]:.6f}f }},  // roll")
print(f"  {{ {K[1,0]:.6f}f, {K[1,1]:.6f}f, {K[1,2]:.6f}f, {K[1,3]:.6f}f }}   // pitch")
print("};")
print("// ──────────────────────────────────────────────────────")

# ═══════════════════════════════════════════════════════════════
#  Simulate step response for preview
# ═══════════════════════════════════════════════════════════════

def simulate_step(K, A, B, step_angle_deg=15.0, duration_s=3.0):
    """Simulate a step command in roll, record roll and pitch response."""
    n = int(duration_s / dt)
    t = np.arange(n) * dt

    x = np.zeros(4)               # start at rest
    x_des = np.zeros(4)

    roll_hist  = np.zeros(n)
    pitch_hist = np.zeros(n)
    u_hist     = np.zeros((n, 2))

    step_idx = int(0.3 / dt)      # step happens at 0.3s

    for i in range(n):
        if i >= step_idx:
            x_des[0] = np.radians(step_angle_deg)   # desired roll angle

        err = x - x_des
        u   = -K @ err                               # LQR control law

        # Clamp to reasonable motor authority (±300 duty units)
        u = np.clip(u, -300, 300)

        x = A @ x + B @ u

        roll_hist[i]  = np.degrees(x[0])
        pitch_hist[i] = np.degrees(x[2])
        u_hist[i]     = u

    return t, roll_hist, pitch_hist, u_hist, np.degrees(x_des[0])

t, roll, pitch, u_out, sp = simulate_step(K, A, B, step_angle_deg=15.0)

fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
fig.patch.set_facecolor("#0a0c10")
fig.suptitle("LQR Step Response Preview  (simulate before flashing!)",
             color="#c8d8f0", fontsize=11)

for ax in axes:
    ax.set_facecolor("#111520")
    ax.tick_params(colors="#c8d8f0")
    for spine in ax.spines.values():
        spine.set_color("#1e2a40")
    ax.yaxis.label.set_color("#c8d8f0")

axes[0].axhline(sp, color="#4a5a7a", linewidth=1, linestyle="--", label="Setpoint")
axes[0].plot(t, roll,  color="#00e5ff", linewidth=2, label="Roll (commanded)")
axes[0].plot(t, pitch, color="#b060ff", linewidth=1.5, label="Pitch (coupling)")
axes[0].set_ylabel("Angle (°)")
axes[0].legend(facecolor="#111520", edgecolor="#1e2a40",
               labelcolor="#c8d8f0", fontsize=9)

axes[1].plot(t, u_out[:, 0], color="#00e676", linewidth=1.5, label="Roll correction")
axes[1].set_ylabel("Control u[0]")
axes[1].legend(facecolor="#111520", edgecolor="#1e2a40",
               labelcolor="#c8d8f0", fontsize=9)

axes[2].plot(t, u_out[:, 1], color="#ffb800", linewidth=1.5, label="Pitch correction")
axes[2].set_ylabel("Control u[1]")
axes[2].set_xlabel("Time (s)")
axes[2].legend(facecolor="#111520", edgecolor="#1e2a40",
               labelcolor="#c8d8f0", fontsize=9)

# Annotate settling time (±5% band)
band = sp * 0.05
settle_idx = len(roll) - 1
step_i = int(0.3 / dt)
for i in range(len(roll) - 1, step_i, -1):
    if abs(roll[i] - sp) > band:
        settle_idx = i + 1
        break
settle_t = t[settle_idx] if settle_idx < len(t) else t[-1]
axes[0].axvline(settle_t, color="#ffb800", linewidth=0.8, linestyle=":",
                label=f"Settle {(settle_t-0.3)*1000:.0f}ms")
axes[0].legend(facecolor="#111520", edgecolor="#1e2a40",
               labelcolor="#c8d8f0", fontsize=9)

print(f"\nSimulated settling time: {(settle_t - 0.3)*1000:.0f} ms")
print(f"Peak overshoot: {max(roll) - sp:.2f}°  ({(max(roll)-sp)/sp*100:.1f}%)")

plt.tight_layout()
plt.show()

print("\nDone. Paste the K_LQR block above into main.cpp and reflash.")