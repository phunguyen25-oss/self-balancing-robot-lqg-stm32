#!/usr/bin/env python3
"""
lqg_design.py -- offline LQG design tool for the 2-wheel self-balancing robot.

This is the Python re-implementation of the 10-step design pipeline from
"vidu LQG xe can bang.docx" (Buoc 1..9). It replaces the "chay 1 lan, ngoai
tuyen tren MATLAB" step: you run this on your PC, it prints Ad, Bd, Cd, Kd,
Ld ready to paste into Core/Inc/config.h. The STM32 NEVER solves Riccati /
does ctrb / obsv / lqr on-line -- it only multiplies these 5 fixed matrices
every 5 ms (see Core/Src/estimator.c and control.c).

WHEN YOU HAVE REAL MEASURED PARAMETERS:
    Replace M, m, l, I, b below with your measured values (see README.md
    "Do thong so that" section for how to measure each one), then re-run:

        python3 lqg_design.py

    Paste the printed AD/BD/CD/KD/LD arrays into config.h, replacing the
    placeholder block. Nothing else in the firmware needs to change --
    the estimator/control code is written generically against these 5
    matrices plus NUM_STATES=4 / NUM_OUTPUTS=2.

Requires: numpy, scipy, control  (pip install numpy scipy control)
"""
import numpy as np
import control as ct

# =========================================================================
# 1) PHYSICAL PARAMETERS -- replace with YOUR measured values.
#    The numbers below are the doc's illustrative example, NOT your robot.
# =========================================================================
M = 0.2      # kg      -- wheel + drivetrain (rotating) mass reflected to cart
m = 0.8      # kg      -- body mass (frame + battery + boards), above the axle
l = 0.10     # m       -- body CG distance from wheel axle
I = 0.005    # kg*m^2  -- body moment of inertia about its own CG
b = 0.02     # N*s/m   -- viscous friction at the wheel axle
g = 9.81     # m/s^2

# =========================================================================
# 2) LQR WEIGHTS (Bryson's rule) -- replace thresholds with what YOU
#    consider "acceptable max deviation" for your robot's size/speed.
# =========================================================================
x_max     = 0.20   # m       -- max acceptable cart position error
xdot_max  = 0.50   # m/s     -- max acceptable cart speed
theta_max = 0.10   # rad     -- max acceptable tilt error (~5.7 deg)
thetadot_max = 1.0 # rad/s   -- max acceptable tilt rate
u_max     = 3.0    # N       -- max acceptable control force (actuator budget)

Q = np.diag([1/x_max**2, 1/xdot_max**2, 1/theta_max**2, 1/thetadot_max**2])
R = np.array([[1/u_max**2]])

# =========================================================================
# 3) SENSOR NOISE (for the Kalman observer gain) -- replace with YOUR
#    sensors' actual measured noise if you characterize them; these are
#    the doc's placeholders for AS5600 (~1mm) and post-filter MPU6050 (~0.02 rad).
# =========================================================================
encoder_pos_noise_std = 0.001   # m,   AS5600 -> linear position noise (1-sigma)
imu_theta_noise_std   = 0.02    # rad, MPU6050 (after complementary filter)
Rn = np.diag([encoder_pos_noise_std**2, imu_theta_noise_std**2])

# Process noise "belief" -- small first guess, then scaled up (see doc
# Buoc 7: x100) until observer poles are ~3-10x faster than controller poles.
Qn_base = np.diag([1e-6, 1e-4, 1e-6, 1e-3])
QN_SCALE = 100.0
Qn = Qn_base * QN_SCALE

# =========================================================================
# 4) SAMPLE TIME -- must match TIM2 period in config.h (CONTROL_PERIOD_S)
# =========================================================================
Ts = 0.005   # 5 ms -> 200 Hz control loop


def build_model():
    p = (M + m) * (I + m * l**2) - (m * l)**2
    if abs(p) < 1e-12:
        raise ValueError("Mass-matrix determinant ~0 -- check M,m,l,I")

    A = np.array([
        [0, 1, 0, 0],
        [0, -b * (I + m * l**2) / p, -(m**2) * g * (l**2) / p, 0],
        [0, 0, 0, 1],
        [0, b * m * l / p, g * m * l * (M + m) / p, 0],
    ])
    B = np.array([[0], [(I + m * l**2) / p], [0], [-m * l / p]])
    # C: row1 = cart position from encoders, row2 = tilt angle from IMU.
    # NOTE: only 2 measurements on purpose -- see doc Buoc 4 (theta alone is
    # NOT observable, rank(Obsv)=3/4; must have both encoder AND IMU).
    C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
    D = np.zeros((2, 1))
    return A, B, C, D, p


def carr1(name, v):
    v = np.asarray(v).ravel()
    body = ", ".join(f"{x:.8f}f" for x in v)
    return f"static const float {name}[{len(v)}] = {{ {body} }};"


def carr2(name, m_):
    rows, cols = m_.shape
    lines = [f"static const float {name}[{rows}][{cols}] = {{"]
    for r in range(rows):
        row = ", ".join(f"{x:.8f}f" for x in m_[r, :])
        lines.append(f"    {{ {row} }},")
    lines.append("};")
    return "\n".join(lines)


def main():
    np.set_printoptions(precision=4, suppress=True)
    A, B, C, D, p = build_model()

    print("=" * 70)
    print("STEP 1-2: continuous state-space model")
    print("=" * 70)
    print("mass-matrix determinant p =", p)
    print("A =\n", A)
    print("B =\n", B.ravel())
    print("C =\n", C)

    print("\n" + "=" * 70)
    print("STEP 3-4: controllability / observability")
    print("=" * 70)
    rank_ctrb = np.linalg.matrix_rank(ct.ctrb(A, B))
    rank_obsv_full = np.linalg.matrix_rank(ct.obsv(A, C))
    rank_obsv_imu_only = np.linalg.matrix_rank(ct.obsv(A, C[1:2, :]))
    print(f"rank(ctrb)            = {rank_ctrb}/4  "
          f"({'OK full rank' if rank_ctrb == 4 else 'NOT CONTROLLABLE -- check model!'})")
    print(f"rank(obsv, IMU only)  = {rank_obsv_imu_only}/4  "
          f"({'full rank (unexpected)' if rank_obsv_imu_only == 4 else 'not observable -- need encoder too, as expected'})")
    print(f"rank(obsv, enc+IMU)   = {rank_obsv_full}/4  "
          f"({'OK full rank' if rank_obsv_full == 4 else 'NOT OBSERVABLE -- check sensor placement!'})")
    if rank_ctrb != 4 or rank_obsv_full != 4:
        print("\n*** WARNING: system is not controllable/observable with these ***")
        print("*** parameters. Do not deploy Kd/Ld computed below.           ***")

    print("\n" + "=" * 70)
    print("STEP 5-6: continuous LQR")
    print("=" * 70)
    print("Q =", np.diag(Q))
    print("R =", R.ravel())
    K, _, _ = ct.lqr(A, B, Q, R)
    eigK = np.linalg.eigvals(A - B @ K)
    print("K =", K.ravel())
    print("eig(A-BK) =", eigK)
    if np.max(eigK.real) >= 0:
        print("*** WARNING: closed loop not stable, retune Q/R ***")

    print("\n" + "=" * 70)
    print("STEP 7: continuous Kalman gain (dual Riccati)")
    print("=" * 70)
    print("Rn =", np.diag(Rn), " Qn (scaled x%.0f) =" % QN_SCALE, np.diag(Qn))
    L, _, _ = ct.lqr(A.T, C.T, Qn, Rn)
    L = L.T
    eigL = np.linalg.eigvals(A - L @ C)
    print("L =\n", L)
    print("eig(A-LC) =", eigL)
    # "speed" of a pole = |Re(pole)| (decay rate); the SLOWEST pole in each
    # set (smallest |Re|) is what dominates settling time / the rule-of-thumb
    # comparison (observer should settle several times faster than the
    # controller's slowest mode, i.e. its slowest pole should still be
    # several times faster than the controller's slowest pole).
    slowest_ctrl = np.min(np.abs(eigK.real))
    slowest_obs = np.min(np.abs(eigL.real))
    ratio = slowest_obs / slowest_ctrl
    print(f"slowest controller pole |Re|={slowest_ctrl:.3f}, "
          f"slowest observer pole |Re|={slowest_obs:.3f}")
    print(f"observer/controller speed ratio ~ {ratio:.2f}x "
          f"({'OK (>=3x rule of thumb)' if ratio >= 3 else 'SLOW -- consider raising QN_SCALE'})")

    print("\n" + "=" * 70)
    print("STEP 9: ZOH discretization @ Ts=%.4gs + discrete redesign" % Ts)
    print("=" * 70)
    sysd = ct.c2d(ct.ss(A, B, C, D), Ts, method="zoh")
    Ad, Bd, Cd = sysd.A, sysd.B, sysd.C
    Kd, _, _ = ct.dlqr(Ad, Bd, Q, R)
    Ld, _, _ = ct.dlqr(Ad.T, Cd.T, Qn * Ts, Rn / Ts)
    Ld = Ld.T
    magK = np.abs(np.linalg.eigvals(Ad - Bd @ Kd))
    magL = np.abs(np.linalg.eigvals(Ad - Ld @ Cd))
    print("Kd =", Kd.ravel())
    print("|eig(Ad-BdKd)| =", magK, "-> stable" if np.max(magK) < 1 else "-> UNSTABLE")
    print("Ld =\n", Ld)
    print("|eig(Ad-LdCd)| =", magL, "-> stable" if np.max(magL) < 1 else "-> UNSTABLE")

    print("\n" + "=" * 70)
    print("PASTE INTO Core/Inc/config.h  (replace the placeholder block)")
    print("=" * 70)
    print(carr2("AD", Ad))
    print()
    print(carr1("BD", Bd))
    print()
    print("/* Cd is fixed by sensor choice: row0=encoder position, row1=IMU angle */")
    print(carr2("CD", Cd))
    print()
    print(carr1("KD", Kd))
    print()
    print(carr2("LD", Ld))
    print()
    print(f"/* re-run with Ts = {Ts}s -- must match CONTROL_PERIOD_S in config.h */")


if __name__ == "__main__":
    main()
