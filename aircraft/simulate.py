"""
DJI Phantom 4 Pro quadrotor — MuJoCo simulation & geometric tracking controller.

Loads phantom4pro.xml and flies a waypoint trajectory with the cascade
geometric controller of Lee, Leok & McClamroch (2010), plus:
  * first-order motor dynamics (tau = 0.03 s) with RPM saturation (8500 rpm),
  * rotor gyroscopic coupling (optional, physically small at hover),
  * exact numeric control allocation (no hand-derived sign mistakes).

Rotor forces (thrust + reaction torque) are applied through
data.xfrc_applied, computed in the body frame and rotated into the world
frame — see phantom4pro.xml and README.md.

Usage:
    python simulate.py            # headless, prints tracking summary + saves plot
    python simulate.py --render   # open the interactive MuJoCo viewer
    python simulate.py --T 16 --dt 0.005

All physical constants live at the top of this file and are documented in
README.md. They are consistent with the geometry in phantom4pro.xml.
"""

import argparse
from pathlib import Path

import numpy as np
import mujoco

# ---------------------------------------------------------------------------
# Physical parameters — DJI Phantom 4 Pro (see README.md for full derivation)
# ---------------------------------------------------------------------------
GRAVITY    = 9.81            # m/s^2
MASS       = 1.388           # kg, take-off mass (battery + props included)
KF         = 1.0e-5          # thrust coefficient       N / (rad/s)^2
KM         = 1.5e-7          # reaction-torque coeff.   N*m / (rad/s)^2
L          = 0.175           # arm length, m  (wheelbase 350 mm -> L = 0.175)
ROTOR_Z    = 0.03            # rotor plane height above COM, m
OMEGA_MAX  = 8500.0 * 2.0*np.pi/60.0   # max rotor speed, rad/s (8500 rpm)
TAU_MOTOR  = 0.03            # s, first-order motor+ESC time constant
I_ROTOR    = 5.0e-5          # kg*m^2, prop + motor-bell inertia (gyro term)
GYRO       = True            # enable rotor gyroscopic coupling
LINEAR_DRAG = np.array([0.18, 0.18, 0.28])     # N / (m/s), body-frame drag
ANGULAR_DRAG = np.array([0.018, 0.018, 0.025]) # N*m / (rad/s), body-frame damping

# Rotor geometry (X-configuration), body frame x=forward, y=left, z=up.
# rotor_dir[i] = +1 CCW / -1 CW (viewed from above); reaction torque sign follows.
_HALF = L / np.sqrt(2.0)
ROTOR_XY = np.array([
    [_HALF, -_HALF],   # 0 front-right  CW
    [-_HALF, -_HALF],  # 1 front-left   CCW
    [-_HALF,  _HALF],  # 2 rear-left    CW
    [_HALF,  _HALF],   # 3 rear-right   CCW
])
ROTOR_DIR = np.array([-1.0, 1.0, -1.0, 1.0])

# Control allocation:  u = [f, tau_x, tau_y, tau_z] -> w = [w0^2..w3^2] = B^-1 u
# Built numerically from the rotor geometry so the signs cannot be wrong.
_B = np.zeros((4, 4))
_B[0, :] = KF                        # total thrust
for i in range(4):
    x, y = ROTOR_XY[i]
    _B[1, i] =  y * KF                # tau_x = sum_i y_i * F_i
    _B[2, i] = -x * KF                # tau_y = -sum_i x_i * F_i
    _B[3, i] = ROTOR_DIR[i] * KM      # tau_z = sum_i s_i * k_m * w_i
_B_INV = np.linalg.inv(_B)

# Controller gains (per-axis for position; scalar for attitude)
KP_POS = np.array([3.0, 3.0, 6.0])   # 1/s^2
KD_POS = np.array([2.5, 2.5, 4.0])   # 1/s
KR      = 2.0                        # N*m / rad
KW      = 0.4                        # N*m / (rad/s)


def quat2mat(q):
    """Body->world rotation from a MuJoCo quaternion (w, x, y, z)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def vee(S):
    """vee map of a 3x3 skew-symmetric matrix -> 3-vector."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def _min_jerk_segment(t, t0, t1, p0, p1):
    """Return position, velocity, and acceleration for a quintic segment."""
    duration = t1 - t0
    s = np.clip((t - t0) / duration, 0.0, 1.0)
    h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
    hd = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / duration
    hdd = (60.0*s - 180.0*s**2 + 120.0*s**3) / (duration**2)
    delta = p1 - p0
    return p0 + h * delta, hd * delta, hdd * delta


def _arc_segment(t, t0, t1, center, radius, z):
    """Trace an upper semicircle with continuous position derivatives."""
    duration = t1 - t0
    s = np.clip((t - t0) / duration, 0.0, 1.0)
    h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
    hd = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / duration
    hdd = (60.0*s - 180.0*s**2 + 120.0*s**3) / (duration**2)

    # theta goes from pi to zero: (0, 0) -> (2*radius, 0), bowing +y.
    theta = np.pi * (1.0 - h)
    theta_d = -np.pi * hd
    theta_dd = -np.pi * hdd
    d1 = np.array([-radius * np.sin(theta), radius * np.cos(theta), 0.0])
    d2 = np.array([-radius * np.cos(theta), -radius * np.sin(theta), 0.0])
    pos = np.array([
        center[0] + radius * np.cos(theta),
        center[1] + radius * np.sin(theta),
        z,
    ])
    vel = d1 * theta_d
    acc = d2 * theta_d**2 + d1 * theta_dd
    # Heading follows the tangent.  At both endpoints this remains well-defined.
    yaw = theta - 0.5 * np.pi
    return pos, vel, acc, yaw


def waypoint(t):
    """Return (pos_d, vel_d, acc_d, yaw_d) for a smooth curved flight."""
    takeoff_z = 1.25
    land_z = 0.35
    cruise = np.array([0.0, 0.0, takeoff_z])
    arc_end = np.array([2.0, 0.0, takeoff_z])
    landing = np.array([2.0, 0.0, land_z])

    if t < 2.0:
        pos, vel, acc = _min_jerk_segment(
            t, 0.0, 2.0, np.array([0.0, 0.0, land_z]), cruise
        )
        return pos, vel, acc, 0.0
    if t < 4.0:
        # Rotate to the arc's entry tangent while holding position.
        pos = cruise.copy()
        s = np.clip((t - 2.0) / 2.0, 0.0, 1.0)
        h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
        return pos, np.zeros(3), np.zeros(3), 0.5 * np.pi * h
    if t < 10.0:
        return _arc_segment(t, 4.0, 10.0, np.array([1.0, 0.0]), 1.0, takeoff_z)
    if t < 11.0:
        return arc_end, np.zeros(3), np.zeros(3), -0.5 * np.pi
    if t < 13.5:
        pos, vel, acc = _min_jerk_segment(t, 11.0, 13.5, arc_end, landing)
        return pos, vel, acc, -0.5 * np.pi
    return landing, np.zeros(3), np.zeros(3), -0.5 * np.pi


def control(model, data, pos_d, vel_d, acc_d, yaw_d, omega, dt):
    """Geometric tracking controller. Returns the rotor speeds (after motor
    dynamics). The forces themselves are applied in apply_forces()."""
    pos = data.qpos[0:3]
    R   = quat2mat(data.qpos[3:7])
    vel = data.qvel[0:3]
    w   = data.qvel[3:6]             # MuJoCo free-joint angular velocity, body frame
    zB  = R[:, 2]

    # --- position loop -> desired body z-axis & thrust magnitude ---
    a_d = -KP_POS * (pos - pos_d) - KD_POS * (vel - vel_d) + acc_d
    F   = MASS * (a_d + np.array([0.0, 0.0, GRAVITY]))
    zBd = F / np.linalg.norm(F)
    f   = max(float(F @ zB), 0.0)          # thrust (>=0, cannot push down)

    # --- attitude loop (desired heading = yaw_d) ---
    xc  = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
    yBd = np.cross(zBd, xc)
    yBd = yBd / np.linalg.norm(yBd)
    xBd = np.cross(yBd, zBd)
    Rd  = np.column_stack([xBd, yBd, zBd])

    eR  = 0.5 * vee(Rd.T @ R - R.T @ Rd)   # attitude (rotation) error, rad
    eW  = w                                 # omega_d = 0 for fixed yaw
    tau = -KR * eR - KW * eW                # desired torque, body frame

    # --- allocation -> commanded rotor speeds ---
    w2    = _B_INV @ np.array([f, tau[0], tau[1], tau[2]])
    w2    = np.clip(w2, 0.0, None)
    w_cmd = np.sqrt(w2)

    # --- motor dynamics + saturation ---
    omega = omega + (dt / TAU_MOTOR) * (w_cmd - omega)
    omega = np.clip(omega, 0.0, OMEGA_MAX)
    return omega


def apply_forces(model, data, omega):
    """Apply thrust + reaction torque + gyro coupling via data.xfrc_applied.

    Everything is computed in the body frame (so the forces tilt with the
    airframe), then rotated once into the world frame required by xfrc_applied.
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    R  = quat2mat(data.qpos[3:7])
    v_body = R.T @ data.qvel[0:3]
    w_body = data.qvel[3:6]

    F_body = -LINEAR_DRAG * v_body
    tau_body = -ANGULAR_DRAG * w_body
    for i in range(4):
        f = KF * omega[i] ** 2
        r = np.array([ROTOR_XY[i][0], ROTOR_XY[i][1], ROTOR_Z])
        F_body += np.array([0.0, 0.0, f])                 # thrust, body +z
        tau_body += np.cross(r, np.array([0.0, 0.0, f]))  # thrust lever arm
        tau_body += np.array([0.0, 0.0, ROTOR_DIR[i] * KM * omega[i] ** 2])

    if GYRO:
        h = I_ROTOR * float(np.sum(ROTOR_DIR * omega)) * np.array([0.0, 0.0, 1.0])
        tau_body += np.cross(h, w_body)                   # rotor gyroscopic torque

    data.xfrc_applied[body_id, 0:3] = R @ F_body
    data.xfrc_applied[body_id, 3:6] = R @ tau_body


def update_rotor_visuals(model, data, omega, angles):
    """Spin the visual-only hinged propellers in render/saved states."""
    for i, name in enumerate(("rotor0_spin", "rotor1_spin", "rotor2_spin", "rotor3_spin")):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            continue
        qadr = model.jnt_qposadr[joint_id]
        vadr = model.jnt_dofadr[joint_id]
        data.qpos[qadr] = angles[i]
        data.qvel[vadr] = ROTOR_DIR[i] * omega[i]


def run(T=16.0, dt=0.005, render=False):
    model_path = Path(__file__).with_name("phantom4pro.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data  = mujoco.MjData(model)
    model.opt.timestep = dt

    omega = np.full(4, np.sqrt(MASS * GRAVITY / (4 * KF)))  # start near hover
    prop_angles = np.zeros(4)

    steps = int(T / dt)
    tlog  = np.zeros(steps)
    plog  = np.zeros((steps, 3))
    pdlog = np.zeros((steps, 3))

    if render:
        import time
        from mujoco import viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)

    for k in range(steps):
        t = k * dt
        pos_d, vel_d, acc_d, yaw_d = waypoint(t)
        omega = control(model, data, pos_d, vel_d, acc_d, yaw_d, omega, dt)
        prop_angles = (prop_angles + ROTOR_DIR * omega * dt) % (2.0 * np.pi)
        data.xfrc_applied[:] = 0.0
        apply_forces(model, data, omega)

        mujoco.mj_step(model, data)
        update_rotor_visuals(model, data, omega, prop_angles)

        tlog[k]   = t
        plog[k]   = data.qpos[0:3].copy()
        pdlog[k]  = pos_d

        if render:
            viewer.sync()
            time.sleep(dt)

    if render:
        viewer.close()

    err = np.linalg.norm(plog - pdlog, axis=1)
    rms = float(np.sqrt(np.mean(err ** 2)))
    mx  = float(np.max(err))

    print(f"simulated {T:.1f}s  (dt={dt})")
    print(f"final position        = [{plog[-1,0]:+.3f}, {plog[-1,1]:+.3f}, {plog[-1,2]:+.3f}] m")
    print(f"final |omega|         = [{omega[0]:.0f}, {omega[1]:.0f}, {omega[2]:.0f}, {omega[3]:.0f}] rad/s")
    print(f"RMS tracking error    = {rms*100:.2f} cm")
    print(f"max  tracking error   = {mx*100:.2f} cm")

    np.savez("trajectory.npz", t=tlog, pos=plog, pos_des=pdlog, err=err)
    _plot(tlog, plog, pdlog, err)
    return rms


def _plot(t, pos, pos_d, err):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("(matplotlib not installed — skipping plot; see trajectory.npz)")
        return

    fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    labels = ["x", "y", "z"]
    for i in range(3):
        ax[i].plot(t, pos_d[:, i], "k--", lw=1.0, label="desired")
        ax[i].plot(t, pos[:, i], "C0", lw=1.2, label="actual")
        ax[i].set_ylabel(f"{labels[i]} [m]")
        ax[i].grid(True, alpha=0.3)
        ax[i].legend(loc="upper right", fontsize=8)
    ax[0].set_title("DJI Phantom 4 Pro — waypoint tracking (MuJoCo)")
    ax[2].set_xlabel("time [s]")
    plt.tight_layout()
    plt.savefig("trajectory.png", dpi=120)
    print("saved trajectory.png")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=16.0)
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--render", action="store_true")
    args = ap.parse_args()
    run(args.T, args.dt, args.render)
