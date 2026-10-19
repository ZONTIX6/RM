"""
Three-quadrotor swarm demo for the Phantom 4 Pro model.

The vehicles start on the ground at reproducibly random positions around a
ground target.  After a minimum-jerk takeoff, every vehicle runs a local
distributed model-predictive controller (DMPC).  The local prediction model
uses only its own state and the latest neighbor states.  A control-barrier
function (CBF) safety filter then projects the nominal acceleration onto the
pairwise separation constraints before the command reaches the SE(3) inner
loop.
"""

import argparse
import copy
import itertools
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import mujoco

import simulate as single


N_DRONES = 3
TARGET_GROUND = np.array([2.0, 0.0, 0.0])
CRUISE_Z = 1.25
TARGET_AIR = TARGET_GROUND + np.array([0.0, 0.0, CRUISE_Z])
DEFAULT_SEED = None
INITIAL_RADIUS = (1.6, 2.7)
MIN_INITIAL_SEPARATION = 0.90
INITIAL_ANGLE_SPREAD = 0.55

FORMATION_OFFSETS = np.array([
    [0.00, 0.46, 0.0],
    [-0.40, -0.23, 0.0],
    [0.40, -0.23, 0.0],
])

# Mission timing.  The free-joint origin is the vehicle COM; with the XML
# ground plane at z=-0.05, GROUND_START_Z puts the skid capsules on the plane.
GROUND_START_Z = 0.099
TAKEOFF_DURATION = 3.0
TRANSIT_DURATION = 8.0

# Distributed MPC settings.  The optimizer is solved once per control period,
# while the MuJoCo dynamics continue at the smaller physics timestep.
MPC_HORIZON = 14
MPC_PREDICTION_DT = 0.08
MPC_CONTROL_PERIOD = 0.08
MPC_Q_STATE = np.array([5.0, 5.0, 8.0, 1.8, 1.8, 2.4])
MPC_Q_TERMINAL = np.array([12.0, 12.0, 18.0, 3.0, 3.0, 3.6])
MPC_Q_FORMATION = np.array([3.2, 3.2, 4.0])
MPC_R_ACCEL = np.array([0.20, 0.20, 0.26])
MPC_R_SMOOTH = np.array([0.90, 0.90, 1.10])
ACCEL_LIMIT = np.array([3.0, 3.0, 3.0])
JERK_LIMIT = np.array([4.0, 4.0, 4.5])

# Pairwise CBF safety filter.  It is a small Euclidean projection QP solved
# with repeated half-space projections, which is sufficient for three agents.
CBF_SAFE_DISTANCE = 0.72
CBF_ALPHA = 4.0
CBF_PROJECTION_ITERS = 12

KP_POS = np.array([5.5, 5.5, 8.5])
KD_POS = np.array([4.5, 4.5, 6.0])
KR = 2.4
KW = 0.5


def quat2mat(q):
    return single.quat2mat(q)


def vee(S):
    return single.vee(S)


def sample_initial_positions(seed):
    """Sample well-separated random ground positions around the target."""
    rng = np.random.default_rng(seed)
    formation_angles = np.arctan2(FORMATION_OFFSETS[:, 1], FORMATION_OFFSETS[:, 0])
    for _ in range(1000):
        angles = formation_angles + rng.uniform(
            -INITIAL_ANGLE_SPREAD,
            INITIAL_ANGLE_SPREAD,
            N_DRONES,
        )
        radii = rng.uniform(INITIAL_RADIUS[0], INITIAL_RADIUS[1], N_DRONES)
        positions = np.column_stack([
            TARGET_GROUND[0] + radii * np.cos(angles),
            TARGET_GROUND[1] + radii * np.sin(angles),
            np.full(N_DRONES, GROUND_START_Z),
        ])
        distances = [
            np.linalg.norm(positions[i] - positions[j])
            for i in range(N_DRONES)
            for j in range(i + 1, N_DRONES)
        ]
        if min(distances) >= MIN_INITIAL_SEPARATION:
            goal_positions = TARGET_AIR + FORMATION_OFFSETS
            assignment = min(
                itertools.permutations(range(N_DRONES)),
                key=lambda perm: sum(
                    np.linalg.norm(positions[perm[i]] - goal_positions[i])
                    for i in range(N_DRONES)
                ),
            )
            return positions[list(assignment)]
    raise RuntimeError("Could not sample separated ground positions")


def smoothstep5(s):
    """Quintic blend with zero velocity and acceleration at both endpoints."""
    s = float(np.clip(s, 0.0, 1.0))
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def smoothstep5_derivatives(s, duration):
    """Return first and second derivatives of the quintic blend."""
    if duration <= 0.0:
        return 0.0, 0.0
    s = float(np.clip(s, 0.0, 1.0))
    sd = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / duration
    sdd = (60.0 * s - 180.0 * s**2 + 120.0 * s**3) / (duration**2)
    return sd, sdd


def quintic_reference(t, p0, p1, duration):
    """Smoothly interpolate position, velocity, and acceleration."""
    if t <= 0.0:
        return p0.copy(), np.zeros(3), np.zeros(3)
    if t >= duration:
        return p1.copy(), np.zeros(3), np.zeros(3)
    s = t / duration
    sd, sdd = smoothstep5_derivatives(s, duration)
    delta = p1 - p0
    return p0 + smoothstep5(s) * delta, sd * delta, sdd * delta


def mission_reference(t, initial_positions):
    """Return smooth per-agent references for takeoff and convergence."""
    positions = np.zeros((N_DRONES, 3))
    velocities = np.zeros((N_DRONES, 3))
    accelerations = np.zeros((N_DRONES, 3))

    for i in range(N_DRONES):
        takeoff_start = initial_positions[i].copy()
        takeoff_goal = np.array([
            initial_positions[i, 0],
            initial_positions[i, 1],
            CRUISE_Z,
        ])
        if t < TAKEOFF_DURATION:
            p, v, a = quintic_reference(
                t, takeoff_start, takeoff_goal, TAKEOFF_DURATION
            )
        else:
            transit_start = takeoff_goal
            transit_goal = TARGET_AIR + FORMATION_OFFSETS[i]
            p, v, a = quintic_reference(
                t - TAKEOFF_DURATION,
                transit_start,
                transit_goal,
                TRANSIT_DURATION,
            )
        positions[i] = p
        velocities[i] = v
        accelerations[i] = a
    return positions, velocities, accelerations


def jerk_limit(accel, desired_accel, dt):
    delta = np.clip(desired_accel - accel, -JERK_LIMIT * dt, JERK_LIMIT * dt)
    return np.clip(accel + delta, -ACCEL_LIMIT, ACCEL_LIMIT)


def _prediction_matrices(horizon, dt):
    """Build condensed double-integrator prediction matrices."""
    eye = np.eye(3)
    A = np.block([
        [eye, dt * eye],
        [np.zeros((3, 3)), eye],
    ])
    B = np.vstack([
        0.5 * dt**2 * eye,
        dt * eye,
    ])
    F = np.zeros((6 * horizon, 6))
    G = np.zeros((6 * horizon, 3 * horizon))
    for k in range(horizon):
        F[6 * k:6 * (k + 1)] = np.linalg.matrix_power(A, k + 1)
        for m in range(k + 1):
            G[6 * k:6 * (k + 1), 3 * m:3 * (m + 1)] = (
                np.linalg.matrix_power(A, k - m) @ B
            )
    return F, G


def _position_selector(horizon):
    selector = np.zeros((3 * horizon, 6 * horizon))
    for k in range(horizon):
        selector[3 * k:3 * (k + 1), 6 * k:6 * k + 3] = np.eye(3)
    return selector


class DistributedMPC:
    """Local finite-horizon DMPC using the latest neighbor state estimates."""

    def __init__(self, horizon=MPC_HORIZON, prediction_dt=MPC_PREDICTION_DT):
        self.horizon = horizon
        self.prediction_dt = prediction_dt
        self.F, self.G = _prediction_matrices(horizon, prediction_dt)
        self.P = _position_selector(horizon)

        q_state = np.diag(MPC_Q_STATE)
        q_terminal = np.diag(MPC_Q_TERMINAL)
        self.Q = np.kron(np.eye(horizon), q_state)
        self.Q[-6:, -6:] = q_terminal
        self.Q_form = np.kron(np.eye(horizon), np.diag(MPC_Q_FORMATION))
        self.R = np.kron(np.eye(horizon), np.diag(MPC_R_ACCEL))

        # D U = [u_0-u_prev, u_1-u_0, ...] penalizes command changes.
        self.D = np.zeros((3 * horizon, 3 * horizon))
        self.D[:3, :3] = np.eye(3)
        for k in range(1, horizon):
            self.D[3 * k:3 * (k + 1), 3 * k:3 * (k + 1)] = np.eye(3)
            self.D[3 * k:3 * (k + 1), 3 * (k - 1):3 * k] = -np.eye(3)
        self.R_smooth = np.kron(np.eye(horizon), np.diag(MPC_R_SMOOTH))

        Gp = self.P @ self.G
        self._Gp = Gp
        self._H = (
            self.G.T @ self.Q @ self.G
            + self.R
            + self.D.T @ self.R_smooth @ self.D
            + 2.0 * self._Gp.T @ self.Q_form @ self._Gp
            + 1e-7 * np.eye(3 * horizon)
        )

    def solve(
        self,
        t,
        positions,
        velocities,
        initial_positions,
        drone_index,
        previous_accel,
    ):
        """Solve one local QP and return the first acceleration command."""
        refs_p = []
        refs_v = []
        refs_a = []
        for k in range(self.horizon):
            p_ref, v_ref, a_ref = mission_reference(
                t + (k + 1) * self.prediction_dt,
                initial_positions,
            )
            refs_p.append(p_ref[drone_index])
            refs_v.append(v_ref[drone_index])
            refs_a.append(a_ref[drone_index])

        x0 = np.concatenate([positions[drone_index], velocities[drone_index]])
        x_ref = np.column_stack([refs_p, refs_v]).reshape(-1)
        u_ref = np.asarray(refs_a).reshape(-1)
        x_free = self.F @ x0
        rhs = self.G.T @ self.Q @ (x_ref - x_free)
        rhs += self.R @ u_ref

        # Each neighbor contributes a local formation-relative prediction.
        # Constant-velocity neighbor prediction is the information exchanged
        # between agents; no centralized future trajectory is required.
        p_free = self.P @ x_free
        for j in range(N_DRONES):
            if j == drone_index:
                continue
            tau = self.prediction_dt * np.arange(1, self.horizon + 1)
            neighbor_prediction = (
                positions[j][None, :]
                + tau[:, None] * velocities[j][None, :]
                - (FORMATION_OFFSETS[j] - FORMATION_OFFSETS[drone_index])[None, :]
            )
            formation_rhs = neighbor_prediction.reshape(-1) - p_free
            rhs += self._Gp.T @ self.Q_form @ formation_rhs

        delta_ref = np.zeros(3 * self.horizon)
        delta_ref[:3] = previous_accel
        u = np.linalg.solve(self._H, rhs + self.D.T @ self.R_smooth @ delta_ref)
        return np.clip(u[:3], -ACCEL_LIMIT, ACCEL_LIMIT)


def cbf_filter(positions, velocities, nominal_accelerations, dt):
    """Project accelerations onto pairwise discrete-time CBF constraints."""
    filtered = np.clip(nominal_accelerations.copy(), -ACCEL_LIMIT, ACCEL_LIMIT)
    active_constraints = 0
    for _ in range(CBF_PROJECTION_ITERS):
        changed = False
        for i in range(N_DRONES):
            for j in range(N_DRONES):
                if i == j:
                    continue
                displacement = positions[i] - positions[j]
                distance = np.linalg.norm(displacement)
                if distance < 1e-6:
                    continue
                relative_velocity = velocities[i] - velocities[j]
                h = distance**2 - CBF_SAFE_DISTANCE**2
                normal = 2.0 * dt * displacement
                rhs = (
                    -2.0 * np.dot(displacement, relative_velocity)
                    - CBF_ALPHA * h
                    + np.dot(normal, filtered[j])
                )
                violation = rhs - np.dot(normal, filtered[i])
                if violation <= 0.0 or np.dot(normal, normal) < 1e-10:
                    continue
                filtered[i] += (
                    violation / np.dot(normal, normal)
                ) * normal
                filtered[i] = np.clip(filtered[i], -ACCEL_LIMIT, ACCEL_LIMIT)
                active_constraints += 1
                changed = True
        if not changed:
            break
    return filtered, active_constraints


def make_swarm_xml(src_path, dst_path, initial_positions):
    tree = ET.parse(src_path)
    root = tree.getroot()
    root.set("model", "phantom4pro_swarm")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Missing worldbody in template XML")

    ground = worldbody.find("geom[@name='ground']")
    if ground is None:
        raise ValueError("Could not find ground geom in template XML")
    ground.set("contype", "2")
    ground.set("conaffinity", "1")
    ground.set("friction", "0.8 0.1 0.1")

    template = None
    for child in list(worldbody):
        if child.tag == "body" and child.get("name") == "quadrotor":
            template = child
            worldbody.remove(child)
            break
    if template is None:
        raise ValueError("Could not find quadrotor template body")

    def suffix_names(elem, suffix):
        name = elem.get("name")
        if name:
            elem.set("name", f"{name}_{suffix}")
        for child in list(elem):
            suffix_names(child, suffix)

    for i, pos in enumerate(initial_positions):
        clone = copy.deepcopy(template)
        suffix_names(clone, i)
        clone.set("name", f"quadrotor_{i}")
        clone.set("pos", f"{pos[0]} {pos[1]} {pos[2]}")
        for geom in clone.iter("geom"):
            if geom.get("name", "").startswith(("skid_r", "skid_l")):
                geom.set("contype", "1")
                geom.set("conaffinity", "2")
                geom.set("friction", "0.8 0.1 0.1")
        worldbody.append(clone)

    goal = ET.Element(
        "geom",
        {
            "name": "goal_pad",
            "type": "cylinder",
            "pos": f"{TARGET_GROUND[0]} {TARGET_GROUND[1]} 0.005",
            "size": "0.12 0.005",
            "rgba": "0.86 0.12 0.10 0.85",
        },
    )
    worldbody.append(goal)

    add_scene_markers(worldbody, initial_positions)

    ET.indent(tree, space="  ")
    tree.write(dst_path, encoding="utf-8", xml_declaration=True)


def add_scene_markers(worldbody, initial_positions):
    colors = [
        "0.05 0.32 0.95 0.85",
        "1.00 0.45 0.02 0.85",
        "0.05 0.65 0.18 0.85",
    ]

    def add_segment(name, p0, p1, radius, rgba):
        if np.linalg.norm(p1 - p0) < 1e-4:
            return
        worldbody.append(ET.Element(
            "geom",
            {
                "name": name,
                "type": "capsule",
                "fromto": f"{p0[0]:.4f} {p0[1]:.4f} {p0[2]:.4f} {p1[0]:.4f} {p1[1]:.4f} {p1[2]:.4f}",
                "size": f"{radius:.4f}",
                "rgba": rgba,
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
            },
        ))

    def add_marker(name, pos, size, rgba):
        worldbody.append(ET.Element(
            "geom",
            {
                "name": name,
                "type": "sphere",
                "pos": f"{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}",
                "size": f"{size:.4f}",
                "rgba": rgba,
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
            },
        ))

    add_segment(
        "target_vertical",
        TARGET_GROUND + np.array([0.0, 0.0, 0.02]),
        TARGET_AIR,
        0.004,
        "0.85 0.05 0.05 0.55",
    )
    add_marker("target_air_point", TARGET_AIR, 0.025, "0.85 0.05 0.05 0.95")

    for i, pos in enumerate(initial_positions):
        add_marker(f"initial_uav{i}", pos, 0.030, colors[i])
        add_marker(f"formation_goal{i}", TARGET_AIR + FORMATION_OFFSETS[i], 0.022, colors[i])


def get_freejoint_state(model, data, joint_id):
    qadr = model.jnt_qposadr[joint_id]
    vadr = model.jnt_dofadr[joint_id]
    pos = data.qpos[qadr:qadr + 3]
    quat = data.qpos[qadr + 3:qadr + 7]
    vel = data.qvel[vadr:vadr + 3]
    w = data.qvel[vadr + 3:vadr + 6]
    return pos, quat, vel, w


def control_drone(pos_d, vel_d, acc_d, yaw_d, pos, quat, vel, w, omega, dt):
    R = quat2mat(quat)
    zB = R[:, 2]

    a_d = -KP_POS * (pos - pos_d) - KD_POS * (vel - vel_d) + acc_d
    F = single.MASS * (a_d + np.array([0.0, 0.0, single.GRAVITY]))
    F_norm = np.linalg.norm(F)
    zBd = F / F_norm if F_norm > 1e-9 else np.array([0.0, 0.0, 1.0])
    f = max(float(F @ zB), 0.0)

    xc = np.array([math.cos(yaw_d), math.sin(yaw_d), 0.0])
    yBd = np.cross(zBd, xc)
    if np.linalg.norm(yBd) < 1e-6:
        yBd = np.array([0.0, 1.0, 0.0])
    yBd = yBd / np.linalg.norm(yBd)
    xBd = np.cross(yBd, zBd)
    Rd = np.column_stack([xBd, yBd, zBd])

    eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)
    eW = w
    tau = -KR * eR - KW * eW

    w2 = single._B_INV @ np.array([f, tau[0], tau[1], tau[2]])
    w2 = np.clip(w2, 0.0, None)
    w_cmd = np.sqrt(w2)

    omega = omega + (dt / single.TAU_MOTOR) * (w_cmd - omega)
    omega = np.clip(omega, 0.0, single.OMEGA_MAX)
    return omega


def apply_forces(model, data, body_id, quat, vel, w, omega):
    R = quat2mat(quat)
    v_body = R.T @ vel
    tau_body = -single.ANGULAR_DRAG * w
    F_body = -single.LINEAR_DRAG * v_body

    for i in range(4):
        f = single.KF * omega[i] ** 2
        r = np.array([single.ROTOR_XY[i][0], single.ROTOR_XY[i][1], single.ROTOR_Z])
        F_body += np.array([0.0, 0.0, f])
        tau_body += np.cross(r, np.array([0.0, 0.0, f]))
        tau_body += np.array([0.0, 0.0, single.ROTOR_DIR[i] * single.KM * omega[i] ** 2])

    data.xfrc_applied[body_id, 0:3] = R @ F_body
    data.xfrc_applied[body_id, 3:6] = R @ tau_body


def update_rotor_visuals(model, data, joint_ids, omega, angles):
    for i in range(N_DRONES):
        for j in range(4):
            jid = joint_ids[i][j]
            qadr = model.jnt_qposadr[jid]
            vadr = model.jnt_dofadr[jid]
            data.qpos[qadr] = angles[i, j]
            data.qvel[vadr] = single.ROTOR_DIR[j] * omega[i, j]


def run(T=14.0, dt=0.005, render=False, speed=1.0, seed=DEFAULT_SEED):
    if speed <= 0.0:
        raise ValueError("speed must be positive")

    if seed is None:
        seed = int(np.random.default_rng().integers(0, 2**31 - 1))
    initial_positions = sample_initial_positions(seed)
    xml_path = Path(__file__).with_name("phantom4pro_swarm.xml")
    make_swarm_xml(Path(__file__).with_name("phantom4pro.xml"), xml_path, initial_positions)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    model.opt.timestep = dt
    mujoco.mj_forward(model, data)

    body_ids = []
    joint_ids = []
    root_joint_ids = []
    for i in range(N_DRONES):
        body_ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"quadrotor_{i}"))
        root_joint_ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"root_{i}"))
        joint_ids.append([
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"rotor0_spin_{i}"),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"rotor1_spin_{i}"),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"rotor2_spin_{i}"),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"rotor3_spin_{i}"),
        ])

    omega = np.full((N_DRONES, 4), np.sqrt(single.MASS * single.GRAVITY / (4 * single.KF)))
    prop_angles = np.zeros((N_DRONES, 4))
    mpc = DistributedMPC()
    control_stride = max(1, int(round(MPC_CONTROL_PERIOD / dt)))
    nominal_accelerations = np.zeros((N_DRONES, 3))
    command_accelerations = np.zeros((N_DRONES, 3))

    steps = int(T / dt)
    tlog = np.zeros(steps)
    center_log = np.zeros((steps, 3))
    center_des_log = np.zeros((steps, 3))
    pos_log = np.zeros((steps, N_DRONES, 3))
    pos_des_log = np.zeros((steps, N_DRONES, 3))
    pair_log = np.zeros((steps, 3))
    accel_log = np.zeros((steps, N_DRONES, 3))
    nominal_accel_log = np.zeros((steps, N_DRONES, 3))
    cbf_log = np.zeros(steps)
    phase_log = np.zeros(steps, dtype=np.int8)

    if render:
        import time
        from mujoco import viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)
        # Start with the whole mission in view, but keep a free camera so the
        # user can orbit and zoom with the mouse.
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = np.array([1.0, 0.0, 0.70])
        viewer.cam.distance = 4.2
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -28.0
        render_stride = max(1, int(round(1.0 / (60.0 * dt))))

    for k in range(steps):
        t = k * dt
        pos_step = np.zeros((N_DRONES, 3))
        vel_step = np.zeros((N_DRONES, 3))
        quat_step = []
        w_step = []

        for i in range(N_DRONES):
            pos, quat, vel, w = get_freejoint_state(model, data, root_joint_ids[i])
            pos_step[i] = pos
            vel_step[i] = vel
            quat_step.append(quat.copy())
            w_step.append(w.copy())

        desired_positions, desired_velocities, _ = mission_reference(
            t, initial_positions
        )

        if k % control_stride == 0:
            nominal_accelerations = np.array([
                mpc.solve(
                    t,
                    pos_step,
                    vel_step,
                    initial_positions,
                    i,
                    command_accelerations[i],
                )
                for i in range(N_DRONES)
            ])

        # Rate-limit the nominal DMPC output for visibly smooth motion, then
        # apply the CBF projection as the final safety-critical outer command.
        smooth_accelerations = np.array([
            jerk_limit(command_accelerations[i], nominal_accelerations[i], dt)
            for i in range(N_DRONES)
        ])
        command_accelerations, cbf_active = cbf_filter(
            pos_step,
            vel_step,
            smooth_accelerations,
            dt,
        )
        for i in range(N_DRONES):
            omega[i] = control_drone(
                desired_positions[i],
                desired_velocities[i],
                command_accelerations[i],
                0.0,
                pos_step[i],
                quat_step[i],
                vel_step[i],
                w_step[i],
                omega[i],
                dt,
            )

        des_step = desired_positions.copy()

        prop_angles = (prop_angles + single.ROTOR_DIR[np.newaxis, :] * omega * dt) % (2.0 * np.pi)
        data.xfrc_applied[:] = 0.0
        for i in range(N_DRONES):
            apply_forces(model, data, body_ids[i], quat_step[i], vel_step[i], w_step[i], omega[i])

        mujoco.mj_step(model, data)
        update_rotor_visuals(model, data, joint_ids, omega, prop_angles)

        center_actual = pos_step.mean(axis=0)
        center_des = des_step.mean(axis=0)
        pair_log[k, 0] = np.linalg.norm(pos_step[0] - pos_step[1])
        pair_log[k, 1] = np.linalg.norm(pos_step[0] - pos_step[2])
        pair_log[k, 2] = np.linalg.norm(pos_step[1] - pos_step[2])

        tlog[k] = t
        center_log[k] = center_actual
        center_des_log[k] = center_des
        pos_log[k] = pos_step
        pos_des_log[k] = des_step
        accel_log[k] = command_accelerations
        nominal_accel_log[k] = nominal_accelerations
        cbf_log[k] = cbf_active
        phase_log[k] = 0 if t < TAKEOFF_DURATION else (1 if t < TAKEOFF_DURATION + TRANSIT_DURATION else 2)

        if render:
            if k == 0 or k % render_stride == 0:
                viewer.sync()
            time.sleep(dt / speed)

    if render:
        print("simulation finished; close the MuJoCo window when you are done inspecting it")
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.05)

    center_err = np.linalg.norm(center_log - center_des_log, axis=1)
    agent_err = np.linalg.norm(pos_log - pos_des_log, axis=2)
    target_err = np.linalg.norm(center_log - TARGET_AIR, axis=1)
    goal_agent_err = np.linalg.norm(pos_log - (TARGET_AIR + FORMATION_OFFSETS), axis=2)
    formation_err = np.array([
        np.mean([
            np.linalg.norm(
                (pos_log[k, i] - pos_log[k, j])
                - (FORMATION_OFFSETS[i] - FORMATION_OFFSETS[j])
            )
            for i in range(N_DRONES)
            for j in range(i + 1, N_DRONES)
        ])
        for k in range(steps)
    ])
    rms_center = float(np.sqrt(np.mean(center_err ** 2)))
    max_center = float(np.max(center_err))
    rms_agent = float(np.sqrt(np.mean(agent_err ** 2)))
    final_target_err = float(target_err[-1])
    final_formation_err = float(formation_err[-1])
    min_sep = float(np.min(pair_log))
    convergence_err = np.maximum(target_err, formation_err)
    settled = np.flatnonzero(convergence_err > 0.10)
    settling_time = float(tlog[settled[-1]] + dt) if len(settled) else 0.0

    print(f"simulated {T:.1f}s  (dt={dt})")
    print(f"random seed        = {seed}")
    print(f"controller         = distributed MPC + CBF safety filter")
    print(f"initial center     = [{initial_positions[:,0].mean():+.3f}, {initial_positions[:,1].mean():+.3f}, {initial_positions[:,2].mean():+.3f}] m")
    print(f"final center        = [{center_log[-1,0]:+.3f}, {center_log[-1,1]:+.3f}, {center_log[-1,2]:+.3f}] m")
    print(f"target center       = [{TARGET_AIR[0]:+.3f}, {TARGET_AIR[1]:+.3f}, {TARGET_AIR[2]:+.3f}] m")
    print(f"RMS reference error = {rms_center*100:.2f} cm")
    print(f"RMS target error    = {np.sqrt(np.mean(target_err**2))*100:.2f} cm")
    print(f"final target error  = {final_target_err*100:.2f} cm")
    print(f"RMS goal error      = {np.sqrt(np.mean(goal_agent_err**2))*100:.2f} cm")
    print(f"final formation err = {final_formation_err*100:.2f} cm")
    print(f"settling time       = {settling_time:.2f} s (target + formation < 10 cm)")
    print(f"min pair distance   = {min_sep:.2f} m")
    print(f"CBF corrections     = {int(np.sum(cbf_log))}")

    np.savez(
        "swarm_trajectory.npz",
        t=tlog,
        center=center_log,
        center_des=center_des_log,
        pos=pos_log,
        pos_des=pos_des_log,
        pair_dist=pair_log,
        target_err=target_err,
        formation_err=formation_err,
        initial_positions=initial_positions,
        formation_offsets=FORMATION_OFFSETS,
        target_ground=TARGET_GROUND,
        accel=accel_log,
        nominal_accel=nominal_accel_log,
        cbf_active=cbf_log,
        phase=phase_log,
    )
    _plot(
        tlog,
        center_log,
        center_des_log,
        pos_log,
        pos_des_log,
        pair_log,
        target_err,
        formation_err,
        initial_positions,
        phase_log,
        nominal_accel_log,
        accel_log,
    )
    return rms_center


def _plot(
    t,
    center,
    center_des,
    pos,
    pos_des,
    pair_dist,
    target_err,
    formation_err,
    initial_positions,
    phase,
    nominal_accel,
    safe_accel,
):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception:
        print("(matplotlib not installed — skipping plot; see swarm_trajectory.npz)")
        return

    fig3 = plt.figure(figsize=(11, 5))
    ax3d = fig3.add_subplot(1, 2, 1, projection="3d")
    axxy = fig3.add_subplot(1, 2, 2)

    colors = ["C0", "C1", "C2"]
    for i, c in enumerate(colors):
        ax3d.plot(pos[:, i, 0], pos[:, i, 1], pos[:, i, 2], color=c, lw=1.7, label=f"uav {i} actual")
        ax3d.plot(pos_des[:, i, 0], pos_des[:, i, 1], pos_des[:, i, 2], color=c, lw=1.0, ls="--", alpha=0.75, label=f"uav {i} desired")
        ax3d.scatter(
            [initial_positions[i, 0]],
            [initial_positions[i, 1]],
            [initial_positions[i, 2]],
            color=c,
            s=28,
            marker="o",
        )
        axxy.plot(pos[:, i, 0], pos[:, i, 1], color=c, lw=1.7, label=f"uav {i} actual")
        axxy.plot(pos_des[:, i, 0], pos_des[:, i, 1], color=c, lw=1.0, ls="--", alpha=0.75, label=f"uav {i} desired")
        axxy.scatter([initial_positions[i, 0]], [initial_positions[i, 1]], color=c, s=28, marker="o")

    ax3d.scatter([TARGET_GROUND[0]], [TARGET_GROUND[1]], [0.0], c="crimson", s=55, marker="x", label="ground target")
    ax3d.scatter([TARGET_AIR[0]], [TARGET_AIR[1]], [TARGET_AIR[2]], c="crimson", s=45, marker="*", label="target hover point")
    axxy.scatter([TARGET_GROUND[0]], [TARGET_GROUND[1]], c="crimson", s=55, marker="x", label="ground target")
    ax3d.set_title("3D swarm trajectories: ground takeoff + DMPC")
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("z [m]")
    ax3d.view_init(elev=24, azim=-58)
    ax3d.grid(True, alpha=0.3)
    ax3d.legend(loc="upper left", fontsize=7)
    axxy.set_title("Top-down view")
    axxy.set_xlabel("x [m]")
    axxy.set_ylabel("y [m]")
    axxy.set_aspect("equal", adjustable="box")
    axxy.grid(True, alpha=0.3)
    axxy.legend(loc="upper left", fontsize=7)
    plt.tight_layout()
    plt.savefig("swarm_paths.png", dpi=140)
    print("saved swarm_paths.png")

    fig, ax = plt.subplots(5, 1, figsize=(8, 9), sharex=True)
    labels = ["x", "y", "z"]
    for i in range(3):
        ax[i].plot(t, center_des[:, i], "k--", lw=1.0, label="desired")
        ax[i].plot(t, center[:, i], "C0", lw=1.2, label="actual")
        ax[i].set_ylabel(f"{labels[i]} [m]")
        ax[i].grid(True, alpha=0.3)
        ax[i].legend(loc="upper right", fontsize=8)
    ax[3].plot(t, pair_dist[:, 0], label="d01")
    ax[3].plot(t, pair_dist[:, 1], label="d02")
    ax[3].plot(t, pair_dist[:, 2], label="d12")
    ax[3].axhline(CBF_SAFE_DISTANCE, color="crimson", ls="--", lw=0.9, label="CBF limit")
    ax[3].set_ylabel("pair [m]")
    ax[3].grid(True, alpha=0.3)
    ax[3].legend(loc="upper right", fontsize=8)
    accel_norm_nominal = np.linalg.norm(nominal_accel, axis=2).mean(axis=1)
    accel_norm_safe = np.linalg.norm(safe_accel, axis=2).mean(axis=1)
    ax[4].plot(t, accel_norm_nominal, color="C1", lw=1.1, label="DMPC nominal")
    ax[4].plot(t, accel_norm_safe, color="C2", lw=1.1, label="after CBF + jerk")
    ax[4].set_ylabel("|a| [m/s2]")
    ax[4].set_xlabel("time [s]")
    ax[4].grid(True, alpha=0.3)
    ax[4].legend(loc="upper right", fontsize=8)
    for axis in ax:
        axis.axvline(TAKEOFF_DURATION, color="0.35", ls=":", lw=0.8)
        axis.axvline(TAKEOFF_DURATION + TRANSIT_DURATION, color="0.35", ls=":", lw=0.8)
    ax[0].set_title("DMPC convergence with smooth takeoff and CBF safety")
    plt.tight_layout()
    plt.savefig("swarm_trajectory.png", dpi=120)
    print("saved swarm_trajectory.png")

    fig2, ax2 = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    ax2[0].plot(t, target_err, color="crimson", lw=1.5)
    ax2[0].axhline(0.10, color="k", ls="--", lw=0.9, alpha=0.7)
    ax2[0].axvline(TAKEOFF_DURATION, color="0.35", ls=":", lw=0.8)
    ax2[0].axvline(TAKEOFF_DURATION + TRANSIT_DURATION, color="0.35", ls=":", lw=0.8)
    ax2[0].set_ylabel("center error [m]")
    ax2[0].set_title("Convergence diagnostics")
    ax2[0].grid(True, alpha=0.3)
    ax2[1].plot(t, formation_err, color="C4", lw=1.5)
    ax2[1].axhline(0.10, color="k", ls="--", lw=0.9, alpha=0.7)
    ax2[1].axvline(TAKEOFF_DURATION, color="0.35", ls=":", lw=0.8)
    ax2[1].axvline(TAKEOFF_DURATION + TRANSIT_DURATION, color="0.35", ls=":", lw=0.8)
    ax2[1].set_ylabel("formation error [m]")
    ax2[1].set_xlabel("time [s]")
    ax2[1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("swarm_convergence.png", dpi=140)
    print("saved swarm_convergence.png")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=14.0)
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="render playback speed multiplier; 0.5 is half speed")
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED,
                    help="random seed for reproducible initial positions")
    args = ap.parse_args()
    run(args.T, args.dt, args.render, args.speed, args.seed)
