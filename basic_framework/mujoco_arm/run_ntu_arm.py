from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

try:
    import mujoco
    import mujoco.viewer
except ImportError as exc:
    raise SystemExit(
        "MuJoCo python package is not installed. Install it with: pip install mujoco"
    ) from exc


XML_PATH = Path(__file__).with_name("robticarm.xml")

HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

DEFAULT_SERVO_KP = 18.0
DEFAULT_SERVO_KV = 5.0
DEFAULT_SERVO_FORCE = 24.0

# User-facing coordinates use the bottom center of joint1 as the world origin.
SCRIPT_TARGET_WORLD = (0.85, 0.22, 0.5)
JOINT1_BOTTOM_WORLD_POS = (0.0, 0.0, 0.0)
J1_CENTER_WORLD_POS = (0.0, 0.0, 0.045)
J2_HEIGHT_FROM_J1_CENTER = 0.135
J2_WORLD_Z = J1_CENTER_WORLD_POS[2] + J2_HEIGHT_FROM_J1_CENTER
UPPER_ARM_LEN = 0.304
TCP_LEN_FROM_J3 = 0.3498 + 0.105 + 0.085 + 0.190

PICK_BLOCK_START_WORLD = (0.55, -0.22, 0.175)
PICK_BLOCK_PLACE_WORLD = (0.46, 0.30, 0.175)
PICK_GRIPPER_OFFSET = 0.042
PICK_DEMO_DURATION = 14.0
PICK_GRASP_TIME = 4.65
PICK_RELEASE_TIME = 10.25
PICK_LIFT_HEIGHT = 0.14
GRIPPER_OPEN = 0.018
GRIPPER_CLOSED = 0.001


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def smoothstep(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def lerp(a: float, b: float, s: float) -> float:
    return a + s * (b - a)


def world_from_user(pos_user: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        pos_user[0] + JOINT1_BOTTOM_WORLD_POS[0],
        pos_user[1] + JOINT1_BOTTOM_WORLD_POS[1],
        pos_user[2] + JOINT1_BOTTOM_WORLD_POS[2],
    )


def user_from_world(pos_world) -> tuple[float, float, float]:
    return (
        float(pos_world[0] - JOINT1_BOTTOM_WORLD_POS[0]),
        float(pos_world[1] - JOINT1_BOTTOM_WORLD_POS[1]),
        float(pos_world[2] - JOINT1_BOTTOM_WORLD_POS[2]),
    )


def add_pos(a: tuple[float, float, float],
            b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def target_to_tcp_target_world(target_world: tuple[float, float, float],
                               offset: float = PICK_GRIPPER_OFFSET) -> tuple[float, float, float]:
    radial = math.hypot(target_world[0], target_world[1])
    if radial < 1e-6:
        return (target_world[0] + offset, target_world[1], target_world[2])
    return (
        target_world[0] + offset * target_world[0] / radial,
        target_world[1] + offset * target_world[1] / radial,
        target_world[2],
    )


def ik_target_world(target_world: tuple[float, float, float]) -> list[float]:
    x, y, z = target_world
    q1 = math.atan2(y, x)
    radial = math.hypot(x, y)
    z_from_j2 = z - J2_WORLD_Z

    l1 = UPPER_ARM_LEN
    l2 = TCP_LEN_FROM_J3
    dist2 = radial * radial + z_from_j2 * z_from_j2
    cos_elbow = (dist2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if cos_elbow < -1.0 or cos_elbow > 1.0:
        raise ValueError(
            f"target {target_world} is outside the simplified arm workspace "
            f"(cos_elbow={cos_elbow:.3f})"
        )

    elbow = -math.acos(clamp(cos_elbow, -1.0, 1.0))
    shoulder_line = math.atan2(z_from_j2, radial)
    shoulder_offset = math.atan2(l2 * math.sin(elbow), l1 + l2 * math.cos(elbow))
    link1_angle = shoulder_line - shoulder_offset

    q2 = math.pi / 2.0 - link1_angle
    q3 = link1_angle - (link1_angle + elbow) - math.pi / 2.0
    q = [q1, q2, q3, 0.0, 0.0, 0.0]
    return clamp_arm_joints(q)


def clamp_arm_joints(q: list[float]) -> list[float]:
    ranges = [
        (-3.14, 3.14),
        (-2.36, 2.36),
        (-2.62, 2.62),
        (-3.14, 3.14),
        (-1.5708, 1.5708),
        (-3.1416, 3.1416),
    ]
    return [clamp(q[i], ranges[i][0], ranges[i][1]) for i in range(6)]


def format_joints_rad_deg(q: list[float]) -> str:
    parts = []
    for i, value in enumerate(q[:6], start=1):
        parts.append(f"J{i}={value:+.4f}rad/{math.degrees(value):+.1f}deg")
    return "  ".join(parts)


def current_arm_joints(model: mujoco.MjModel, data: mujoco.MjData) -> list[float]:
    joints = ["j1_yaw", "j2_pitch", "j3_pitch", "j4_roll", "j5_wrist_pitch", "j6_tool_roll"]
    values = []
    for name in joints:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        values.append(float(data.qpos[model.jnt_qposadr[joint_id]]))
    return values


def current_joint_positions(model: mujoco.MjModel, data: mujoco.MjData) -> list[tuple[str, tuple[float, float, float]]]:
    bodies = [
        ("J1", "j1_yaw_frame"),
        ("J2", "j2_pitch_frame"),
        ("J3", "j3_elbow_frame"),
        ("J4", "wrist_center_frame"),
        ("J5", "j5_pitch_frame"),
        ("J6", "j6_tool_roll_frame"),
    ]
    positions = []
    for label, body_name in bodies:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id >= 0:
            positions.append((label, user_from_world(data.xpos[body_id])))

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    if site_id >= 0:
        positions.append(("TCP", user_from_world(data.site_xpos[site_id])))
    return positions


def format_positions(positions: list[tuple[str, tuple[float, float, float]]]) -> str:
    parts = []
    for label, pos in positions:
        parts.append(f"{label}=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})m")
    return "  ".join(parts)


def format_arm_state(model: mujoco.MjModel, data: mujoco.MjData, elapsed: float) -> str:
    angles = format_joints_rad_deg(current_arm_joints(model, data))
    positions = format_positions(current_joint_positions(model, data))
    return f"t={elapsed:6.2f}s\n  angles:    {angles}\n  positions: {positions}"


def diff_motors_to_virtual_wrist(motor_a_rad: float, motor_b_rad: float,
                                 gear_ratio: float = 36.0,
                                 bevel_ratio: float = 1.0) -> tuple[float, float]:
    """Return virtual J5/J6 from the two M2006 motor angles.

    This mirrors the firmware design: MuJoCo simulates virtual wrist joints, while
    the differential transmission is handled as a control-layer mapping.
    """
    out_a = motor_a_rad / gear_ratio
    out_b = motor_b_rad / gear_ratio
    j5_pitch = 0.5 * (out_a + out_b) * bevel_ratio
    j6_roll = 0.5 * (out_a - out_b) * bevel_ratio
    return j5_pitch, j6_roll


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, q: list[float]) -> None:
    for i, value in enumerate(q):
        if model.jnt_limited[i]:
            low, high = model.jnt_range[i]
            value = clamp(value, low, high)
        data.qpos[i] = value
        data.qvel[i] = 0.0
        if i < model.nu:
            data.ctrl[i] = value
    mujoco.mj_forward(model, data)


def set_arm_ctrl(model: mujoco.MjModel, data: mujoco.MjData, q: list[float]) -> None:
    for i, value in enumerate(q[:6]):
        if model.actuator_ctrllimited[i]:
            low, high = model.actuator_ctrlrange[i]
            value = clamp(value, low, high)
        data.ctrl[i] = value


def set_joint_qpos(model: mujoco.MjModel, data: mujoco.MjData,
                   joint_name: str, value: float) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return
    qpos_addr = model.jnt_qposadr[joint_id]
    qvel_addr = model.jnt_dofadr[joint_id]
    data.qpos[qpos_addr] = value
    data.qvel[qvel_addr] = 0.0


def set_gripper_opening(model: mujoco.MjModel, data: mujoco.MjData, opening: float) -> None:
    opening = clamp(opening, GRIPPER_CLOSED, GRIPPER_OPEN)
    set_joint_qpos(model, data, "gripper_upper_slide", opening)
    set_joint_qpos(model, data, "gripper_lower_slide", opening)


def set_mocap_body_pos(model: mujoco.MjModel, data: mujoco.MjData,
                       body_name: str, pos: tuple[float, float, float]) -> None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return
    mocap_id = model.body_mocapid[body_id]
    if mocap_id < 0:
        return
    data.mocap_pos[mocap_id] = pos
    data.mocap_quat[mocap_id] = (1.0, 0.0, 0.0, 0.0)


def tcp_grasp_pos(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, float, float]:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    if site_id < 0:
        return world_from_user(PICK_BLOCK_START_WORLD)
    tcp_pos = data.site_xpos[site_id]
    tcp_x_axis = data.site_xmat[site_id].reshape(3, 3)[:, 0]
    return (
        float(tcp_pos[0] - PICK_GRIPPER_OFFSET * tcp_x_axis[0]),
        float(tcp_pos[1] - PICK_GRIPPER_OFFSET * tcp_x_axis[1]),
        float(tcp_pos[2] - PICK_GRIPPER_OFFSET * tcp_x_axis[2]),
    )


def build_pick_waypoints() -> list[tuple[float, list[float]]]:
    pick_tcp = target_to_tcp_target_world(PICK_BLOCK_START_WORLD)
    place_tcp = target_to_tcp_target_world(PICK_BLOCK_PLACE_WORLD)
    above_pick_tcp = add_pos(pick_tcp, (0.0, 0.0, PICK_LIFT_HEIGHT))
    above_place_tcp = add_pos(place_tcp, (0.0, 0.0, PICK_LIFT_HEIGHT))
    return [
        (0.0, HOME_POSE),
        (2.4, ik_target_world(above_pick_tcp)),
        (4.1, ik_target_world(pick_tcp)),
        (4.9, ik_target_world(pick_tcp)),
        (6.4, ik_target_world(above_pick_tcp)),
        (8.1, ik_target_world(above_place_tcp)),
        (9.8, ik_target_world(place_tcp)),
        (10.6, ik_target_world(place_tcp)),
        (12.2, ik_target_world(above_place_tcp)),
        (14.0, HOME_POSE),
    ]


def interpolate_waypoints(phase: float, waypoints: list[tuple[float, list[float]]]) -> list[float]:
    for i in range(len(waypoints) - 1):
        t0, q0 = waypoints[i]
        t1, q1 = waypoints[i + 1]
        if phase <= t1:
            s = smoothstep((phase - t0) / (t1 - t0))
            return [lerp(q0[j], q1[j], s) for j in range(6)]
    return waypoints[-1][1]


def pick_demo_pose(phase: float) -> list[float]:
    return interpolate_waypoints(phase, build_pick_waypoints())


def pick_demo_gripper_opening(phase: float) -> float:
    if phase < 3.9:
        return GRIPPER_OPEN
    if phase < PICK_GRASP_TIME:
        s = smoothstep((phase - 3.9) / (PICK_GRASP_TIME - 3.9))
        return lerp(GRIPPER_OPEN, GRIPPER_CLOSED, s)
    if phase < 9.9:
        return GRIPPER_CLOSED
    if phase < PICK_RELEASE_TIME:
        s = smoothstep((phase - 9.9) / (PICK_RELEASE_TIME - 9.9))
        return lerp(GRIPPER_CLOSED, GRIPPER_OPEN, s)
    return GRIPPER_OPEN


def run_pick_demo(model: mujoco.MjModel, data: mujoco.MjData, elapsed: float) -> None:
    phase = elapsed % PICK_DEMO_DURATION
    set_pose(model, data, pick_demo_pose(phase))
    set_gripper_opening(model, data, pick_demo_gripper_opening(phase))

    if PICK_GRASP_TIME <= phase < PICK_RELEASE_TIME:
        set_mocap_body_pos(model, data, "pick_block", tcp_grasp_pos(model, data))
    elif phase >= PICK_RELEASE_TIME:
        set_mocap_body_pos(model, data, "pick_block", world_from_user(PICK_BLOCK_PLACE_WORLD))
    else:
        set_mocap_body_pos(model, data, "pick_block", world_from_user(PICK_BLOCK_START_WORLD))
    mujoco.mj_forward(model, data)


def target_demo_pose(target_world: tuple[float, float, float], elapsed: float,
                     start_q: list[float], move_time: float) -> list[float]:
    target_q = ik_target_world(target_world)
    s = smoothstep(elapsed / move_time)
    return [lerp(start_q[i], target_q[i], s) for i in range(6)]


def configure_position_servos(model: mujoco.MjModel, kp: float, kv: float, force: float) -> None:
    for i in range(model.nu):
        model.actuator_gainprm[i, :] = 0.0
        model.actuator_biasprm[i, :] = 0.0
        model.actuator_gainprm[i, 0] = kp
        model.actuator_biasprm[i, 1] = -kp
        model.actuator_biasprm[i, 2] = -kv
        model.actuator_forcerange[i, 0] = -force
        model.actuator_forcerange[i, 1] = force


def configure_joint_passive_dynamics(model: mujoco.MjModel, damping: float | None,
                                     armature: float | None) -> None:
    if damping is not None:
        model.dof_damping[:] = damping
    if armature is not None:
        model.dof_armature[:] = armature


def disable_position_servos(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    for i in range(model.nu):
        model.actuator_gainprm[i, :] = 0.0
        model.actuator_biasprm[i, :] = 0.0
        model.actuator_forcerange[i, 0] = 0.0
        model.actuator_forcerange[i, 1] = 0.0
        data.ctrl[i] = 0.0


def demo_pose(t: float) -> list[float]:
    return [
        1.25 * math.sin(0.35 * t),
        0.35 * math.sin(0.45 * t + 0.8),
        0.55 * math.sin(0.42 * t),
        1.10 * math.sin(0.70 * t),
        0.75 * math.sin(0.85 * t + 0.5),
        1.55 * math.sin(0.95 * t),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="NTU 6-DOF spherical-wrist arm MuJoCo viewer")
    parser.add_argument("--xml", type=Path, default=XML_PATH, help="MJCF model path")
    parser.add_argument("--demo", action="store_true", help="run a smooth joint-space demo")
    parser.add_argument("--pick-demo", action="store_true",
                        help="run a scripted point-pick and place demonstration")
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="move TCP to a target point in meters, using joint1 bottom as origin")
    parser.add_argument("--script-target", action="store_true",
                        help="move TCP to SCRIPT_TARGET_WORLD configured at the top of this script")
    parser.add_argument("--target-move-time", type=float, default=4.0,
                        help="seconds used to move from the initial pose to --target")
    parser.add_argument("--print-joints", action="store_true",
                        help="print current J1..J6 angles and positions while the viewer is running")
    parser.add_argument("--print-period", type=float, default=0.5,
                        help="seconds between joint-angle printouts")
    parser.add_argument("--pose", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="initial joint pose in radians")
    parser.add_argument("--passive-gravity", action="store_true",
                        help="disable position servos so gravity can move the arm freely")
    parser.add_argument("--servo-kp", type=float, default=DEFAULT_SERVO_KP,
                        help="position-servo stiffness used when --passive-gravity is not set")
    parser.add_argument("--servo-kv", type=float, default=DEFAULT_SERVO_KV,
                        help="position-servo velocity damping used when --passive-gravity is not set")
    parser.add_argument("--servo-force", type=float, default=DEFAULT_SERVO_FORCE,
                        help="position-servo force limit used when --passive-gravity is not set")
    parser.add_argument("--gravity-scale", type=float, default=1.0,
                        help="multiply gravity for visual debugging; keep 1.0 for realistic viewing")
    parser.add_argument("--joint-damping", type=float, default=None,
                        help="override all joint damping values")
    parser.add_argument("--joint-armature", type=float, default=None,
                        help="override all joint armature values")
    args = parser.parse_args()
    target_world = tuple(args.target) if args.target is not None else None
    if args.script_target and target_world is None:
        target_world = SCRIPT_TARGET_WORLD
    if target_world is not None:
        try:
            target_q = ik_target_world(target_world)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
    else:
        target_q = None

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = [0.0, 0.0, -9.8 * args.gravity_scale]
    configure_joint_passive_dynamics(model, args.joint_damping, args.joint_armature)

    if args.passive_gravity and not args.pick_demo and target_world is None:
        disable_position_servos(model, data)
    else:
        configure_position_servos(model, args.servo_kp, args.servo_kv, args.servo_force)

    initial = args.pose if args.pose is not None else HOME_POSE
    set_pose(model, data, list(initial))
    set_gripper_opening(model, data, GRIPPER_OPEN)
    set_mocap_body_pos(model, data, "pick_block", world_from_user(target_world or PICK_BLOCK_START_WORLD))
    if args.passive_gravity and not args.pick_demo and target_world is None:
        disable_position_servos(model, data)

    print("Loaded:", args.xml)
    if args.passive_gravity:
        print(f"Passive gravity mode: position servos are disabled, gravity scale={args.gravity_scale}.")
    else:
        print(f"Use the MuJoCo Control panel sliders named joint1..joint6. servo kp={args.servo_kp}, kv={args.servo_kv}, force={args.servo_force}, gravity scale={args.gravity_scale}")
    if args.pick_demo:
        print("Pick demo: the arm reaches the red block, closes the gripper, lifts it, and places it on the blue pad.")
    if target_world is not None and target_q is not None:
        print(f"Target in joint1-bottom world frame: x={target_world[0]:.3f} m, y={target_world[1]:.3f} m, z={target_world[2]:.3f} m")
        print("IK target joints:", format_joints_rad_deg(target_q))
    print("joint1 is the bottom base yaw axis; its blue shaft and red flag rotate with the base.")

    with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance = 0.8
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        if target_world is not None:
            viewer.cam.lookat[:] = world_from_user(target_world)
        else:
            viewer.cam.lookat[:] = [0.22, 0.0, 0.32]

        start = time.time()
        target_start_q = list(initial[:6])
        last_print = -1.0
        while viewer.is_running():
            loop_start = time.time()
            elapsed = time.time() - start

            if args.pick_demo:
                run_pick_demo(model, data, elapsed)
            elif target_world is not None:
                set_pose(model, data, target_demo_pose(target_world, elapsed, target_start_q,
                                                       args.target_move_time))
            elif args.demo:
                set_pose(model, data, demo_pose(elapsed))

            mujoco.mj_step(model, data)
            if args.pick_demo:
                run_pick_demo(model, data, elapsed)
            if args.print_joints and elapsed - last_print >= args.print_period:
                print(format_arm_state(model, data, elapsed))
                last_print = elapsed
            viewer.sync()

            sleep_time = model.opt.timestep - (time.time() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    main()
