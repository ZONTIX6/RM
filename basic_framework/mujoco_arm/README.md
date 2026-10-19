# NTU Spherical-Wrist Arm MuJoCo Model

@copyright Copyright (c) 2026 NTU NSE

Files:

- `robticarm.xml`: standardized MJCF model of the 6-DOF arm.
- `run_ntu_arm.py`: Python launcher for MuJoCo viewer.

Run:

```powershell
cd basic_framework/mujoco_arm
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe .\run_ntu_arm.py
.\.venv\Scripts\python.exe .\run_ntu_arm.py --demo
.\.venv\Scripts\python.exe .\run_ntu_arm.py --pick-demo
.\.venv\Scripts\python.exe .\run_ntu_arm.py --target 0.55 -0.22 0.27 --print-joints
.\.venv\Scripts\python.exe .\run_ntu_arm.py --script-target --print-joints
.\.venv\Scripts\python.exe .\run_ntu_arm.py --repeat-target --print-joints
.\.venv\Scripts\python.exe .\run_ntu_arm.py --passive-gravity --pose 0 -0.8 1.0 0 0 0
.\.venv\Scripts\python.exe .\run_ntu_arm.py --servo-kp 18 --servo-kv 5 --servo-force 24
```

You can also double-click `run_pick_demo.bat` to start the pick-and-place demo, or `run_repeat_target.bat` to run the repeated target motion.

If Python cannot import MuJoCo:

```powershell
python -m pip install -r requirements.txt
```

Model convention:

- Units are meters in MuJoCo.
- This XML exposes six position-control sliders named `joint1` through `joint6`.
- Use `.\.venv\Scripts\python.exe .\run_ntu_arm.py --pick-demo` to watch the arm pick the red block from the first pad and place it on the second pad.
- Use `--target X Y Z` to move the TCP to a point in meters. The target origin is the bottom center of joint1.
- Use `--script-target` to move to `SCRIPT_TARGET_WORLD` configured near the top of `run_ntu_arm.py`.
- Use `--repeat-target` to move back and forth between the initial pose and the target pose. It uses `SCRIPT_TARGET_WORLD` unless `--target X Y Z` is also provided.
- If both `--target X Y Z` and `--script-target` are present, the command-line `--target` value wins.
- Use `--print-joints` to print the current J1..J6 angles plus J1..J6/TCP positions.
- `--target` and `--pick-demo` use scripted joint-position trajectories so the displayed pose and printed joint angles stay exact and do not depend on soft slider servo tuning.
- Gravity is enabled in the XML. In the default viewer mode, the position actuators hold the arm at the slider targets, so gravity is mostly cancelled by the servos.
- Use `--passive-gravity` to disable the position servos and watch the arm move under gravity.
- Keep `--gravity-scale` at its default `1.0` for realistic viewing. Larger values, such as `--gravity-scale 3`, are only useful for debugging.
- The default position servo is intentionally damped (`kp=18`, `kv=5`, `force=24`) so dragging sliders does not excite visible oscillation.
- Use lower `--servo-kp` and `--servo-force` values if you want the control sliders to stay active but feel softer under gravity.
- The MuJoCo world origin is on the floor. J1 is at the bottom rotating base, and J2 is the shoulder pitch axis above it.
- Default ready pose is `q = [0, -0.9, 0.9, 1.5708, 0, 0]`.
- In the default ready pose, the upper arm points backward, the forearm is horizontal, and the J5 axis is vertical.
- J1 is the bottom base yaw axis. J2 is the shoulder pitch axis above the rotating column.
- The J1 motor is modeled as a horizontal ground-mounted motor on the fixed base, driving the yaw pulley through a belt.
- J2 and J3 are parallel pitch axes.
- J2 to J3 distance is `0.304 m`.
- Wrist center offset from J3 is `0.3498 m`, matching the CAD/URDF-derived J3-to-J4 distance.
- J4 is modeled as the coaxial forearm roll motor at the wrist center.
- The wrist is now kept as a minimal connected physical chain: J4 roll, a short wrist link, J5 pitch, a short tool link, then J6 roll and the gripper.
- The physical M2006 differential mapping belongs to the firmware control layer, matching the `diff_wrist` module. This simplified MuJoCo model exposes only the six virtual output joints.

Important:

- In the MuJoCo viewer, use the Control panel sliders named `joint1` to `joint6`.
- `joint1` / `j1_yaw` is the bottom base rotation joint.
- The model uses simplified engineering-style geometry rather than CAD meshes, so it is suitable for checking motion, joint directions, and approximate workspace.
