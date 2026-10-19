# 无人机集群 MuJoCo 仿真指南（基于 simulate.py 扩展）

> 适用对象：已经跑通 `simulate.py`（DJI Phantom 4 Pro 单机仿真），想把仿真扩展成 **N 架无人机组成的集群**的新手。
> 本文档以你现有的 `simulate.py` + `phantom4pro.xml` 为起点，一步步讲清楚：**要改哪些配置、集群算法写在哪里、怎么上手跑起来**。

---

## 0. 这份文档怎么用

按顺序做，每一步都有可直接复制的代码：

| 步骤 | 内容 | 你得到的成果 |
|---|---|---|
| 第 1 步 | 多机模型配置（XML） | 一份包含 N 架无人机的模型 |
| 第 2 步 | 代码改造（索引 + 控制循环） | 让 N 架无人机各自飞起来 |
| 第 3 步 | 集群算法（编队 / 避碰 / 通信） | 集群行为（编队、防撞、一致性） |
| 第 4 步 | 参数速查 + 完整代码骨架 | 一份可直接跑的 `simulate_swarm.py` |

**核心思想一句话**：集群仿真 = "单机模型复制 N 份（改名字、错开位置）" + "单机控制器循环跑 N 次" + "在每架的控制输入上加集群项（编队偏差、避碰排斥力）"。

---

## 1. 总览：从单机到集群，到底要改什么

对照现有代码，一张表看全所有改动点：

| 配置/实现 | 单机现状 | 集群要做的事 | 改哪里 |
|---|---|---|---|
| 模型文件 | `phantom4pro.xml`（1 架） | 生成含 N 架的多机 XML（每架一个 `freejoint` body） | 第 2 节 |
| 命名 | body 名 `quadrotor`、关节 `root`、`rotor0_spin`… | 全部加编号后缀，保证**全局唯一**（`quadrotor_0`、`rotor1_spin_2`…） | 第 2 节 |
| 初始位置 | 单机 `pos="0 0 0.35"` | 每架错开（如沿 x 方向间隔 1.5 m），避免初始重叠 | 第 2 节 |
| 碰撞 | 默认 `contype="0"`（全模型无碰撞） | 新手阶段**保持无物理碰撞**，只做逻辑避碰；进阶再开碰撞 | 第 2.3 节 |
| 物理参数 | 标量（MASS、KF…） | 可逐机不同（数组）或全机共用（最简单） | 第 4 节 |
| 控制器 | `control()` 单机 | 每机独立调用一次（`for` 循环或向量化） | 第 3 节 |
| 轨迹 | `waypoint(t)` 单条 | 编队轨迹：leader 轨迹 + 每机偏移量 | 第 3.1 节 |
| 集群算法 | 无 | 编队控制、碰撞避免、一致性、通信拓扑 | 第 3 节 |
| 数据记录 | 二维数组 `(steps, 3)` | 三维数组 `(steps, N, 3)` | 第 3 节 |
| 渲染 | viewer 显示 1 架 | 无需改动，viewer 自动渲染模型里所有 body | — |

---

## 2. 第一步：多机模型配置（XML）

MuJoCo 一个模型里可以有多个 `freejoint` 刚体，**不需要**启动多个 MuJoCo 实例。要做的是把 `quadrotor` body 复制 N 份，每份改名、错开初始位置。

### 2.1 推荐方案：脚本生成多机 XML（最不容易出错）

手写 N 份 XML 容易抄错，推荐用 Python 生成。下面用 `MjModel.from_xml_string()` 直接把生成的 XML 字符串加载进 MuJoCo，**不需要额外生成 .xml 文件**。

> 说明：为了让你看清结构，下面模板用的是"简化外观"（一个机身椭球 + 4 个视觉桨叶关节 + 一个碰撞保护球）。如果你想要完整的大疆外观，把 `phantom4pro.xml` 里 `<body name="quadrotor">...</body>` 的全部内容复制进下面的 `agent_body` 模板字符串即可（记得把内部名字也加 `_{k}` 后缀）。

```python
import numpy as np
import mujoco

def build_swarm_xml(n: int, spacing: float = 1.5) -> str:
    """生成 n 架无人机的 MuJoCo XML 字符串。

    spacing : 相邻无人机初始位置的水平间隔（米），防止初始重叠。
    命名规则：quadrotor_0..quadrotor_{n-1}，内部关节/几何全部带编号后缀。
    """
    # 单架无人机的 XML 模板。{} 占位符会被 format 填入编号。
    agent_body = """
    <body name="quadrotor_{k}" pos="{x} 0 0.35">
      <freejoint name="root_{k}"/>
      <!-- 质量/惯量沿用原模型标定值 -->
      <inertial pos="0 0 0" mass="1.388" diaginertia="0.010 0.010 0.018"/>

      <!-- 简化机身（视觉） -->
      <geom name="shell_{k}" type="ellipsoid" pos="0 0 0.018"
            size="0.082 0.070 0.028" rgba="0.93 0.94 0.93 1"/>
      <!-- 碰撞保护球：只在你想开物理碰撞时生效（见 2.3），默认不参与碰撞 -->
      <geom name="collide_{k}" type="sphere" pos="0 0 0" size="0.25"
            contype="1" conaffinity="1" mass="0.0"/>

      <!-- 4 个视觉桨叶关节（保持与 simulate.py 的 update_rotor_visuals 对应） -->
      <body name="prop0_body_{k}" pos="0.12374 -0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor0_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop0_{k}" type="box" pos="0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
      <body name="prop1_body_{k}" pos="-0.12374 -0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor1_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop1_{k}" type="box" pos="-0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
      <body name="prop2_body_{k}" pos="-0.12374 0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor2_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop2_{k}" type="box" pos="0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
      <body name="prop3_body_{k}" pos="0.12374 0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor3_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop3_{k}" type="box" pos="-0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
    </body>
    """

    # 把 n 架无人机按一条直线排开：第 k 架在 x = (k - (n-1)/2) * spacing
    agents = []
    for k in range(n):
        x = (k - (n - 1) / 2.0) * spacing
        agents.append(agent_body.format(k=k, x=f"{x:.3f}"))

    return f"""<mujoco model="phantom_swarm">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <default>
    <geom contype="0" conaffinity="0" density="0"/>
    <joint damping="0" frictionloss="0" armature="0"/>
  </default>
  <worldbody>
    <light name="key" pos="0 -3 5" dir="0 1 -1" diffuse="0.8 0.8 0.78"/>
    <geom name="ground" type="plane" size="20 20 0.05" pos="0 0 -0.05"
          rgba="0.74 0.76 0.78 1"/>
    {''.join(agents)}
  </worldbody>
</mujoco>
"""

# 用法：直接生成并加载模型
# model = mujoco.MjModel.from_xml_string(build_swarm_xml(3))
```

**为什么命名要全局唯一？** 因为代码里用 `mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor_0")` 按名字查找 body。名字重复或拼错，`mj_name2id` 会返回 **-1**，这是新手最常见的报错来源之一。

### 2.2 各架无人机的初始位置

上面模板把第 k 架放在 `x = (k - (n-1)/2) * spacing`，即 3 架时位置为 `(-1.5, 0, 0.35)`、`(0, 0, 0.35)`、`(1.5, 0, 0.35)`，对称排开。**初始位置必须错开**，否则起飞瞬间多机状态完全重合，视觉上像一架，避碰算法也会在 t=0 就触发。

### 2.3 碰撞：先关掉物理碰撞，再考虑打开

你现在的 `phantom4pro.xml` 里 `<default><geom contype="0" conaffinity="0"/></default>`，意思是**所有几何体都不参与碰撞**——无人机可以互相穿过、穿过地面。这其实很省心：

- **新手阶段（推荐）**：保持无物理碰撞，机间防撞用**逻辑避碰**（第 3.3 节的人工势场），避免多机接触带来的物理抖动和不稳定。
- **进阶阶段**：需要物理碰撞时，给每架加一个 `contype="1" conaffinity="1"` 的碰撞几何（上面模板里已经预留了 `collide_{k}` 球体，半径 0.25 m 约等于整机包络），同时把地面 `ground` 也设为可碰撞（`contype="1" conaffinity="1"`）。这样机机、机地都会发生真实接触力。
  - 注意：物理碰撞会让仿真变慢、可能出现接触抖动；且 `collide` 球的 `mass="0.0"` 表示它不贡献质量（质量仍在 `inertial` 里）。

---

## 3. 第二步：代码改造 —— 从单机循环到集群循环

### 3.1 多机索引 helper（最重要的一段）

MuJoCo 把 N 个 `freejoint` 刚体的状态**顺序拼接**在 `qpos` / `qvel` 里：

- 每架无人机占用 `qpos` 的 **7 个数**：`[x, y, z, qw, qx, qy, qz]`（位置 + 四元数）
- 占用 `qvel` 的 **6 个数**：`[vx, vy, vz, ωx, ωy, ωz]`（线速度 + 机体角速度）

写一个 helper 一次性算好每架的切片，后面所有代码都用它，不会索引错：

```python
def get_agent_slices(model, n: int):
    """返回每架无人机的 (qpos切片, qvel切片, body_id)。

    用法：for q_sl, v_sl, bid in slices: pos = data.qpos[q_sl] ...
    """
    slices = []
    for k in range(n):
        name = f"quadrotor_{k}"
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"模型里找不到 body: {name}")
        joint_id = model.body_jntadr[body_id]      # 该 body 的第一个关节（freejoint）
        q0 = model.jnt_qposadr[joint_id]           # 该关节在 qpos 里的起始下标
        v0 = model.jnt_dofadr[joint_id]            # 该关节在 qvel 里的起始下标
        slices.append((slice(q0, q0 + 7), slice(v0, v0 + 6), body_id))
    return slices
```

### 3.2 把单机控制器改造成"每机一个"

原 `control()` 从 `data.qpos` 读单机状态。集群版建议把它改成**纯函数**：输入每机的状态向量，输出该机的转速。这样主循环 `for k in range(n)` 调用即可，逻辑不变：

```python
def control_agent(pos, quat, vel, w, pos_d, vel_d, acc_d, yaw_d, omega, dt, P):
    """单架无人机的几何控制器（逻辑与 simulate.py 的 control 完全一致）。

    P : dict，装着所有物理参数（见第 4 节参数表），方便逐机不同。
    """
    R = quat2mat(quat)                      # 机体→世界旋转矩阵
    # —— 位置环 ——
    a_d = -P["KP_POS"] * (pos - pos_d) - P["KD_POS"] * (vel - vel_d) + acc_d
    F   = P["MASS"] * (a_d + np.array([0.0, 0.0, P["GRAVITY"]]))
    zBd = F / np.linalg.norm(F)
    f   = max(float(F @ R[:, 2]), 0.0)
    # —— 姿态环 ——
    xc  = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
    yBd = np.cross(zBd, xc); yBd /= np.linalg.norm(yBd)
    xBd = np.cross(yBd, zBd)
    Rd  = np.column_stack([xBd, yBd, zBd])
    eR  = 0.5 * vee(Rd.T @ R - R.T @ Rd)
    tau = -P["KR"] * eR - P["KW"] * w
    # —— 控制分配 ——
    w2 = P["B_INV"] @ np.array([f, tau[0], tau[1], tau[2]])
    w2 = np.clip(w2, 0.0, None)
    w_cmd = np.sqrt(w2)
    # —— 电机一阶响应 + 饱和 ——
    omega = omega + (dt / P["TAU_MOTOR"]) * (w_cmd - omega)
    return np.clip(omega, 0.0, P["OMEGA_MAX"])
```

对应地，`apply_forces_agent` 每机调用一次，把力写进自己的 `data.xfrc_applied[body_id]`：

```python
def apply_forces_agent(data, body_id, pos, quat, vel, w, omega, P):
    """把单机的转速变成推力/力矩，写入 xfrc_applied（逻辑同 simulate.py）。"""
    R = quat2mat(quat)
    v_body = R.T @ vel
    F_body = -P["LINEAR_DRAG"] * v_body
    tau_body = -P["ANGULAR_DRAG"] * w
    for i in range(4):
        f = P["KF"] * omega[i] ** 2
        r = np.array([P["ROTOR_XY"][i][0], P["ROTOR_XY"][i][1], P["ROTOR_Z"]])
        F_body += np.array([0.0, 0.0, f])
        tau_body += np.cross(r, np.array([0.0, 0.0, f]))
        tau_body += np.array([0.0, 0.0, P["ROTOR_DIR"][i] * P["KM"] * omega[i] ** 2])
    if P["GYRO"]:
        h = P["I_ROTOR"] * float(np.sum(P["ROTOR_DIR"] * omega)) * np.array([0.0, 0.0, 1.0])
        tau_body += np.cross(h, w)
    data.xfrc_applied[body_id, 0:3] = R @ F_body
    data.xfrc_applied[body_id, 3:6] = R @ tau_body
```

### 3.3 集群主循环（数据记录改成三维）

```python
def run_swarm(n=3, T=16.0, dt=0.005, render=False, spacing=1.5):
    model = mujoco.MjModel.from_xml_string(build_swarm_xml(n, spacing))
    data  = mujoco.MjData(model)
    model.opt.timestep = dt

    P = make_params()                       # 第 4 节的参数打包函数
    slices = get_agent_slices(model, n)     # 每机 (qpos切片, qvel切片, body_id)

    # 每机独立状态：转速 + 视觉桨叶角
    omega = np.full((n, 4), np.sqrt(P["MASS"] * P["GRAVITY"] / (4 * P["KF"])))
    prop_angles = np.zeros((n, 4))

    steps = int(T / dt)
    tlog  = np.zeros(steps)
    plog  = np.zeros((steps, n, 3))     # 实际位置：时刻 × 无人机 × 坐标
    pdlog = np.zeros((steps, n, 3))     # 期望位置：时刻 × 无人机 × 坐标

    if render:
        import time
        from mujoco import viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)

    for k_step in range(steps):
        t = k_step * dt
        data.xfrc_applied[:] = 0.0

        # 收集所有机当前状态（用切片读一次，后面循环用）
        pos = np.stack([data.qpos[q_sl][0:3]  for q_sl, _, _ in slices])
        quat = np.stack([data.qpos[q_sl][3:7] for q_sl, _, _ in slices])
        vel = np.stack([data.qvel[v_sl][0:3]  for _, v_sl, _ in slices])
        w   = np.stack([data.qvel[v_sl][3:6]  for _, v_sl, _ in slices])

        # 编队轨迹：leader 沿原 waypoint 飞，每机加自己的偏移（见第 4 节算法）
        pos_d, vel_d, acc_d, yaw_d = swarm_waypoint(t, n, OFFSETS)

        for k in range(n):
            # 先算避碰修正（用到所有机的位置），再加到控制里
            a_rep = avoidance_acc(pos, k, SAFE_DIST, K_REP)
            omega[k] = control_agent(pos[k], quat[k], vel[k], w[k],
                                     pos_d[k], vel_d[k], acc_d[k] + a_rep, yaw_d[k],
                                     omega[k], dt, P)
            apply_forces_agent(data, slices[k][2], pos[k], quat[k], vel[k], w[k], omega[k], P)
            prop_angles[k] = (prop_angles[k] + P["ROTOR_DIR"] * omega[k] * dt) % (2*np.pi)
            update_rotor_visuals_agent(model, data, k, omega[k], prop_angles[k])

        mujoco.mj_step(model, data)

        tlog[k_step] = t
        plog[k_step] = pos
        pdlog[k_step] = pos_d
        if render:
            viewer.sync()
            time.sleep(dt)

    if render:
        viewer.close()

    # 误差统计（每机一个 RMS）
    err = np.linalg.norm(plog - pdlog, axis=2)          # (steps, n)
    for k in range(n):
        rms = np.sqrt(np.mean(err[:, k] ** 2))
        print(f"UAV {k}: RMS tracking error = {rms*100:.2f} cm")
    np.savez("trajectory_swarm.npz", t=tlog, pos=plog, pos_des=pdlog, err=err)
    return err
```

`update_rotor_visuals_agent` 和原版一样，只是关节名带编号：

```python
def update_rotor_visuals_agent(model, data, k, omega, angles):
    for i in range(4):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"rotor{i}_spin_{k}")
        if jid < 0:
            continue
        data.qpos[model.jnt_qposadr[jid]] = angles[i]
        data.qvel[model.jnt_dofadr[jid]] = ROTOR_DIR[i] * omega[i]
```

---

## 4. 第三步：集群算法（核心增量）

单机控制器不变，集群行为全部体现在"**给每架无人机的期望轨迹/期望加速度加什么**"上。下面从易到难给出 4 种，可叠加使用。

### 4.1 编队轨迹：Leader-Follower（最简单、先跑通这个）

让 1 号机当 leader 沿原 `waypoint(t)` 飞，其余机跟踪"leader 位置 + 固定编队偏移"：

```python
OFFSETS = np.array([            # 每架相对 leader 的编队偏移（米），n 行
    [0.0, 0.0, 0.0],            # UAV 0：leader 自己
    [1.5, 0.0, 0.0],            # UAV 1：leader 右侧 1.5 m
    [-1.5, 0.0, 0.0],           # UAV 2：leader 左侧 1.5 m
])                              # 想要更多无人机就继续加行

def swarm_waypoint(t, n, offsets):
    """每架的目标 = leader 轨迹 + 自己的偏移（速度/加速度/偏航跟 leader）。"""
    pos_l, vel_l, acc_l, yaw_l = waypoint(t)      # 复用原 simulate.py 的 waypoint
    pos_d  = np.tile(pos_l, (n, 1)) + offsets
    vel_d  = np.tile(vel_l, (n, 1))
    acc_d  = np.tile(acc_l, (n, 1))
    yaw_d  = np.full(n, yaw_l)
    return pos_d, vel_d, acc_d, yaw_d
```

> 注意：如果 `n > len(OFFSETS)`，`offsets` 会索引越界——要么让 `OFFSETS` 按 `n` 生成（如 `np.column_stack([np.arange(n)*spacing - ..., zeros])`），要么在代码里断言 `len(OFFSETS) >= n`。

### 4.2 一致性控制（Consensus）：让集群自己"对齐"

不指定 leader，而是每架根据**邻居状态**动态调整，最终整个集群收敛到一致运动。对二阶系统（位置+速度）的经典一致性：

```
期望加速度 a_i = a_ref + Σ_j a_ij · [ Kp·(p_j − p_i) + Kd·(v_j − v_i) ]
```

其中 `a_ij` 是通信邻接矩阵的元素（i 能收到 j 的信息则为 1）。实现：

```python
def consensus_acc(pos, vel, i, A, KP_CONS=1.0, KD_CONS=2.0):
    """给第 i 架的期望加速度加上一致性修正项（返回 3 维向量）。"""
    a = np.zeros(3)
    for j in range(len(A)):
        if A[i, j] > 0 and j != i:
            a += KP_CONS * (pos[j] - pos[i]) + KD_CONS * (vel[j] - vel[i])
    return a
```

在主循环里：`acc_d[k] = acc_ref + consensus_acc(pos, vel, k, A)`。

**结合编队**：把上面公式里的 `p_j` 换成"去偏移后的位置" `p_j − offset_j`，就能让集群既保持队形又一致性收敛：
`a_i += Σ_j a_ij · [Kp·((p_j−off_j) − (p_i−off_i)) + Kd·(v_j − v_i)]`。

### 4.3 碰撞避免：人工势场（APF）

最实用的做法是给每架的**期望加速度**加一个排斥项：当两机距离小于安全距离时产生互相推开的力，距离越近推力越大：

```python
SAFE_DIST = 0.8     # 安全距离（米）：小于此距离开始排斥
K_REP     = 1.2     # 排斥强度（越大越"硬"）

def avoidance_acc(pos, i, safe_dist=SAFE_DIST, k_rep=K_REP):
    """第 i 架收到的避碰加速度（来自所有其他无人机）。"""
    a = np.zeros(3)
    for j in range(len(pos)):
        if j == i:
            continue
        d = pos[i] - pos[j]            # 指向远离 j 的方向
        dist = np.linalg.norm(d)
        if 0 < dist < safe_dist:
            # 简化势场梯度：距离越近，排斥越强（1/dist 项）
            a += k_rep * (1.0 / dist - 1.0 / safe_dist) / (dist * dist) * d
    return a
```

主循环里的接入方式（第 3.3 节已经写了）：

```python
a_rep = avoidance_acc(pos, k, SAFE_DIST, K_REP)
omega[k] = control_agent(..., acc_d[k] + a_rep, ...)   # 加在期望加速度上
```

**调参直觉**：`SAFE_DIST` 应明显大于编队间距的邻居距离（否则编队内部会一直互相排斥打架）；`K_REP` 从 1.0 起步，抖动就减小、穿模就增大。

### 4.4 通信拓扑：邻接矩阵与连通性检查

集群算法（尤其一致性）依赖"谁和谁通信"。用一个 N×N 邻接矩阵描述：

```python
# 示例：环形拓扑（每架只跟左右邻居通信）
A = np.array([
    [0, 1, 0, 1],
    [1, 0, 1, 0],
    [0, 1, 0, 1],
    [1, 0, 1, 0],
])

def check_connected(A):
    """用拉普拉斯矩阵的第二小特征值判断拓扑是否连通（>0 即连通）。"""
    L = np.diag(A.sum(axis=1)) - A
    eig = np.linalg.eigvalsh(L)
    return eig[1] > 1e-8

print("拓扑连通:", check_connected(A))
```

**新手建议**：先全连通（`A` 全 1）跑通，再试环形/链式拓扑观察差异。一致性算法在不连通的拓扑下无法收敛，这是理论结论（代数连通度>0 才收敛）。

### 4.5 算法选择速查

| 需求 | 用哪个 | 加在哪里 |
|---|---|---|
| 各机各飞各的轨迹 | 复用 `waypoint`，每机不同 | `pos_d` |
| 保持固定队形巡航 | Leader-Follower 偏移 | `pos_d` |
| 集群自主对齐、无 leader | 一致性控制 | `acc_d` |
| 防止互相撞上 | 人工势场 | `acc_d`（叠加） |
| 模拟通信限制 | 邻接矩阵控制一致性求和范围 | `A` |

---

## 5. 第四步：关键参数速查表

### 5.1 单机物理参数（沿用 simulate.py，打包进 dict）

| 参数 | 含义 | 建议初值 | 调参方向 |
|---|---|---|---|
| `GRAVITY` | 重力加速度 | 9.81 | 不要动 |
| `MASS` | 单机质量 kg | 1.388 | 变重→响应变慢，需同步增大 KP |
| `KF` | 推力系数 | 1.0e-5 | 影响悬停转速，一般不动 |
| `KM` | 反扭矩系数 | 1.5e-7 | 一般不动 |
| `OMEGA_MAX` | 最大转速 rad/s | 8500rpm | 电机饱和上限 |
| `TAU_MOTOR` | 电机时间常数 | 0.03 | 调大→电机更"肉"，更容易跟踪迟钝 |
| `KP_POS` | 位置比例增益 | [3,3,6] | 调大→跟踪更紧但可能抖动 |
| `KD_POS` | 位置微分增益 | [2.5,2.5,4] | 与 KP 配比，抑制超调 |
| `KR` / `KW` | 姿态环增益 | 2.0 / 0.4 | 姿态响应快慢 |

### 5.2 集群新增参数

| 参数 | 含义 | 建议初值 | 放哪 / 怎么调 |
|---|---|---|---|
| `n` | 无人机数量 | 3 起步 | 命令行 `--N 3` |
| `spacing` | 初始位置间隔 m | 1.5 | 大于机身直径即可 |
| `OFFSETS` | 编队偏移（n×3） | 见 4.1 | 决定队形形状 |
| `SAFE_DIST` | 避碰安全距离 m | 0.8 | 必须大于期望的机间最小间距 |
| `K_REP` | 避碰排斥强度 | 1.2 | 抖动减小、穿模增大 |
| `KP_CONS` / `KD_CONS` | 一致性增益 | 1.0 / 2.0 | 过大→振荡，过小→收敛慢 |
| `A` | 通信邻接矩阵 | 全连通起步 | 保持拓扑连通（见 4.4） |

### 5.3 参数打包函数（多机版建议这样组织）

```python
def make_params(**overrides):
    """把第 2 节的所有物理参数打包成 dict，方便逐机覆盖。"""
    _HALF = L / np.sqrt(2.0)
    ROTOR_XY = np.array([[_HALF,-_HALF],[-_HALF,-_HALF],[-_HALF,_HALF],[_HALF,_HALF]])
    ROTOR_DIR = np.array([-1.0, 1.0, -1.0, 1.0])
    B = np.zeros((4, 4))
    B[0, :] = KF
    for i in range(4):
        x, y = ROTOR_XY[i]
        B[1, i] =  y * KF; B[2, i] = -x * KF; B[3, i] = ROTOR_DIR[i] * KM
    params = dict(
        GRAVITY=GRAVITY, MASS=MASS, KF=KF, KM=KM, L=L, ROTOR_Z=ROTOR_Z,
        OMEGA_MAX=OMEGA_MAX, TAU_MOTOR=TAU_MOTOR, I_ROTOR=I_ROTOR, GYRO=GYRO,
        LINEAR_DRAG=LINEAR_DRAG, ANGULAR_DRAG=ANGULAR_DRAG,
        ROTOR_XY=ROTOR_XY, ROTOR_DIR=ROTOR_DIR, B_INV=np.linalg.inv(B),
        KP_POS=KP_POS, KD_POS=KD_POS, KR=KR, KW=KW,
    )
    params.update(overrides)      # 支持逐机覆盖，如 make_params(MASS=1.5)
    return params
```

> **逐机异构**：想让某架更重/更快，就给它单独 `P_i = make_params(MASS=1.6)`，在 `control_agent` 调用时传各自的 `P`。这是集群仿真里模拟"载荷不同"的标准做法。

---

## 6. 完整可运行骨架：simulate_swarm.py

把 2–4 节的代码按下面顺序拼起来，就是一份可直接运行的集群仿真：

```python
# simulate_swarm.py —— 无人机集群 MuJoCo 仿真（从 simulate.py 扩展）
# 运行：python simulate_swarm.py --N 3 --render
import argparse
import numpy as np
import mujoco

# ===== 0. 单机物理参数（与 simulate.py 完全一致）=====
GRAVITY = 9.81; MASS = 1.388; KF = 1.0e-5; KM = 1.5e-7
L = 0.175; ROTOR_Z = 0.03
OMEGA_MAX = 8500.0 * 2.0 * np.pi / 60.0
TAU_MOTOR = 0.03; I_ROTOR = 5.0e-5; GYRO = True
LINEAR_DRAG  = np.array([0.18, 0.18, 0.28])
ANGULAR_DRAG = np.array([0.018, 0.018, 0.025])
KP_POS = np.array([3.0, 3.0, 6.0]); KD_POS = np.array([2.5, 2.5, 4.0])
KR = 2.0; KW = 0.4

# ===== 1. 集群参数 =====
SAFE_DIST = 0.8      # 避碰安全距离
K_REP     = 1.2      # 避碰排斥强度
OFFSETS   = None     # 由 n 自动生成（见下方 make_offsets）

# ===== 2. 基础工具（原样复制自 simulate.py）=====
def quat2mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])

def vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])

def _min_jerk_segment(t, t0, t1, p0, p1):
    duration = t1 - t0
    s = np.clip((t - t0) / duration, 0.0, 1.0)
    h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
    hd = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / duration
    hdd = (60.0*s - 180.0*s**2 + 120.0*s**3) / (duration**2)
    delta = p1 - p0
    return p0 + h*delta, hd*delta, hdd*delta

def _arc_segment(t, t0, t1, center, radius, z):
    duration = t1 - t0
    s = np.clip((t - t0) / duration, 0.0, 1.0)
    h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
    hd = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / duration
    hdd = (60.0*s - 180.0*s**2 + 120.0*s**3) / (duration**2)
    theta = np.pi * (1.0 - h); theta_d = -np.pi * hd; theta_dd = -np.pi * hdd
    d1 = np.array([-radius*np.sin(theta), radius*np.cos(theta), 0.0])
    d2 = np.array([-radius*np.cos(theta), -radius*np.sin(theta), 0.0])
    pos = np.array([center[0]+radius*np.cos(theta), center[1]+radius*np.sin(theta), z])
    vel = d1 * theta_d
    acc = d2 * theta_d**2 + d1 * theta_dd
    return pos, vel, acc, theta - 0.5*np.pi

def waypoint(t):
    """leader 的参考轨迹（与 simulate.py 相同）。"""
    takeoff_z = 1.25; land_z = 0.35
    cruise = np.array([0.0, 0.0, takeoff_z]); arc_end = np.array([2.0, 0.0, takeoff_z])
    landing = np.array([2.0, 0.0, land_z])
    if t < 2.0:
        pos, vel, acc = _min_jerk_segment(t, 0.0, 2.0, np.array([0.0, 0.0, land_z]), cruise)
        return pos, vel, acc, 0.0
    if t < 4.0:
        s = np.clip((t - 2.0) / 2.0, 0.0, 1.0)
        h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
        return cruise.copy(), np.zeros(3), np.zeros(3), 0.5*np.pi*h
    if t < 10.0:
        return _arc_segment(t, 4.0, 10.0, np.array([1.0, 0.0]), 1.0, takeoff_z)
    if t < 11.0:
        return arc_end, np.zeros(3), np.zeros(3), -0.5*np.pi
    if t < 13.5:
        pos, vel, acc = _min_jerk_segment(t, 11.0, 13.5, arc_end, landing)
        return pos, vel, acc, -0.5*np.pi
    return landing, np.zeros(3), np.zeros(3), -0.5*np.pi

# ===== 3. 多机模型生成（第 2 节）=====
def build_swarm_xml(n, spacing):
    agent_body = """
    <body name="quadrotor_{k}" pos="{x} 0 0.35">
      <freejoint name="root_{k}"/>
      <inertial pos="0 0 0" mass="1.388" diaginertia="0.010 0.010 0.018"/>
      <geom name="shell_{k}" type="ellipsoid" pos="0 0 0.018" size="0.082 0.070 0.028" rgba="0.93 0.94 0.93 1"/>
      <geom name="collide_{k}" type="sphere" pos="0 0 0" size="0.25" contype="1" conaffinity="1" mass="0.0"/>
      <body name="prop0_body_{k}" pos="0.12374 -0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor0_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop0_{k}" type="box" pos="0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
      <body name="prop1_body_{k}" pos="-0.12374 -0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor1_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop1_{k}" type="box" pos="-0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
      <body name="prop2_body_{k}" pos="-0.12374 0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor2_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop2_{k}" type="box" pos="0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
      <body name="prop3_body_{k}" pos="0.12374 0.12374 0.057">
        <inertial pos="0 0 0" mass="0.000001" diaginertia="0.000000001 0.000000001 0.000000002"/>
        <joint name="rotor3_spin_{k}" type="hinge" axis="0 0 1"/>
        <geom name="prop3_{k}" type="box" pos="-0.048 0 0.002" size="0.057 0.007 0.0015"/>
      </body>
    </body>
    """
    agents = []
    for k in range(n):
        x = (k - (n - 1) / 2.0) * spacing
        agents.append(agent_body.format(k=k, x=f"{x:.3f}"))
    return f"""<mujoco model="phantom_swarm">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <default><geom contype="0" conaffinity="0" density="0"/>
    <joint damping="0" frictionloss="0" armature="0"/></default>
  <worldbody>
    <light name="key" pos="0 -3 5" dir="0 1 -1" diffuse="0.8 0.8 0.78"/>
    <geom name="ground" type="plane" size="20 20 0.05" pos="0 0 -0.05" rgba="0.74 0.76 0.78 1"/>
    {''.join(agents)}
  </worldbody>
</mujoco>
"""

# ===== 4. 多机索引 helper（第 3.1 节）=====
def get_agent_slices(model, n):
    slices = []
    for k in range(n):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"quadrotor_{k}")
        if bid < 0:
            raise ValueError(f"找不到 body: quadrotor_{k}")
        jnt = model.body_jntadr[bid]
        q0 = model.jnt_qposadr[jnt]; v0 = model.jnt_dofadr[jnt]
        slices.append((slice(q0, q0+7), slice(v0, v0+6), bid))
    return slices

# ===== 1.5 旋翼几何（全局定义：参数打包和桨叶视觉都要用）=====
_HALF = L / np.sqrt(2.0)
ROTOR_XY = np.array([[_HALF,-_HALF],[-_HALF,-_HALF],[-_HALF,_HALF],[_HALF,_HALF]])
ROTOR_DIR = np.array([-1.0, 1.0, -1.0, 1.0])

# ===== 5. 参数打包（第 5.3 节）=====
def make_params():
    B = np.zeros((4, 4)); B[0, :] = KF
    for i in range(4):
        x, y = ROTOR_XY[i]
        B[1, i] = y*KF; B[2, i] = -x*KF; B[3, i] = ROTOR_DIR[i]*KM
    return dict(GRAVITY=GRAVITY, MASS=MASS, KF=KF, KM=KM, L=L, ROTOR_Z=ROTOR_Z,
                OMEGA_MAX=OMEGA_MAX, TAU_MOTOR=TAU_MOTOR, I_ROTOR=I_ROTOR, GYRO=GYRO,
                LINEAR_DRAG=LINEAR_DRAG, ANGULAR_DRAG=ANGULAR_DRAG,
                ROTOR_XY=ROTOR_XY, ROTOR_DIR=ROTOR_DIR, B_INV=np.linalg.inv(B),
                KP_POS=KP_POS, KD_POS=KD_POS, KR=KR, KW=KW)

# ===== 6. 单机控制器 / 施力 / 视觉（第 3.2 节）=====
def control_agent(pos, quat, vel, w, pos_d, vel_d, acc_d, yaw_d, omega, dt, P):
    R = quat2mat(quat)
    a_d = -P["KP_POS"]*(pos - pos_d) - P["KD_POS"]*(vel - vel_d) + acc_d
    F = P["MASS"] * (a_d + np.array([0.0, 0.0, P["GRAVITY"]]))
    zBd = F / np.linalg.norm(F)
    f = max(float(F @ R[:, 2]), 0.0)
    xc = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
    yBd = np.cross(zBd, xc); yBd /= np.linalg.norm(yBd)
    xBd = np.cross(yBd, zBd)
    Rd = np.column_stack([xBd, yBd, zBd])
    eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)
    tau = -P["KR"]*eR - P["KW"]*w
    w2 = P["B_INV"] @ np.array([f, tau[0], tau[1], tau[2]])
    w2 = np.clip(w2, 0.0, None)
    w_cmd = np.sqrt(w2)
    omega = omega + (dt / P["TAU_MOTOR"]) * (w_cmd - omega)
    return np.clip(omega, 0.0, P["OMEGA_MAX"])

def apply_forces_agent(data, body_id, pos, quat, vel, w, omega, P):
    R = quat2mat(quat)
    v_body = R.T @ vel
    F_body = -P["LINEAR_DRAG"] * v_body
    tau_body = -P["ANGULAR_DRAG"] * w
    for i in range(4):
        f = P["KF"] * omega[i]**2
        r = np.array([P["ROTOR_XY"][i][0], P["ROTOR_XY"][i][1], P["ROTOR_Z"]])
        F_body += np.array([0.0, 0.0, f])
        tau_body += np.cross(r, np.array([0.0, 0.0, f]))
        tau_body += np.array([0.0, 0.0, P["ROTOR_DIR"][i] * P["KM"] * omega[i]**2])
    if P["GYRO"]:
        h = P["I_ROTOR"] * float(np.sum(P["ROTOR_DIR"] * omega)) * np.array([0.0, 0.0, 1.0])
        tau_body += np.cross(h, w)
    data.xfrc_applied[body_id, 0:3] = R @ F_body
    data.xfrc_applied[body_id, 3:6] = R @ tau_body

def update_rotor_visuals_agent(model, data, k, omega, angles):
    for i in range(4):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"rotor{i}_spin_{k}")
        if jid < 0:
            continue
        data.qpos[model.jnt_qposadr[jid]] = angles[i]
        data.qvel[model.jnt_dofadr[jid]] = ROTOR_DIR[i] * omega[i]

# ===== 7. 集群算法（第 4 节）=====
def make_offsets(n, spacing):
    """三角形/直线编队：先按直线排，可自行改成 V 形等。"""
    off = np.zeros((n, 3))
    for k in range(n):
        off[k, 0] = (k - (n - 1) / 2.0) * spacing
    return off

def swarm_waypoint(t, n, offsets):
    pos_l, vel_l, acc_l, yaw_l = waypoint(t)
    return (np.tile(pos_l, (n, 1)) + offsets,
            np.tile(vel_l, (n, 1)),
            np.tile(acc_l, (n, 1)),
            np.full(n, yaw_l))

def avoidance_acc(pos, i, safe_dist, k_rep):
    a = np.zeros(3)
    for j in range(len(pos)):
        if j == i:
            continue
        d = pos[i] - pos[j]
        dist = np.linalg.norm(d)
        if 0 < dist < safe_dist:
            a += k_rep * (1.0/dist - 1.0/safe_dist) / (dist*dist) * d
    return a

# ===== 8. 主循环（第 3.3 节）=====
def run_swarm(n=3, T=16.0, dt=0.005, render=False, spacing=1.5):
    model = mujoco.MjModel.from_xml_string(build_swarm_xml(n, spacing))
    data = mujoco.MjData(model)
    model.opt.timestep = dt

    P = make_params()
    slices = get_agent_slices(model, n)
    offsets = make_offsets(n, spacing)

    omega = np.full((n, 4), np.sqrt(P["MASS"]*P["GRAVITY"]/(4*P["KF"])))
    prop_angles = np.zeros((n, 4))

    steps = int(T / dt)
    tlog = np.zeros(steps); plog = np.zeros((steps, n, 3)); pdlog = np.zeros((steps, n, 3))

    if render:
        import time
        from mujoco import viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)

    for k_step in range(steps):
        t = k_step * dt
        data.xfrc_applied[:] = 0.0

        pos  = np.stack([data.qpos[q][0:3]  for q, _, _ in slices])
        quat = np.stack([data.qpos[q][3:7]  for q, _, _ in slices])
        vel  = np.stack([data.qvel[v][0:3]  for _, v, _ in slices])
        w    = np.stack([data.qvel[v][3:6]  for _, v, _ in slices])

        pos_d, vel_d, acc_d, yaw_d = swarm_waypoint(t, n, offsets)

        for k in range(n):
            a_rep = avoidance_acc(pos, k, SAFE_DIST, K_REP)
            omega[k] = control_agent(pos[k], quat[k], vel[k], w[k],
                                     pos_d[k], vel_d[k], acc_d[k] + a_rep, yaw_d[k],
                                     omega[k], dt, P)
            apply_forces_agent(data, slices[k][2], pos[k], quat[k], vel[k], w[k], omega[k], P)
            prop_angles[k] = (prop_angles[k] + P["ROTOR_DIR"]*omega[k]*dt) % (2*np.pi)
            update_rotor_visuals_agent(model, data, k, omega[k], prop_angles[k])

        mujoco.mj_step(model, data)

        tlog[k_step] = t; plog[k_step] = pos; pdlog[k_step] = pos_d
        if render:
            viewer.sync(); time.sleep(dt)

    if render:
        viewer.close()

    err = np.linalg.norm(plog - pdlog, axis=2)
    for k in range(n):
        print(f"UAV {k}: RMS tracking error = {np.sqrt(np.mean(err[:, k]**2))*100:.2f} cm")
    np.savez("trajectory_swarm.npz", t=tlog, pos=plog, pos_des=pdlog, err=err)
    return err

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--N", type=int, default=3, help="无人机数量")
    ap.add_argument("--T", type=float, default=16.0)
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--spacing", type=float, default=1.5, help="初始间隔 m")
    ap.add_argument("--render", action="store_true")
    args = ap.parse_args()
    run_swarm(n=args.N, T=args.T, dt=args.dt, render=args.render, spacing=args.spacing)
```

**运行方式**：

```bash
python simulate_swarm.py --N 3 --render      # 3 架，打开可视化窗口
python simulate_swarm.py --N 5 --T 20        # 5 架，无窗口，跑 20 秒
```

---

## 7. 新手 FAQ 与调试清单

**Q1：`mj_name2id` 返回 -1 / 报 "找不到 body"**
名字不匹配。检查：XML 里是否叫 `quadrotor_0`，代码里是否写 `quadrotor_0`（大小写、下划线、编号要完全一致）。用 `model.names` 或打印 `mujoco.mj_name2id(...)` 的值来排查。

**Q2：多架无人机起飞瞬间叠在一起 / 画面里只有一架**
初始位置没错开。检查 `build_swarm_xml` 的 `pos` 是否每架不同（`spacing` 太小时视觉上像重合）。

**Q3：无人机互相穿过，避碰好像没生效**
默认 `contype=0` 是正常的（无物理碰撞）。确认你用的避碰是**逻辑 APF**（第 4.3 节），它加在 `acc_d` 上；检查 `SAFE_DIST` 是否设置得比机间距离大、`K_REP` 是否太小（先试 2.0）。

**Q4：转速一直饱和在 `OMEGA_MAX`，无人机乱飞**
控制器增益太大或模型参数不匹配。先把 `KP_POS` 调小（如 [2,2,4]），确认单机（`simulate.py`）能稳定跟踪后再跑集群。

**Q5：仿真很慢，怎么办？**
- 先减少 `n`（3–5 架起步）；
- 关闭物理碰撞（保持 `contype=0`）会快很多；
- 把 `--render` 去掉（渲染是主要开销）；
- 进阶：把每机控制器批量向量化（numpy 一次算 N 架），或改用 MuJoCo 的 MJX（GPU 加速，适合几十上百架）。

**Q6：怎么确认有没有真的撞上？**
逻辑避碰阶段可以记录"最小机间距离"：
```python
dmin = min(np.linalg.norm(pos[i]-pos[j]) for i in range(n) for j in range(i))
```
低于 `SAFE_DIST` 且持续缩小，说明避碰参数需要调整。开启物理碰撞后，也可以用 `data.ncon`（接触数量）判断。

**Q7：一致性算法不收敛 / 越飞越散**
大概率是通信拓扑不连通。用第 4.4 节的 `check_connected(A)` 检查；先全连通（全 1 矩阵）验证，再加稀疏拓扑。另外一致性增益 `KP_CONS/KD_CONS` 太大会振荡。

**Q8：想让每架飞不同的轨迹（不是编队）**
不改 `swarm_waypoint` 的"整体偏移"逻辑，而是给每架写一个独立的 `waypoint`（把原 `waypoint(t)` 的航点按编号错开，比如第 k 架绕第 k 个圆心飞）。

---

## 8. 术语表

| 术语 | 含义 |
|---|---|
| `freejoint` | MuJoCo 中"自由关节"，让刚体拥有 6 自由度（位置+姿态），无人机必须用它 |
| `qpos` / `qvel` | 广义位置 / 广义速度向量，多机时按顺序拼接 |
| `xfrc_applied` | MuJoCo 施加外力/力矩的接口（世界系，作用于 body 原点） |
| 控制分配 | 把"总推力+三轴力矩"拆成 4 个旋翼转速（B 矩阵求逆） |
| 几何控制 | 用旋转矩阵/四元数算姿态误差的控制器（Lee et al. 2010） |
| Leader-Follower | 编队控制：领航机跟踪参考轨迹，跟随机跟踪"领航机+偏移" |
| 一致性（Consensus） | 集群每架根据邻居状态调整，最终状态对齐的分布式控制 |
| 人工势场（APF） | 用"排斥势场"避免碰撞：距离越近排斥越强 |
| 邻接矩阵 / 拉普拉斯矩阵 | 描述通信拓扑的矩阵；拉普拉斯第二小特征值>0 说明拓扑连通 |
| `contype` / `conaffinity` | MuJoCo 碰撞分组参数：都设 0 表示不参与任何碰撞 |

---

## 9. 下一步建议（按需进阶）

1. **性能**：把 `control_agent` / `apply_forces_agent` 的 `for k` 循环向量化为批量 numpy 运算（N 架同时算），几十架也能实时跑。
2. **GPU 集群**：改用 MuJoCo 的 MJX（JAX 后端），单机 GPU 可并行仿真成百上千架，适合大规模集群训练/仿真。
3. **真实感**：给无人机加气动模型（旋翼尾流干扰）、风场（`xfrc_applied` 加扰动项）、通信时延/丢包（给邻居状态加随机延迟）。
4. **任务层**：把 `waypoint` 换成编队重构、目标分配（如多机编队穿过障碍区）、分布式覆盖等任务逻辑。
