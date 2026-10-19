"""
================================================================================
 DJI Phantom 4 Pro 四旋翼 —— MuJoCo 仿真 + 几何跟踪控制器（详细注释版）
================================================================================
【这个脚本在干什么】
  · 加载描述无人机几何/惯量/关节的模型文件 phantom4pro.xml（MuJoCo 模型）
  · 用 Lee、Leok & McClamroch (2010) 的"级联几何控制器"让无人机跟踪一条
    预设的航点轨迹（起飞 → 空中转向 → 圆弧巡航 → 悬停 → 降落）
  · 额外模拟了真实系统的细节：
      1) 电机一阶响应延迟（时间常数 0.03 s）与转速饱和（8500 rpm）
      2) 旋翼陀螺效应（可选，悬停附近影响很小）
      3) 用"数值方法"构造控制分配矩阵，避免手推符号时出错

【执行流程总览】（理解整个脚本的关键）
  初始化(加载模型/初始转速) 
    → 主循环每个仿真步：
        waypoint(t)     算出"期望的位置/速度/加速度/偏航角"
        control()       由期望量算出"4 个旋翼应有的转速"
        apply_forces()  把转速变成"推力/力矩"，施加到 MuJoCo 模型上
        mj_step()       MuJoCo 内部推进一个物理步（解算运动）
        update_rotor_visuals()  让画面里的螺旋桨转起来（纯视觉效果）
    → 结束后打印跟踪误差统计、保存数据与图片

【难懂概念速查】
  · qpos / qvel    ：MuJoCo 的广义位置/速度。自由刚体（无人机）的 qpos 前 7 个
                     数是 [x,y,z, 四元数(w,x,y,z)]，qvel 前 6 个数是 [线速度, 角速度]
  · xfrc_applied   ：MuJoCo 提供的"给刚体施加外力/力矩"的接口（作用在机体原点）
  · 控制分配      ：把"想要的总推力 f + 三轴力矩 τ"换算成"4 个旋翼各自的转速"
  · 几何控制      ：不把姿态参数化成欧拉角，而是直接用旋转矩阵/四元数做误差，
                     在大的姿态变化下也不会出现奇异性问题
================================================================================
"""

import argparse                 # 解析命令行参数（如 --render）
from pathlib import Path        # 跨平台路径处理

import numpy as np              # 数值计算
import mujoco                   # MuJoCo 物理引擎的 Python 接口


# ============================================================================
# 1. 物理参数 —— DJI Phantom 4 Pro 的实测/查表参数（推导见 README.md）
# ============================================================================
GRAVITY    = 9.81            # 重力加速度 (m/s^2)
MASS       = 1.388           # 起飞质量 (kg)，含电池和桨
KF         = 1.0e-5          # 推力系数：一个旋翼的推力 F = KF * ω²，单位 N/(rad/s)²
KM         = 1.5e-7          # 反扭矩系数：旋翼旋转产生的阻力矩 M = KM * ω²，单位 N·m/(rad/s)²
L          = 0.175           # 机臂长度 (m)：350 mm 轴距 → 中心到每个旋翼的水平距离
ROTOR_Z    = 0.03            # 旋翼平面在机体坐标系中高于重心 (COM) 的高度 (m)
OMEGA_MAX  = 8500.0 * 2.0*np.pi/60.0   # 最大转速：8500 rpm 换算成 rad/s（转/分 → 弧度/秒）
TAU_MOTOR  = 0.03            # 电机+电调的一阶时间常数 (s)：转速响应快慢
I_ROTOR    = 5.0e-5          # 旋翼+电机转子转动惯量 (kg·m²)，用于陀螺力矩计算
GYRO       = True            # 是否启用旋翼陀螺耦合效应
LINEAR_DRAG  = np.array([0.18, 0.18, 0.28])     # 平动空气阻力系数：F_drag = -k * v，N/(m/s)
ANGULAR_DRAG = np.array([0.018, 0.018, 0.025])  # 转动空气阻尼系数：τ_drag = -k * ω，N·m/(rad/s)

# 旋翼几何布局（X 型），机体坐标：x 向前、y 向左、z 向上
# rotor_dir[i] = +1 逆时针 / -1 顺时针（从上方俯视）；反扭矩符号随旋向改变
_HALF = L / np.sqrt(2.0)     # X 型布局中，旋翼在 x/y 轴上的投影距离 = L/√2
ROTOR_XY = np.array([          # 4 个旋翼在机体水平面 (x,y) 上的坐标
    [_HALF, -_HALF],   # 0 号：右前旋翼，顺时针
    [-_HALF, -_HALF],  # 1 号：左前旋翼，逆时针
    [-_HALF,  _HALF],  # 2 号：左后旋翼，顺时针
    [_HALF,  _HALF],   # 3 号：右后旋翼，逆时针
])
ROTOR_DIR = np.array([-1.0, 1.0, -1.0, 1.0])   # 对应上述每个旋翼的旋向

# ----------------------------------------------------------------------------
# 2. 控制分配矩阵 B：u = [f, τx, τy, τz] → [ω0², ω1², ω2², ω3²]
#    （用"数值方法"按旋翼几何直接构造，避免手推符号出错）
# ----------------------------------------------------------------------------
# 关系：u = B @ w2，其中 w2[i] = ω_i² 是第 i 个旋翼转速的平方。
# 求解时用 B 的逆：w2 = B⁻¹ @ u，一次矩阵求逆就完成"总需求 → 各旋翼分配"。
_B = np.zeros((4, 4))
_B[0, :] = KF                        # 总推力 f = KF·Σω² ：把 KF 填满第一行
for i in range(4):
    x, y = ROTOR_XY[i]
    _B[1, i] =  y * KF                # 滚转力矩 τx = Σ y_i·F_i （y 坐标×推力）
    _B[2, i] = -x * KF                # 俯仰力矩 τy = -Σ x_i·F_i （符号约定使"前推=低头"）
    _B[3, i] = ROTOR_DIR[i] * KM      # 偏航力矩 τz = Σ s_i·KM·ω_i² （旋向×反扭矩系数）
_B_INV = np.linalg.inv(_B)            # 求逆，主循环里直接用

# 控制器增益（位置环按轴分别设；姿态环用标量）
KP_POS = np.array([3.0, 3.0, 6.0])   # 位置比例增益 (1/s²)：z 轴更硬（要对抗重力）
KD_POS = np.array([2.5, 2.5, 4.0])   # 位置微分增益 (1/s)
KR      = 2.0                        # 姿态比例增益 (N·m/rad)
KW      = 0.4                        # 角速度阻尼增益 (N·m/(rad/s))


# ============================================================================
# 3. 基础数学工具
# ============================================================================
def quat2mat(q):
    """四元数 → 旋转矩阵（body 系→world 系）。

    MuJoCo 的四元数顺序是 (w, x, y, z)，w 是标量部分。
    返回的 3×3 矩阵 R 满足：世界系向量 = R @ 机体系向量。
    这个公式是四元数→旋转矩阵的标准转换（Rodrigues 公式展开）。
    """
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def vee(S):
    """vee 映射：把 3×3 反对称矩阵"摊平"成 3 维向量。

    任意 3×3 反对称矩阵 S 可以唯一写成 [v]× 的形式（叉乘矩阵），
    vee 就是取回那个向量 v 的逆操作。几何控制里用它把
    "旋转误差矩阵"变成"等效的小角度误差向量"（单位：弧度）。
    """
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


# ============================================================================
# 4. 轨迹生成（航点规划）
# ============================================================================
def _min_jerk_segment(t, t0, t1, p0, p1):
    """"最小加加速度"轨迹段：五次多项式插值，从 p0 平滑飞到 p1。

    为什么用五次多项式：它保证位置、速度、加速度【全部连续】，且两端
    速度/加速度都为零——无人机不会出现"急停/急启"的冲击，物理上可飞。
    s∈[0,1] 是归一化时间，h(s)=10s³-15s⁴+6s⁵ 是标准最小加加速度插值函数：
        h(0)=0, h(1)=1；h'(0)=h'(1)=0；h''(0)=h''(1)=0
    hd、hdd 是 h 对时间的 1/2 阶导数（除以 duration 做链式法则）。
    返回值：位置、速度、加速度（都是 3 维向量）。
    """
    duration = t1 - t0
    s = np.clip((t - t0) / duration, 0.0, 1.0)   # 归一化时间，钳制到 [0,1]
    h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5          # 位置插值
    hd = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / duration    # 速度
    hdd = (60.0*s - 180.0*s**2 + 120.0*s**3) / (duration**2)  # 加速度
    delta = p1 - p0
    return p0 + h * delta, hd * delta, hdd * delta


def _arc_segment(t, t0, t1, center, radius, z):
    """圆弧段：在水平面画一个"上半圆"，位置导数保持连续。

    用一个五次插值 h(s) 驱动角度 θ 从 π 均匀走到 0：
        θ = π·(1-h) → 起点 (center.x - r, center.y) 走到 (center.x + r, center.y)
        中间"拱起来"朝 +y 方向，正好是上半圆。
    速度/加速度由链式法则推出：pos' = d(pos)/dθ · θ̇，pos'' = d²(pos)/dθ²·θ̇² + d(pos)/dθ·θ̈
    偏航 yaw 跟随圆弧切线方向（-0.5π 的常数偏置是为了让机头朝切线）。
    """
    duration = t1 - t0
    s = np.clip((t - t0) / duration, 0.0, 1.0)
    h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
    hd = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / duration
    hdd = (60.0*s - 180.0*s**2 + 120.0*s**3) / (duration**2)

    # theta 从 pi 到 0：(0,0) -> (2*radius, 0)，朝 +y 拱起
    theta = np.pi * (1.0 - h)
    theta_d = -np.pi * hd          # θ 对时间的一阶导（链式法则）
    theta_dd = -np.pi * hdd        # θ 对时间的二阶导
    d1 = np.array([-radius * np.sin(theta), radius * np.cos(theta), 0.0])  # d(pos)/dθ
    d2 = np.array([-radius * np.cos(theta), -radius * np.sin(theta), 0.0])  # d²(pos)/dθ²
    pos = np.array([
        center[0] + radius * np.cos(theta),
        center[1] + radius * np.sin(theta),
        z,
    ])
    vel = d1 * theta_d                                   # 速度 = 一阶链式
    acc = d2 * theta_d**2 + d1 * theta_dd                # 加速度 = 二阶链式
    # 机头方向跟随圆弧切线；两端点处这个表达式依然良定义
    yaw = theta - 0.5 * np.pi
    return pos, vel, acc, yaw


def waypoint(t):
    """按时间 t 返回期望状态：(期望位置, 期望速度, 期望加速度, 期望偏航角)。

    整条任务轨迹分 5 段（时间轴看成一个"剧本"）：
        0.0–2.0 s   从地面 0.35 m 平滑起飞到巡航高度 1.25 m（最小加加速度段）
        2.0–4.0 s   原地悬停，同时机头旋转 90°（对准圆弧入口切线）
        4.0–10.0 s  沿上半圆弧巡航（圆心 (1,0)、半径 1、高度 1.25 m）
        10.0–11.0 s 在圆弧终点悬停，机头转回 -90°
        11.0–13.5 s 平滑降落到 (2, 0, 0.35 m) 降落点
        13.5 s 之后 停在降落点不动
    """
    takeoff_z = 1.25
    land_z = 0.35
    cruise = np.array([0.0, 0.0, takeoff_z])     # 巡航起点（起飞终点）
    arc_end = np.array([2.0, 0.0, takeoff_z])    # 圆弧终点
    landing = np.array([2.0, 0.0, land_z])       # 降落点

    if t < 2.0:
        # 第 1 段：从 (0,0,0.35) 平滑升到 (0,0,1.25)
        pos, vel, acc = _min_jerk_segment(
            t, 0.0, 2.0, np.array([0.0, 0.0, land_z]), cruise
        )
        return pos, vel, acc, 0.0                # 起飞阶段偏航保持 0
    if t < 4.0:
        # 第 2 段：位置不动，偏航从 0 平滑转到 90°（π/2）
        pos = cruise.copy()
        s = np.clip((t - 2.0) / 2.0, 0.0, 1.0)
        h = 10.0*s**3 - 15.0*s**4 + 6.0*s**5     # 还是五次插值，保证偏航角速度连续
        return pos, np.zeros(3), np.zeros(3), 0.5 * np.pi * h
    if t < 10.0:
        # 第 3 段：上半圆弧巡航，yaw 自动跟随切线
        return _arc_segment(t, 4.0, 10.0, np.array([1.0, 0.0]), 1.0, takeoff_z)
    if t < 11.0:
        # 第 4 段：圆弧终点悬停 1 秒，机头转到 -90°（准备降落方向）
        return arc_end, np.zeros(3), np.zeros(3), -0.5 * np.pi
    if t < 13.5:
        # 第 5 段：从圆弧终点平滑降落到降落点
        pos, vel, acc = _min_jerk_segment(t, 11.0, 13.5, arc_end, landing)
        return pos, vel, acc, -0.5 * np.pi
    # 时间超出剧本：一直停在降落点
    return landing, np.zeros(3), np.zeros(3), -0.5 * np.pi


# ============================================================================
# 5. 几何跟踪控制器（核心！）
# ============================================================================
def control(model, data, pos_d, vel_d, acc_d, yaw_d, omega, dt):
    """几何跟踪控制器：输入期望轨迹 + 当前状态，输出 4 个旋翼的【实际转速】。

    控制思路是"级联"的，像一个洋葱从外到里逐层剥：
        [外环] 位置环：位置误差 → 期望的加速度 → 期望的总推力方向和大小
        [中环] 姿态环：当前姿态 vs 期望姿态 → 期望的三轴力矩
        [内环] 分配 + 电机：推力/力矩 → 4 个旋翼转速 → 电机一阶响应 → 实际转速
    输入：
        pos_d/vel_d/acc_d  期望位置/速度/加速度（来自 waypoint）
        yaw_d              期望偏航角（机头朝向）
        omega              上一时刻的 4 个旋翼转速（用于电机动态递推）
        dt                 仿真步长
    输出：更新后的 4 个旋翼转速 omega（真正的力在 apply_forces 里施加）
    """
    pos = data.qpos[0:3]            # 当前位置 (x,y,z)（世界系）
    R   = quat2mat(data.qpos[3:7])  # 当前姿态：机体→世界的旋转矩阵
    vel = data.qvel[0:3]            # 当前线速度（世界系）
    w   = data.qvel[3:6]            # 当前角速度（机体坐标系！MuJoCo 对自由刚体如此约定）
    zB  = R[:, 2]                   # 机体 z 轴在世界系中的方向（旋翼推力方向）

    # ---------- ① 位置环：由位置误差算期望加速度，再算期望推力 ----------
    # 形式就是经典 PD + 前馈：a_d = -Kp·(p-p_d) - Kd·(v-v_d) + acc_d
    # 即：位置偏了就反向加速（Kp），速度偏了就阻尼（Kd），同时跟着目标加速度走（前馈）
    a_d = -KP_POS * (pos - pos_d) - KD_POS * (vel - vel_d) + acc_d
    # 无人机要产生这个加速度，旋翼必须同时克服重力：
    # 总推力矢量 F = m·(a_d + g)，g 是重力加速度矢量 [0,0,9.81]
    F   = MASS * (a_d + np.array([0.0, 0.0, GRAVITY]))
    # 推力方向就是期望的"机体 z 轴"方向（旋翼只能沿机体 z 轴向上推）
    zBd = F / np.linalg.norm(F)
    # 实际推力大小 = 当前机体 z 轴与期望方向的内积（投影），且不能为负（旋翼不能倒转推地）
    f   = max(float(F @ zB), 0.0)

    # ---------- ② 姿态环：由期望 z 轴 + 期望偏航，构造期望姿态矩阵 ----------
    # 期望航向在水平面内的方向（yaw 角 → 单位向量）
    xc  = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
    # 机体 y 轴 = zBd × xc（保证 y 轴垂直于 z 轴和航向，再归一化）
    yBd = np.cross(zBd, xc)
    yBd = yBd / np.linalg.norm(yBd)
    # 机体 x 轴 = y × z，三轴正交构成期望旋转矩阵 Rd（列向量拼接）
    xBd = np.cross(yBd, zBd)
    Rd  = np.column_stack([xBd, yBd, zBd])

    # 姿态误差（几何控制的核心公式）：
    #   eR = 0.5·vee(RdᵀR - RᵀRd)，这是两个旋转矩阵之间的"李代数误差"，
    #   物理意义是当前姿态到期望姿态的等效小角度旋转向量（弧度）
    eR  = 0.5 * vee(Rd.T @ R - R.T @ Rd)
    eW  = w                                 # 期望角速度取 0（悬停/匀速段都适用）
    # 姿态环也是 PD：力矩 = -KR·姿态误差 - KW·角速度（机体坐标系下）
    tau = -KR * eR - KW * eW

    # ---------- ③ 控制分配：总需求 → 4 个旋翼的转速 ----------
    # 上一步算出想要的 [推力 f, 力矩 τx, τy, τz]，这一步用 B 矩阵的逆
    # 把它拆成 4 个旋翼转速的平方：w2 = B⁻¹ @ [f, τx, τy, τz]
    w2    = _B_INV @ np.array([f, tau[0], tau[1], tau[2]])
    # 转速平方不能为负（物理上 ω²≥0），负数钳到 0（意味着该旋翼停转）
    w2    = np.clip(w2, 0.0, None)
    w_cmd = np.sqrt(w2)                     # 开方得到"期望转速命令"

    # ---------- ④ 电机一阶动力学 + 转速饱和 ----------
    # 电机不是瞬间达到指令转速，而是一阶惯性环节：
    #   dω/dt = (ω_cmd - ω)/τ  → 离散化：ω ← ω + (dt/τ)·(ω_cmd - ω)
    # τ=0.03s 表示约 3 个时间常数（~0.1s）内追上指令
    omega = omega + (dt / TAU_MOTOR) * (w_cmd - omega)
    # 物理上限：转速不能超过电机最大转速
    omega = np.clip(omega, 0.0, OMEGA_MAX)
    return omega


# ============================================================================
# 6. 把转速变成力/力矩，施加到 MuJoCo 模型
# ============================================================================
def apply_forces(model, data, omega):
    """把 4 个旋翼的转速换算成推力+力矩，写入 data.xfrc_applied。

    全部在【机体坐标系】里算（这样力会随着机身一起倾斜），
    最后统一旋转一次转到【世界坐标系】——因为 xfrc_applied 要求世界系。
    """
    # 按名字找到模型里的 "quadrotor" 刚体（body）的 id
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    R  = quat2mat(data.qpos[3:7])           # 机体→世界旋转矩阵
    v_body = R.T @ data.qvel[0:3]           # 把世界系线速度转到机体系（R.T 即转置=逆）
    w_body = data.qvel[3:6]                 # 角速度本身就是机体系

    # 空气阻力：机体系中 力 = -阻力系数 × 速度（平动） / 力矩 = -阻尼系数 × 角速度（转动）
    F_body = -LINEAR_DRAG * v_body
    tau_body = -ANGULAR_DRAG * w_body
    for i in range(4):
        f = KF * omega[i] ** 2              # 第 i 个旋翼的推力 F = KF·ω²
        r = np.array([ROTOR_XY[i][0], ROTOR_XY[i][1], ROTOR_Z])  # 该旋翼相对重心的位置
        F_body += np.array([0.0, 0.0, f])                 # 推力沿机体 +z 方向
        tau_body += np.cross(r, np.array([0.0, 0.0, f]))  # 推力不在重心 → 产生滚转/俯仰力矩（力臂）
        # 旋翼自身的反扭矩（螺旋桨转动空气时，空气反推机身绕 z 轴转）
        tau_body += np.array([0.0, 0.0, ROTOR_DIR[i] * KM * omega[i] ** 2])

    # 陀螺效应（可选）：旋翼转动有角动量 h = I·Σ(s_i·ω_i)·ẑ；
    # 当机体自身转动时，改变 h 的方向需要力矩，即 h × ω_body（陀螺进动）
    if GYRO:
        h = I_ROTOR * float(np.sum(ROTOR_DIR * omega)) * np.array([0.0, 0.0, 1.0])
        tau_body += np.cross(h, w_body)

    # 写入 MuJoCo 施加外力接口：世界系力 = R @ 机体系力，力矩同理
    data.xfrc_applied[body_id, 0:3] = R @ F_body
    data.xfrc_applied[body_id, 3:6] = R @ tau_body


# ============================================================================
# 7. 视觉效果：让模型里的螺旋桨转起来（不影响物理）
# ============================================================================
def update_rotor_visuals(model, data, omega, angles):
    """旋转模型里 4 个"视觉用"的铰链（rotor0_spin...），纯装饰。

    物理引擎里力是通过 xfrc_applied 直接施加的，不需要桨真的转；
    但画面上如果桨不转会很假，所以这里手动把 4 个铰链的角度/角速度写进
    状态里，让渲染出的桨叶转起来。angles 是累计的转角。
    """
    for i, name in enumerate(("rotor0_spin", "rotor1_spin", "rotor2_spin", "rotor3_spin")):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)  # 按名查铰链 id
        if joint_id < 0:                    # 模型里没有这个铰链就跳过（-1 表示找不到）
            continue
        qadr = model.jnt_qposadr[joint_id]  # 该铰链在 qpos 里的起始索引
        vadr = model.jnt_dofadr[joint_id]   # 该铰链在 qvel 里的起始索引
        data.qpos[qadr] = angles[i]         # 写入累计转角
        data.qvel[vadr] = ROTOR_DIR[i] * omega[i]   # 写入角速度（带旋向符号）


# ============================================================================
# 8. 主程序：初始化 + 主循环 + 统计输出
# ============================================================================
def run(T=16.0, dt=0.005, render=False):
    # 模型文件与脚本同目录：phantom4pro.xml
    model_path = Path(__file__).with_name("phantom4pro.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))  # 解析 XML 生成模型
    data  = mujoco.MjData(model)                            # 创建仿真状态容器
    model.opt.timestep = dt                                 # 设置物理仿真步长

    # 初始转速：悬停转速 ≈ √(mg/(4·KF))（重力由 4 个旋翼平均分担），避免起飞瞬间掉高度
    omega = np.full(4, np.sqrt(MASS * GRAVITY / (4 * KF)))
    prop_angles = np.zeros(4)                               # 螺旋桨累计转角（视觉用）

    steps = int(T / dt)          # 总步数
    tlog  = np.zeros(steps)      # 记录每个时刻的时间
    plog  = np.zeros((steps, 3)) # 记录实际位置
    pdlog = np.zeros((steps, 3)) # 记录期望位置（用于对比误差）

    # 交互式渲染模式：启动 MuJoCo 自带查看器
    if render:
        import time
        from mujoco import viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)

    for k in range(steps):
        t = k * dt
        # ① 查剧本：这个时刻期望飞到哪里、多快、什么朝向
        pos_d, vel_d, acc_d, yaw_d = waypoint(t)
        # ② 控制器：算出 4 个旋翼转速（含电机动态）
        omega = control(model, data, pos_d, vel_d, acc_d, yaw_d, omega, dt)
        # ③ 更新视觉桨叶累计角度
        prop_angles = (prop_angles + ROTOR_DIR * omega * dt) % (2.0 * np.pi)
        # ④ 清空上一帧施加的力，再写入这一帧的力（MuJoCo 不自动清零）
        data.xfrc_applied[:] = 0.0
        apply_forces(model, data, omega)
        # ⑤ 让 MuJoCo 推进一个物理步（解算刚体动力学/接触）
        mujoco.mj_step(model, data)
        # ⑥ 渲染模式下同步画面
        update_rotor_visuals(model, data, omega, prop_angles)

        # 记录数据（拷贝一份，避免被后续步进覆盖）
        tlog[k]   = t
        plog[k]   = data.qpos[0:3].copy()
        pdlog[k]  = pos_d

        if render:
            viewer.sync()
            time.sleep(dt)      # 让画面以真实速度播放

    if render:
        viewer.close()

    # 统计跟踪误差：每个时刻 期望-实际 的 2 范数（三维距离）
    err = np.linalg.norm(plog - pdlog, axis=1)
    rms = float(np.sqrt(np.mean(err ** 2)))   # 均方根误差
    mx  = float(np.max(err))                  # 最大误差

    # 打印结果
    print(f"simulated {T:.1f}s  (dt={dt})")
    print(f"final position        = [{plog[-1,0]:+.3f}, {plog[-1,1]:+.3f}, {plog[-1,2]:+.3f}] m")
    print(f"final |omega|         = [{omega[0]:.0f}, {omega[1]:.0f}, {omega[2]:.0f}, {omega[3]:.0f}] rad/s")
    print(f"RMS tracking error    = {rms*100:.2f} cm")
    print(f"max  tracking error   = {mx*100:.2f} cm")

    # 保存数据（.npz 可用 numpy 加载）和绘图
    np.savez("trajectory.npz", t=tlog, pos=plog, pos_des=pdlog, err=err)
    _plot(tlog, plog, pdlog, err)
    return rms


def _plot(t, pos, pos_d, err):
    """画图：期望 vs 实际的三轴轨迹对比（纯辅助功能）。"""
    try:
        import matplotlib
        matplotlib.use("Agg")           # 无窗口后端（服务器/命令行可用）
        import matplotlib.pyplot as plt
    except Exception:
        print("(matplotlib not installed — skipping plot; see trajectory.npz)")
        return

    fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)   # 3 行子图：x/y/z
    labels = ["x", "y", "z"]
    for i in range(3):
        ax[i].plot(t, pos_d[:, i], "k--", lw=1.0, label="desired")  # 期望（黑虚线）
        ax[i].plot(t, pos[:, i], "C0", lw=1.2, label="actual")      # 实际（蓝色）
        ax[i].set_ylabel(f"{labels[i]} [m]")
        ax[i].grid(True, alpha=0.3)
        ax[i].legend(loc="upper right", fontsize=8)
    ax[0].set_title("DJI Phantom 4 Pro — waypoint tracking (MuJoCo)")
    ax[2].set_xlabel("time [s]")
    plt.tight_layout()
    plt.savefig("trajectory.png", dpi=120)     # 保存图片（不是显示窗口）
    print("saved trajectory.png")


# ============================================================================
# 9. 命令行入口
# ============================================================================
if __name__ == "__main__":
    # 支持三个可选参数：
    #   --T 16       仿真时长（秒），默认 16
    #   --dt 0.005   仿真步长（秒），默认 0.005（=200Hz 控制频率）
    #   --render     是否打开交互式可视化窗口
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=16.0)
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--render", action="store_true")
    args = ap.parse_args()
    run(args.T, args.dt, args.render)
