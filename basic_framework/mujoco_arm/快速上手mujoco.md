# MuJoCo 机械臂仿真修改指南

本文档详细说明如何根据需求修改 `mujoco_arm/` 文件夹中的仿真文件。

---

## 目录

- [文件概览](#文件概览)
- [一、修改仿真模型 (robticarm.xml)](#一修改仿真模型-robticarmxml)
  - [1.1 修改机械臂尺寸与结构](#11-修改机械臂尺寸与结构)
  - [1.2 修改关节运动范围](#12-修改关节运动范围)
  - [1.3 修改外观颜色与材质](#13-修改外观颜色与材质)
  - [1.4 修改物理参数](#14-修改物理参数)
  - [1.5 添加或删除几何体（geom）](#15-添加或删除几何体geom)
  - [1.6 调整仿真正选项](#16-调整仿真正选项)
  - [1.7 修改运动学链结构（添加/删除关节）](#17-修改运动学链结构添加删除关节)
  - [1.8 修改传感器配置](#18-修改传感器配置)
  - [1.9 修改预设关键帧（keyframe）](#19-修改预设关键帧keyframe)
  - [1.10 修改执行器（actuator）](#110-修改执行器actuator)
- [二、修改控制程序 (run_ntu_arm.py)](#二修改控制程序-run_ntu_armpy)
  - [2.1 修改默认伺服参数](#21-修改默认伺服参数)
  - [2.2 编写自定义运动轨迹](#22-编写自定义运动轨迹)
  - [2.3 编写末端执行器（夹爪）控制](#23-编写末端执行器夹爪控制)
  - [2.4 添加键盘交互](#24-添加键盘交互)
  - [2.5 实现闭环控制（如逆运动学）](#25-实现闭环控制如逆运动学)
  - [2.6 记录与导出仿真数据](#26-记录与导出仿真数据)
  - [2.7 添加多个物体（抓取任务）](#27-添加多个物体抓取任务)
  - [2.8 修改革命周期设置](#28-修改革命周期设置)
- [三、修改 README.md](#三修改-readmemd)
- [四、常见需求速查表](#四常见需求速查表)
- [五、运动学链结构参考](#五运动学链结构参考)

---

## 文件概览

| 文件 | 类型 | 作用 |
|---|---|---|
| `robticarm.xml` | MJCF 模型文件 | 定义机械臂几何体、关节、材质、传感器、执行器、物理参数 |
| `run_ntu_arm.py` | Python 脚本 | 加载模型、配置伺服、运行 MuJoCo 渲染窗口 |
| `README.md` | 文档 | 使用说明与模型约定 |

---

## 一、修改仿真模型 (robticarm.xml)

`robticarm.xml` 是整个仿真的核心。它是一个 XML 文件，遵循 MuJoCo 的 MJCF（MuJoCo CFG）格式规范。

### 1.1 修改机械臂尺寸与结构

所有几何体通过 `<geom>` 标签定义，以下是常用的几何体类型及其尺寸参数：

#### 盒子 (box)
```xml
<geom name="base_plate" type="box" pos="0 0 0.015" size="0.18 0.13 0.015" material="base_mat"/>
```
- `size` 的三个值分别是 **X方向半长、Y方向半长、Z方向半长**（单位：米）
- 要改变长宽高，直接修改这三个数字

#### 圆柱体 (cylinder)
```xml
<geom name="yaw_column" type="cylinder" pos="0 0 0.090" size="0.055 0.075" material="base_mat"/>
```
- `size` 的两个值分别是 **半径、半高**
- 注意：圆柱默认轴线沿 Z 方向

#### 胶囊体 (capsule)
```xml
<geom name="upper_arm_main" type="capsule" fromto="0 0 0.070 0 0 0.250" size="0.024" material="link_mat"/>
```
- `fromto` 定义了胶囊体的 **起点 (X Y Z) 和终点 (X Y Z)**
- `size` 是 **半径**
- 要改变连杆长度 → 修改 `fromto` 的终点坐标
- **重要**：修改连杆长度后，必须同步修改其子节点 body 的 `pos` 属性

#### 修改示例：把上臂从 304mm 缩短到 220mm

上臂的**运动学长度**由 `j3_elbow_frame` 的 `pos` 决定（304mm），而胶囊体 `upper_arm_main` 的 `fromto`（0.070 → 0.250，跨度 180mm）只是**视觉几何长度**，它比运动学长度短是因为两端有 clamp 结构（root_clamp 和 tip_clamp）占据了空间。修改时必须同时调整运动学参数和视觉几何体。

**原始（运动学长度 304mm）：**
```xml
<!-- 关键参数：j3 肘关节相对 j2 的 Z 偏移 = 0.304m -->
<body name="j3_elbow_frame" pos="0 0 0.304">
<!-- 视觉胶囊体跨度 = 0.250 - 0.070 = 0.180m -->
<geom name="upper_arm_main" type="capsule" fromto="0 0 0.070 0 0 0.250" size="0.024" material="link_mat"/>
<geom name="upper_arm_tip_clamp" type="box" pos="0 0 0.260" size="0.040 0.052 0.018" material="side_mat"/>
<geom name="upper_arm_side_a" type="capsule" fromto="0 0.030 0.070 0 0.030 0.250" size="0.008" material="side_mat"/>
<geom name="upper_arm_side_b" type="capsule" fromto="0 -0.030 0.070 0 -0.030 0.250" size="0.008" material="side_mat"/>
<geom name="upper_arm_screw_2" type="cylinder" pos="0.025 0.041 0.250" euler="1.570796 0 0" size="0.004 0.003" material="fastener_mat"/>
```

**修改后（运动学长度 220mm，视觉几何体按比例同步缩放）：**
```xml
<!-- 运动学长度从 304mm 缩短到 220mm -->
<body name="j3_elbow_frame" pos="0 0 0.220">
<!-- 视觉胶囊体也按比例缩短，保持 clamp 空间的相对比例 -->
<geom name="upper_arm_main" type="capsule" fromto="0 0 0.070 0 0 0.175" size="0.024" material="link_mat"/>
<geom name="upper_arm_tip_clamp" type="box" pos="0 0 0.185" size="0.040 0.052 0.018" material="side_mat"/>
<geom name="upper_arm_side_a" type="capsule" fromto="0 0.030 0.070 0 0.030 0.175" size="0.008" material="side_mat"/>
<geom name="upper_arm_side_b" type="capsule" fromto="0 -0.030 0.070 0 -0.030 0.175" size="0.008" material="side_mat"/>
<geom name="upper_arm_screw_2" type="cylinder" pos="0.025 0.041 0.175" euler="1.570796 0 0" size="0.004 0.003" material="fastener_mat"/>
```

同时注意：同一个连杆通常有多条 geom（主体、侧边、盖板、螺丝等），**全部**要一起修改保持一致。此外，修改运动学长度后所有下游 body 的 `pos` 也需要重新计算。

#### 坐标系注意事项

- MuJoCo 默认世界坐标系：**+Z 向上**（重力为 `0 0 -9.8`），地面位于 `Z=0`
- 基准长度单位为 **米**，角度单位为 **弧度**（由 `<compiler angle="radian">` 指定）
- 本模型的布局：底座绕 Z 轴旋转（J1 偏航），上臂沿 Z 方向伸展，前臂沿 X 方向伸展

---

### 1.2 修改关节运动范围

所有关节通过 `<joint>` 标签定义。修改 `range` 属性即可改变运动范围。

#### 关节格式
```xml
<joint name="j2_pitch" axis="0 1 0" range="-2.36 2.36"/>
```

| 属性 | 含义 |
|---|---|
| `name` | 关节名称 |
| `axis` | 旋转轴 (X Y Z) |
| `range` | 角度范围 [最小值, 最大值]，单位：弧度 |
| `damping` | 阻尼系数（可在 default 中全局设置） |

#### 弧度与角度的换算

| 角度 | 弧度 |
|---|---|
| 90° | ≈1.57 |
| 180° | ≈3.14 |
| 270° | ≈4.71 |
| 360° | ≈6.28 |

#### 修改示例

```xml
<!-- J2 肩关节从 +-135° 改为 +-90° -->
<joint name="j2_pitch" axis="0 1 0" range="-1.57 1.57"/>

<!-- J3 肘关节从 +-150° 改为 +-90° -->
<joint name="j3_pitch" axis="0 1 0" range="-1.57 1.57"/>

<!-- J1 底座改为一圈 -->
<joint name="j1_yaw" axis="0 0 1" range="-6.28 6.28"/>
```

**注意**：修改关节范围后，对应的 actuator 的 `ctrlrange` 也需要同步修改：

```xml
<!-- 同步修改 actuator -->
<general name="joint1" joint="j1_yaw" ctrllimited="true" ctrlrange="-6.28 6.28"/>
```

---

### 1.3 修改外观颜色与材质

所有材质定义在 `<asset>` 块中，颜色使用 **RGBA** 格式（红、绿、蓝、透明度，值域 0~1）。

#### 材质定义格式
```xml
<material name="link_mat" rgba="0.38 0.43 0.47 1"/>
```

#### 材质的引用
```xml
<geom name="upper_arm_main" type="capsule" ... material="link_mat"/>
```

#### 常用颜色参考

| 颜色 | RGBA 值 |
|---|---|
| 深灰 | `0.2 0.2 0.22 1` |
| 金属灰 | `0.5 0.52 0.55 1` |
| 白色 | `0.85 0.85 0.85 1` |
| 黑色 | `0.02 0.02 0.02 1` |
| 红色 | `0.85 0.15 0.15 1` |
| 蓝色 | `0.15 0.30 0.70 1` |
| 橙色 | `0.90 0.50 0.15 1` |
| 黄色 | `0.90 0.85 0.10 1` |
| 绿色 | `0.15 0.75 0.25 1` |

#### 修改示例：将机械臂改成橙色主题

```xml
<!-- 连杆主体改成橙色 -->
<material name="link_mat" rgba="0.90 0.50 0.15 1"/>

<!-- 盖板改成深橙色 -->
<material name="cover_mat" rgba="0.70 0.35 0.10 1"/>

<!-- 侧面改成浅橙色 -->
<material name="side_mat" rgba="0.85 0.55 0.25 1"/>
```

#### 添加纹理

当前项目对地板使用了棋盘格纹理：
```xml
<texture name="grid" type="2d" builtin="checker" rgb1="0.18 0.19 0.20" rgb2="0.23 0.24 0.25"
         width="512" height="512"/>
<material name="floor_mat" texture="grid" texrepeat="3 3" reflectance="0.12"/>
```

可以修改 `rgb1` 和 `rgb2` 来改变棋盘格颜色。

---

### 1.4 修改物理参数

#### ① 全局关节默认参数

在 `<default>` 块中：
```xml
<default>
    <joint type="hinge" damping="1.80" armature="0.045" frictionloss="0.020"/>
    <geom contype="0" conaffinity="0" density="750" solref="0.01 1"/>
    <general gaintype="fixed" gainprm="18 0 0" biastype="affine" biasprm="0 -18 -5"
             forcelimited="true" forcerange="-24 24"/>
</default>
```

| 参数 | 含义 | 调大效果 |
|---|---|---|
| `damping` | 关节粘性阻尼 | 转动更"粘滞" |
| `armature` | 电枢惯量 | 加速更慢 |
| `frictionloss` | 库仑摩擦力 | 静摩擦更大 |
| `density` | 几何体密度 (kg/m³) | 部件更重 |
| `forcerange` | 伺服力限 (N·m) | 伺服更有力 |

#### ② 修改重力

```xml
<option timestep="0.001" gravity="0 0 -9.8" integrator="implicitfast" solver="Newton"
        iterations="80" tolerance="1e-10"/>
```

| 场景 | gravity 值 |
|---|---|
| 标准地球重力 | `0 0 -9.8` |
| 月球重力 | `0 0 -1.62` |
| 零重力（太空） | `0 0 0` |
| 侧向重力（测试用） | `0 -9.8 0` |

#### ③ 修改仿真的步长

`timestep` 是仿真步长（秒），影响仿真精度和速度：

```xml
<!-- 默认 0.001s（1kHz），降低到 0.002s 会使仿真变快但精度下降 -->
<option timestep="0.002" ...>
```

**建议**：通常保持 `0.001`，只有在性能不够时适当增大到 `0.002` 或 `0.005`。

#### ④ 修改碰撞属性

```xml
<geom contype="0" conaffinity="0" .../>
```

- `contype`：该几何体的碰撞类型（bitmask）
- `conaffinity`：该几何体能碰撞到的类型（bitmask）
- 设为 `0` 和 `0` 表示不参与碰撞检测

如果要启用碰撞，改为：
```xml
<geom contype="1" conaffinity="1" .../>
```

---

### 1.5 添加或删除几何体（geom）

#### 添加新的几何体

在对应的 `<body>` 块内添加 `<geom>` 标签：

```xml
<body name="j6_tool_roll_frame" pos="0.085 0 0">
    <joint name="j6_tool_roll" axis="1 0 0" range="-3.1416 3.1416"/>
    
    <!-- ↓↓↓ 添加一个新几何体（例如摄像头） ↓↓↓ -->
    <geom name="camera_module" type="box" pos="0.030 -0.025 0.015" size="0.015 0.010 0.010" rgba="0.15 0.15 0.18 1"/>
    <!-- ↑↑↑ 添加完毕 ↑↑↑ -->
    
    ...原有的 geom...
</body>
```

#### 添加负载（如抓取目标）

在 `<worldbody>` 的顶层添加独立 body：

```xml
<worldbody>
    ...原有内容...
    
    <!-- 添加一个自由物体 -->
    <body name="target_cube" pos="0.3 0.1 0.05">
        <freejoint/>  <!-- 自由关节，物体可自由运动 -->
        <geom name="target_geom" type="box" size="0.025 0.025 0.025" rgba="0.90 0.20 0.20 1"/>
    </body>
</worldbody>
```

#### 删除几何体

直接删除对应的 `<geom ... />` 行即可。**注意不要删除含有子 body 的 body 块**（会导致子结构一并被删除）。

---

### 1.6 调整仿真正选项

#### 修改相机默认视角

有两种方式：

**方式一：修改 worldbody 中的 camera 定义：**
```xml
<camera name="overview" pos="0.75 -0.95 0.62" xyaxes="0.78 0.63 0 -0.30 0.37 0.88"/>
```
- `pos`：相机位置 (X Y Z)
- `xyaxes`：相机的 X 和 Y 轴方向（决定看向哪里）

**方式二：在 Python 脚本中设置（推荐，更方便调试）：**
```python
viewer.cam.distance = 0.8
viewer.cam.azimuth = 135
viewer.cam.elevation = -25
viewer.cam.lookat[:] = [0.12, 0.0, 0.28]
```

#### 修改光照

```xml
<visual>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.55 0.55 0.55" specular="0.25 0.25 0.25"/>
    <global azimuth="135" elevation="-22"/>
    <quality shadowsize="2048"/>
</visual>
```

| 参数 | 含义 |
|---|---|
| `ambient` | 环境光强度 (R G B) |
| `diffuse` | 漫反射光强度 |
| `specular` | 镜面反射光强度 |
| `shadowsize` | 阴影贴图分辨率（越大阴影越精细，性能越差） |

#### 修改地板大小

```xml
<geom name="floor" type="plane" size="1.3 1.3 0.02" material="floor_mat"/>
```
- 前两个数：平面的 X 和 Y 半长
- `plane` 类型默认在 Z=0 高度

---

### 1.7 修改运动学链结构（添加/删除关节）

#### 当前的关节链

```
base_fixed (固定底座)
  └─ j1_yaw_frame → joint: j1_yaw (底部偏航)
      └─ j2_pitch_frame → joint: j2_pitch (肩部俯仰)
          └─ j3_elbow_frame → joint: j3_pitch (肘部俯仰)
              └─ wrist_center_frame (腕心参考点)
                  └─ j4_roll_frame → joint: j4_roll (前臂滚转)
                      └─ j5_pitch_frame → joint: j5_wrist_pitch (腕部俯仰)
                          └─ j6_tool_roll_frame → joint: j6_tool_roll (工具滚转)
                              └─ gripper_upper_finger → joint: gripper_upper_slide (上指滑动)
                              └─ gripper_lower_finger → joint: gripper_lower_slide (下指滑动)
```

#### 添加第 7 个关节

在 `j6_tool_roll_frame` 内部（gripper 相关的 body 之前）插入新 body：

```xml
<body name="j6_tool_roll_frame" pos="0.085 0 0">
    <joint name="j6_tool_roll" axis="1 0 0" range="-3.1416 3.1416"/>
    ...原有的 geom...
    
    <!-- 添加 J7 旋转台 -->
    <body name="j7_rotate_frame" pos="0.120 0 0">
        <joint name="j7_rotate" axis="0 0 1" range="-1.57 1.57"/>
        <geom name="j7_platform" type="cylinder" pos="0 0 0" size="0.025 0.008" rgba="0.80 0.80 0.20 1"/>
        
        <!-- 可以把 gripper 移到这里，或者添加其他工具 -->
    </body>
    
    ...原有的 gripper...
</body>
```

**重要**：添加关节后必须同步：
1. 在 `<actuator>` 块添加对应的执行器
2. 在 `<sensor>` 块添加对应的传感器
3. 在 `<keyframe>` 块更新预设姿态
4. 在 Python 脚本中更新 `HOME_POSE` 和相关逻辑

---

### 1.8 修改传感器配置

#### 当前传感器

```xml
<sensor>
    <!-- 关节位置传感器 -->
    <jointpos name="j1_q" joint="j1_yaw"/>
    <jointpos name="j2_q" joint="j2_pitch"/>
    <jointpos name="j3_q" joint="j3_pitch"/>
    <jointpos name="j4_q" joint="j4_roll"/>
    <jointpos name="j5_q" joint="j5_wrist_pitch"/>
    <jointpos name="j6_q" joint="j6_tool_roll"/>
    <!-- 末端位置/姿态传感器 -->
    <framepos name="tcp_pos" objtype="site" objname="tcp"/>
    <framequat name="tcp_quat" objtype="site" objname="tcp"/>
</sensor>
```

#### 添加新传感器

| 传感器类型 | XML 语法 | 用途 |
|---|---|---|
| 关节位置 | `<jointpos name="name" joint="joint_name"/>` | 读取关节角度 |
| 关节速度 | `<jointvel name="name" joint="joint_name"/>` | 读取关节角速度 |
| 关节力矩 | `<actuatorfrc name="name" joint="joint_name"/>` | 读取关节轴上的总力矩（含执行器、阻尼、约束等所有力分量） |
| 触觉/力 | `<touch name="name" site="site_name"/>` | 检测碰撞接触 |
| 加速度计 | `<accelerometer name="name" site="site_name"/>` | 测量加速度 |
| 陀螺仪 | `<gyro name="name" site="site_name"/>` | 测量角速度 |
| 帧位置 | `<framepos name="name" objtype="site/body/geom" objname="name"/>` | 读取某物体位置 |

示例 — 添加关节速度传感器：
```xml
<jointvel name="j1_v" joint="j1_yaw"/>
<jointvel name="j2_v" joint="j2_pitch"/>
```

在 Python 中读取：
```python
j1_velocity = data.sensor("j1_v").data[0]
```

---

### 1.9 修改预设关键帧（keyframe）

关键帧允许用户在 MuJoCo 查看器中一键切换到预设姿态。

```xml
<keyframe>
    <key name="home" qpos="0 0 0 0 0 0 0 0" ctrl="0 0 0 0 0 0"/>
    <key name="j1_left" qpos="1.20 0 0 0 0 0 0 0" ctrl="1.20 0 0 0 0 0"/>
    <key name="reach" qpos="0.35 -0.45 0.70 0.40 0.35 -0.80 0 0" ctrl="0.35 -0.45 0.70 0.40 0.35 -0.80"/>
</keyframe>
```

`qpos` 格式：`[j1 j2 j3 j4 j5 j6 gripper_upper gripper_lower]`（8个值）
`ctrl` 格式：`[j1 j2 j3 j4 j5 j6]`（6个值，对应执行器）

#### 添加自定义关键帧

```xml
<key name="my_home" qpos="0 0.5 1.0 0 0.8 0 0 0" ctrl="0 0.5 1.0 0 0.8 0"/>
```

---

### 1.10 修改执行器（actuator）

#### 当前执行器

```xml
<actuator>
    <general name="joint1" joint="j1_yaw" ctrllimited="true" ctrlrange="-3.14 3.14"/>
    <general name="joint2" joint="j2_pitch" ctrllimited="true" ctrlrange="-2.36 2.36"/>
    <general name="joint3" joint="j3_pitch" ctrllimited="true" ctrlrange="-2.62 2.62"/>
    <general name="joint4" joint="j4_roll" ctrllimited="true" ctrlrange="-3.14 3.14"/>
    <general name="joint5" joint="j5_wrist_pitch" ctrllimited="true" ctrlrange="-1.5708 1.5708"/>
    <general name="joint6" joint="j6_tool_roll" ctrllimited="true" ctrlrange="-3.1416 3.1416"/>
</actuator>
```

每个执行器通过 gain/bias 参数实现位置伺服。以默认参数为例（`kp=18, kv=5`），`configure_position_servos` 设置：
```
gainprm[0] = kp
biasprm[1] = -kp
biasprm[2] = -kv
```
效果相当于让关节产生一个朝向 `ctrl` 值的弹性恢复力，力的大小与位置偏差成正比、与速度成反比（阻尼）。直观理解：
```
torque ≈ kp * (ctrl - q) - kv * qvel
```
其中 `ctrl` 是目标角度（用户滑块值或程序设定值），`q` 是当前关节角度，`qvel` 是关节角速度。

#### 修改执行器类型

改成力矩控制（而不是位置控制）：
```xml
<general name="joint1" joint="j1_yaw" ctrllimited="true" ctrlrange="-10 10"
         gaintype="fixed" gainprm="1 0 0" biastype="affine" biasprm="0 1 0"/>
```
这样 `ctrl` 直接等于输出力矩（N·m）。

---

## 二、修改控制程序 (run_ntu_arm.py)

`run_ntu_arm.py` 是仿真的 Python 入口程序，负责加载模型、配置控制策略、启动渲染窗口。

### 2.1 修改默认伺服参数

在文件顶部修改这些常量：

```python
# 文件第 21-23 行
DEFAULT_SERVO_KP = 18.0    # 位置伺服刚度 —— 越大机械臂越"硬"
DEFAULT_SERVO_KV = 5.0     # 速度阻尼 —— 越大越"黏"，减小振荡
DEFAULT_SERVO_FORCE = 24.0 # 力矩上限 (N·m) —— 越大越有力

HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 默认初始姿态
```

**调试经验**：
- `kp` 太小 → 臂在重力下下垂
- `kp` 太大 → 位置控制时振荡
- `kv` 太小 → 停止时有回弹
- `kv` 太大 → 运动变慢
- `force` 太小 → 抬不起臂
- `force` 太大 → 可能不稳定

**推荐调试范围**：

| 场景 | KP | KV | Force |
|---|---|---|---|
| 轻柔运动 | 5~10 | 2~3 | 5~10 |
| 正常演示（默认） | 18 | 5 | 24 |
| 高刚度工业 | 50~100 | 10~20 | 50~100 |
| 被动重力观察 | 0 | 0 | 0（用 --passive-gravity） |

---

### 2.2 编写自定义运动轨迹

#### 方法一：修改 demo 模式

找到 `demo_pose()` 函数（约第 85 行），替换为你的轨迹：

```python
def demo_pose(t: float) -> list[float]:
    """自定义轨迹：前 3 秒不动，之后开始画圆"""
    if t < 3.0:
        # 初始等待
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        t2 = t - 3.0
        return [
            0.8 * math.sin(0.5 * t2),       # J1: 正弦摆动
            -0.3,                            # J2: 固定
            0.5 + 0.2 * math.sin(t2 * 0.8), # J3: 小幅摆动
            0.5 * math.sin(0.6 * t2),       # J4: 滚转摆动
            0.4,                             # J5: 固定
            1.0 * math.sin(1.2 * t2),       # J6: 工具滚转
        ]
```

运行：`python run_ntu_arm.py --demo`

#### 方法二：编写姿态序列（waypoint 模式）

添加一个新函数，实现分段运动：

```python
def waypoint_sequence(t: float) -> list[float]:
    """按时间顺序访问 4 个目标点"""
    # 定义目标点 [j1, j2, j3, j4, j5, j6]
    waypoints = [
        [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0],  # home
        [ 0.5, -0.3,  0.6,  0.0,  0.4,  0.0],  # 点1
        [-0.5, -0.5,  1.0,  0.8,  0.6, -0.5],  # 点2
        [ 0.3, -0.2,  0.8, -0.5,  0.2,  1.2],  # 点3
        [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0],  # 回到 home
    ]

    duration_per_point = 2.0  # 每个点停留 2 秒
    total_duration = duration_per_point * (len(waypoints) - 1)

    # 计算当前在哪个区间
    phase = (t % total_duration) / duration_per_point
    idx = int(phase)
    frac = phase - idx

    if idx >= len(waypoints) - 1:
        return waypoints[-1]

    # 线性插值
    return [
        waypoints[idx][i] + frac * (waypoints[idx + 1][i] - waypoints[idx][i])
        for i in range(6)
    ]
```

#### 方法三：读取外部轨迹文件

```python
import json
import numpy as np

def trajectory_from_file(filename: str = "trajectory.json"):
    """从 JSON 文件读取轨迹点，用线性插值播放"""
    with open(filename) as f:
        data = json.load(f)
    # data 格式: {"times": [0, 1, 2, ...], "joints": [[q1,...], [q2,...], ...]}
    times = np.array(data["times"])
    joints = np.array(data["joints"])

    def pose_func(t: float) -> list[float]:
        t_wrapped = t % times[-1]  # 循环播放
        # 查找所在区间并插值
        idx = np.searchsorted(times, t_wrapped) - 1
        idx = max(0, min(idx, len(times) - 2))
        frac = (t_wrapped - times[idx]) / (times[idx + 1] - times[idx])
        return (joints[idx] + frac * (joints[idx + 1] - joints[idx])).tolist()

    return pose_func
```

---

### 2.3 编写末端执行器（夹爪）控制

当前的夹爪是滑动关节控制的平行二指。控制方法：

```python
def set_gripper(model, data, open_fraction: float):
    """
    控制夹爪开合
    open_fraction: 0.0 = 全闭, 1.0 = 全开
    """
    max_open = 0.018  # 对应 gripper_upper_slide 的 range="0 0.018"
    target = max_open * open_fraction

    # 上指关节的 qpos 索引
    upper_joint_id = model.joint("gripper_upper_slide").id
    lower_joint_id = model.joint("gripper_lower_slide").id

    data.qpos[upper_joint_id] = target
    data.qpos[lower_joint_id] = target

    mujoco.mj_forward(model, data)


def close_gripper_gradually(model, data, speed: float, dt: float):
    """逐步闭合夹爪"""
    upper_id = model.joint("gripper_upper_slide").id
    lower_id = model.joint("gripper_lower_slide").id

    current = data.qpos[upper_id]
    new_pos = max(0.0, current - speed * dt)
    data.qpos[upper_id] = new_pos
    data.qpos[lower_id] = new_pos
```

在仿真的主循环中使用：
```python
while viewer.is_running():
    # ... 其他逻辑 ...

    # 按时间序列控制夹爪
    if 5.0 < data.time < 6.0:
        set_gripper(model, data, 0.0)  # 闭合
    elif data.time >= 6.0:
        set_gripper(model, data, 1.0)  # 打开

    mujoco.mj_step(model, data)
    viewer.sync()
```

---

### 2.4 添加键盘交互

MuJoCo viewer 本身支持键盘交互。要添加自定义键盘逻辑，需要在 `while` 循环中检测：

```python
def handle_keyboard(viewer, model, data):
    """读取键盘并执行对应操作"""
    import glfw  # 需要 pip install glfw

    # 注意：mujoco.viewer 内部使用 glfw
    # 直接访问需要了解 viewer 内部结构
    # 更简单的方式是使用 mujoco 的被动回调
    pass
```

**更实用的方式** — 使用 MuJoCo 的 `MjUIState` 读取滑块值：

```python
# 在循环中读取控制面板的滑块值
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 读取用户通过滑块设置的目标角度
        target_q = data.ctrl.copy()

        # 在此基础上叠加自己的控制逻辑
        # ...

        mujoco.mj_step(model, data)
        viewer.sync()
```

**另一种方式** — 添加自定义快捷键（使用 MuJoCo viewer 的回调机制）：

```python
def key_callback(keycode: int):
    """自定义键盘回调"""
    # keycode 是 glfw 的键码
    if keycode == 82:  # R 键
        print("Resetting to home pose!")
        set_pose(model, data, HOME_POSE)
    elif keycode == 71:  # G 键
        print("Opening gripper!")
        set_gripper(model, data, 1.0)

# 注意：较新版本的 mujoco viewer 可能不直接暴露 key_callback
# 目前最简单的方式是把控制逻辑写在 demo 或自定义函数中
```

---

### 2.5 实现闭环控制（如逆运动学）

#### 用雅可比矩阵做末端速度控制

```python
import numpy as np

def move_tcp_to_target(model, data, target_pos: np.ndarray, target_quat: np.ndarray | None = None,
                       kp_pos: float = 10.0, kp_ori: float = 5.0):
    """
    用雅可比转置做末端位姿控制
    """
    # 获取当前末端位置和雅可比
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    tcp_id = model.site("tcp").id

    mujoco.mj_jacSite(model, data, jacp, jacr, tcp_id)

    # 位置误差
    current_pos = data.site("tcp").xpos.copy()
    pos_error = target_pos - current_pos

    # 姿态误差
    if target_quat is not None:
        current_quat = data.site("tcp").xquat.copy()
        # mju_subQuat(res, qa, qb) = qa * inv(qb)，即从 qb 到 qa 的差异四元数
        error_quat = np.zeros(4)
        mujoco.mju_subQuat(error_quat, target_quat, current_quat)
        # 将差异四元数转为旋转向量（方向=旋转轴，大小=旋转角度）
        ori_error = np.zeros(3)
        mujoco.mju_quat2Vel(ori_error, error_quat, 1.0)
    else:
        jacr = np.zeros_like(jacr)
        ori_error = np.zeros(3)

    # 组合雅可比
    J = np.vstack([jacp, jacr])
    error = np.concatenate([kp_pos * pos_error, kp_ori * ori_error])

    # 用伪逆计算关节速度
    J_pinv = np.linalg.pinv(J)
    dq = J_pinv @ error

    # 积分到位置
    data.qpos[:6] += dq[:6] * model.opt.timestep

    # 更新 ctrl
    data.ctrl[:6] = data.qpos[:6]
```

#### 用 mj_inverse 做计算力矩控制

```python
def computed_torque_control(model, data, target_q: np.ndarray, target_dq: np.ndarray,
                            kp: float = 100.0, kd: float = 20.0):
    """
    计算力矩控制 = 逆动力学 + PD 修正
    """
    # 计算 PD 力矩
    q_error = target_q - data.qpos[:6]
    dq_error = target_dq - data.qvel[:6]

    # 设置加速度目标
    target_ddq = kp * q_error + kd * dq_error
    data.qacc[:6] = target_ddq

    # 用 MuJoCo 逆动力学计算所需力矩
    mujoco.mj_inverse(model, data)
    tau = data.qfrc_inverse[:6].copy()

    # 施加力矩：将执行器改为纯力矩模式
    # 方法一：用 qfrc_applied 直接施力（需先在 XML 中禁用对应关节的 actuator）
    data.qfrc_applied[:6] = tau

    # 方法二：或者通过 ctrl 配合 gainprm=[1 0 0], biasprm=[0 1 0] 施力
    # data.ctrl[:6] = tau
    # 注意：qfrc_applied 和 ctrl 只能选一种，同时使用会重复施力
```

---

### 2.6 记录与导出仿真数据

#### 记录到内存并保存为 CSV

```python
import csv

# 在循环前初始化
log = []
log_interval = 0.01  # 每 10ms 记录一次
last_log_time = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)

        # 定时记录
        if data.time - last_log_time >= log_interval:
            last_log_time = data.time
            log.append({
                'time': data.time,
                'q1': data.qpos[0],
                'q2': data.qpos[1],
                'q3': data.qpos[2],
                'q4': data.qpos[3],
                'q5': data.qpos[4],
                'q6': data.qpos[5],
                'tcp_x': data.sensor("tcp_pos").data[0],
                'tcp_y': data.sensor("tcp_pos").data[1],
                'tcp_z': data.sensor("tcp_pos").data[2],
            })

        viewer.sync()

# 退出后保存
if log:
    with open('simulation_log.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        writer.writerows(log)
    print(f"Saved {len(log)} data points to simulation_log.csv")
```

#### 使用 numpy 保存

```python
import numpy as np

log = []
# ... 在循环中 log.append([data.time, *data.qpos[:6], *data.sensor("tcp_pos").data]) ...

# 保存
np.savez('simulation_data.npz',
         time=np.array([row[0] for row in log]),
         joint_positions=np.array([row[1:7] for row in log]),
         tcp_positions=np.array([row[7:10] for row in log]))
```

---

### 2.7 添加多个物体（抓取任务）

#### 第一步：在 XML 中添加物体

在 `robticarm.xml` 的 `<worldbody>` 末尾添加：

```xml
<!-- 在 </worldbody> 前添加 -->
<body name="object_1" pos="0.35 -0.10 0.06">
    <freejoint/>
    <geom name="obj1_geom" type="cylinder" size="0.02 0.04" rgba="0.90 0.20 0.20 1"/>
</body>
<body name="object_2" pos="0.35 0.10 0.06">
    <freejoint/>
    <geom name="obj2_geom" type="box" size="0.025 0.025 0.025" rgba="0.20 0.60 0.90 1"/>
</body>
```

#### 第二步：在 Python 中检测抓取

```python
def is_object_grasped(model, data, object_geom_name: str) -> bool:
    """检测某个物体是否在夹爪之间"""
    # 注意：contact.geom1 和 contact.geom2 存储的是 geom ID，
    # 不能用 body ID 直接比较，必须用 geom ID
    obj_geom_id = model.geom(object_geom_name).id
    upper_finger_id = model.geom("gripper_finger_top").id
    lower_finger_id = model.geom("gripper_finger_bottom").id

    # 检查物体和两个手指之间是否都有接触
    contacts_upper = False
    contacts_lower = False

    for i in range(data.ncon):
        contact = data.contact[i]
        geom_ids = {contact.geom1, contact.geom2}
        # obj_geom_id 必须是 geom ID，不能是 body ID
        if obj_geom_id in geom_ids:
            if upper_finger_id in geom_ids:
                contacts_upper = True
            if lower_finger_id in geom_ids:
                contacts_lower = True

    return contacts_upper and contacts_lower

# 调用示例：传入 geom 的名称而非 body 名称
# is_object_grasped(model, data, "obj1_geom")
```

---

### 2.8 修改革命周期设置

#### 修改相机初始视角

```python
# 在 run_ntu_arm.py 的 viewer 启动后（约第 140 行）
viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
viewer.cam.distance = 0.8     # 相机距离
viewer.cam.azimuth = 135      # 水平角度（度）
viewer.cam.elevation = -25    # 俯仰角度（度）
viewer.cam.lookat[:] = [0.12, 0.0, 0.28]  # 注视点
```

#### 修改窗口名字

```python
# 目前用的是 launch_passive，不直接暴露窗口标题
# 如需修改，可以考虑使用 launch 模式并自定义
```

#### 固定最大仿真时间

```python
max_sim_time = 30.0  # 30 秒后自动退出

while viewer.is_running():
    mujoco.mj_step(model, data)

    if data.time > max_sim_time:
        print("Simulation time limit reached.")
        break

    viewer.sync()
```

---

## 三、修改 README.md

README.md 是项目的说明文档，应随模型和控制程序的变化同步更新。

需要更新的内容通常包括：
- 运行命令的变化
- 新的命令行参数
- 模型参数的变化（如修改了关节范围、连杆长度等）
- 新增的功能说明

---

## 四、常见需求速查表

| 需求 | 文件 | 修改位置 |
|---|---|---|
| 改机械臂长度 | `robticarm.xml` | geom 的 `fromto`/`pos`/`size` |
| 改关节运动范围 | `robticarm.xml` | joint 的 `range` |
| 改颜色/外观 | `robticarm.xml` | `<material>` 的 `rgba` |
| 改重量 | `robticarm.xml` | geom 的 `density` 或添加 `mass` |
| 改关节阻尼 | `robticarm.xml` | `<joint>` 的 `damping` 或 `<default>` |
| 改重力 | `robticarm.xml` | `<option>` 的 `gravity` |
| 添加传感器 | `robticarm.xml` | `<sensor>` 块 |
| 添加执行器 | `robticarm.xml` | `<actuator>` 块 |
| 添加预设姿态 | `robticarm.xml` | `<keyframe>` 块 |
| 改伺服刚度/阻尼 | `run_ntu_arm.py` / 命令行参数 | `DEFAULT_SERVO_*` 常量 |
| 写自动运动轨迹 | `run_ntu_arm.py` | `demo_pose()` 函数 |
| 加夹爪控制 | `run_ntu_arm.py` | 自定义函数 + 主循环 |
| 加逆运动学控制 | `run_ntu_arm.py` | 自定义控制函数 |
| 记录仿真数据 | `run_ntu_arm.py` | 主循环中添加记录逻辑 |
| 改相机视角 | `run_ntu_arm.py` | `viewer.cam.*` 参数 |
| 改仿真步长 | `robticarm.xml` | `<option>` 的 `timestep` |
| 添加碰撞物体 | `robticarm.xml` | `<worldbody>` 中添加 body+freejoint |
| 改地板大小 | `robticarm.xml` | `floor` geom 的 `size` |
| 改光照 | `robticarm.xml` | `<visual>` 块 |

---

## 五、运动学链结构参考

```
世界原点 (0, 0, 0) — 地面
  │
  └─ base_fixed (固定底座)
     位置: (0, 0, 0)
     包含: 底板、J1电机、皮带轮、皮带
     │
     └─ j1_yaw_frame
        位置: (0, 0, 0.045)
        关节: j1_yaw — 绕 Z 轴旋转 ±180°
        包含: 转台、立柱、前盖板
        │
        └─ j2_pitch_frame
           位置: (0, 0, 0.135)
           关节: j2_pitch — 绕 Y 轴旋转 ±135°
           包含: 肩部电机、侧板、上臂连杆 (长 304mm)
           │
           └─ j3_elbow_frame
              位置: (0, 0, 0.304)
              关节: j3_pitch — 绕 Y 轴旋转 ±150°
              包含: 肘部电机、前臂连杆、腕部连杆 (长 349.8mm)
              │
              └─ wrist_center_frame
                 位置: (0.3498, 0, 0)
                 参考点: wrist_center (site)
                 │
                 └─ j4_roll_frame
                    位置: (0, 0, 0)
                    关节: j4_roll — 绕 X 轴旋转 ±180°
                    包含: J4 电机、输出法兰
                    │
                    └─ j5_pitch_frame
                       位置: (0.105, 0, 0)
                       关节: j5_wrist_pitch — 绕 Y 轴旋转 ±90°
                       包含: 腕部俯仰关节、工具连杆
                       │
                       └─ j6_tool_roll_frame
                          位置: (0.085, 0, 0)
                          关节: j6_tool_roll — 绕 X 轴旋转 ±180°
                          包含: 滚转体、工具杆、夹爪
                          │
                          ├─ gripper_upper_finger
                          │   关节: gripper_upper_slide (滑动 0~18mm)
                          │
                          └─ gripper_lower_finger
                              关节: gripper_lower_slide (滑动 0~18mm)

TCP (末端工具中心点): (0.190, 0, 0) 相对于 j6_tool_roll_frame
```

---

## 调试技巧

1. **模型不加载** → 检查 XML 语法（标签是否闭合、属性名是否正确）
2. **机械臂散架** → 检查 body 的 `pos` 是否正确，子 body 是否从父 body 原点出发
3. **碰撞/穿过地面** → 检查 `contype`/`conaffinity` 是否为 0
4. **伺服不起作用** → 检查 `forcerange` 是否够大
5. **仿真很慢** → 减少 `iterations`、降低几何体复杂度；增大 `timestep` 也可能加速但过大会导致不稳定
6. **颜色不对** → 检查 RGBA 的 A（alpha）是否为 1（不透明）
7. **关节不转动** → 检查 `range` 是否覆盖了目标角度
8. **Python 报错** → 检查 mujoco 版本（`pip show mujoco`），建议 ≥ 3.0

---

*最后更新：2026-08-03*
