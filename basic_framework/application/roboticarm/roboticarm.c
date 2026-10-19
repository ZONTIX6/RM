/**
 * @file roboticarm.c
 * @brief 机械臂应用层实现 — 电机驱动注入、模块编排、控制模式分派
 *
 * ## 架构定位
 *
 * 本模块是 YueLu 三层框架的 **Application 层**, 位于 Module 层之上。
 * 它不实现任何算法 (运动学/轨迹/重力/差动解耦全部在 Module 层),
 * 只做以下五件事:
 *
 *   1. **配置组装**: 从 robot_def.h 宏读取参数, 填入各 Module 的配置结构体
 *   2. **电机注入**: 初始化 6 个电机驱动实例, 挂载重力前馈指针
 *   3. **命令获取**: 从 Pub-Sub 消息中心读取 robot_cmd 发来的控制指令
 *   4. **模式分派**: 根据 arm_mode 选择 ZERO_FORCE / POSITION / CARTESIAN 控制分支
 *   5. **反馈推送**: 将当前关节位置/速度/状态打包发布给 robot_cmd
 *
 * ## 数据流向 (一个控制周期, 200 Hz)
 *
 * @code
 * robot_cmd (arm_cmd_send)
 *     │  Pub-Sub
 *     ▼
 * SubGetMessage → arm_cmd_recv
 *     │
 *     ├─[ZERO_FORCE]────→ ArmStopAllMotors → ArmFillFeedback(ONLINE) → PubPush
 *     │
 *     ├─[POSITION]──────→ ArmBuildJointTargetFromCmd → ArmRunJointTrajectory
 *     │                      │                              │
 *     │                      │  cmd J5/J6 deg→rad           ├─ Traj_JointInit (若目标变化)
 *     │                      │  ArmKine_ClampJointLimits     ├─ Traj_JointStep (梯形插值)
 *     │                      │                              └─ ArmSetJointRefs → 6× motor
 *     │
 *     └─[CARTESIAN]─────→ RPY→Rot → ArmPoseTargetChanged?
 *                            │              │
 *                            │   YES ───→ ArmKine_IK (可达性校验)
 *                            │              │  ├─ 不可达 → Traj_CartReset + 保持
 *                            │              │  └─ 可达   → Traj_CartInit (直线+SLERP)
 *                            │              │
 *                            │   NO (或已建) → Traj_CartStep (IK逐点)
 *                            │                    ├─ IK失败×N → ArmSyncTargets
 *                            │                    └─ IK成功   → ArmSetJointRefs
 * @endcode
 *
 * ## 电机配置
 *
 * | 关节 | 电机 | 驱动总线 | 反馈量 | PID 环 | 前馈类型 |
 * |------|------|---------|--------|--------|---------|
 * | J1-J3 | DM | CAN2 | position (rad) | 角度→速度 串级 | grav_torque_dm (N·m) |
 * | J4 | DJI GM6020 | CAN1 | total_angle (deg) | 角度→速度 串级 | grav_torque_dji_j4 (电流) |
 * | J5-J6 | DJI M2006×2 | CAN1 | total_angle (deg) | 角度→速度 串级 | grav_torque_dji_wrist (电流, 差动映射) |
 *
 * ## CAN ID 分配
 *
 * | 总线 | 设备 | tx_id | 反馈 ID |
 * |------|------|-------|---------|
 * | CAN1 | 底盘 M3508 | 1-4 | 0x201-0x204 |
 * | CAN1 | J4 GM6020 | 7 | 0x20B |
 * | CAN1 | J5 M2006 | 5 | 0x205 |
 * | CAN1 | J6 M2006 | 6 | 0x206 |
 * | CAN2 | J1 DM | 1 | 0x11 |
 * | CAN2 | J2 DM | 2 | 0x12 |
 * | CAN2 | J3 DM | 3 | 0x13 |
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#include "roboticarm.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_log.h"
#include "arm_kinematics.h"
#include "trajectory_planner.h"
#include "gravity_compensation.h"
#include "diff_wrist.h"
#include "general_def.h"    // DEGREE_2_RAD, RAD_2_DEGREE
#include <string.h>

/* ================================================================
 * 应用层常量
 * ================================================================ */

/** @brief 控制任务周期 (s), 对应 200 Hz */
#define ARM_TASK_DT           0.005f

/** @brief 关节空间目标变化判定阈值 (rad²), 平方和小于此值视为目标未变 */
#define ARM_TARGET_EPS        0.0001f

/** @brief 笛卡尔空间位置变化判定阈值 (mm²) */
#define ARM_CART_POS_EPS_MM2  1.0f

/** @brief 笛卡尔空间姿态变化判定阈值 (Frobenius 范数平方) */
#define ARM_CART_ORI_EPS      0.0001f

/** @brief 电机进入就绪状态所需的最小反馈帧数 */
#define ARM_MIN_FEEDBACK_COUNT 2

/** @brief 笛卡尔轨迹连续 IK 失败上限, 超过则放弃当前轨迹并保持 */
#define CART_IK_MAX_FAIL      5

/* ================================================================
 * 模块级静态变量 — Pub-Sub 通信
 * ================================================================ */

/** @brief 订阅 "arm_cmd", 接收 robot_cmd 发来的控制指令 */
static Subscriber_t *arm_cmd_sub;

/** @brief 发布 "arm_feed", 向 robot_cmd 推送当前关节状态 */
static Publisher_t *arm_feed_pub;

/** @brief 最近一帧收到的控制命令 (目标角度 / 笛卡尔位姿 / 模式 / 夹爪) */
static RoboticArm_Ctrl_Cmd_s arm_cmd_recv;

/** @brief 待发布的反馈数据 (关节位置/速度/状态) */
static RoboticArm_Upload_Data_s arm_feed_data;

/* ================================================================
 * 模块级静态变量 — 电机与执行器
 * ================================================================ */

/** @brief J1-J3 DM 电机实例指针 */
static DMMotorInstance *dm_j1, *dm_j2, *dm_j3;

/** @brief J4 GM6020, J5/J6 M2006 差动腕部电机实例指针 */
static DJIMotorInstance *dji_j4, *dji_j5, *dji_j6;

/** @brief 差动腕部解耦实例 (正反解状态) */
static DiffWrist_t diff_wrist;

/* ================================================================
 * 模块级静态变量 — 轨迹规划
 * ================================================================ */

/** @brief 关节空间轨迹实例 (梯形速度剖面) */
static JointTrajectory_t joint_traj;

/** @brief 笛卡尔空间轨迹实例 (直线 + SLERP 插值) */
static CartesianTrajectory_t cart_traj;

/* ================================================================
 * 模块级静态变量 — 目标缓存与状态
 * ================================================================ */

/** @brief 上一次收到的关节目标 (rad), 用于判断是否需要新建轨迹 */
static Vec6 current_target_joints;

/** @brief 上一次收到的末端目标位姿, 用于判断笛卡尔目标是否变化 */
static EndEffectorPose_t current_cart_target;

/** @brief 上一帧的控制模式, 用于检测模式切换 */
static robotic_arm_mode_e last_arm_mode = ARM_ZERO_FORCE;

/** @brief 笛卡尔轨迹连续 IK 失败计数, 超过 CART_IK_MAX_FAIL 则放弃 */
static int cart_ik_fail_count = 0;

/* ================================================================
 * 模块级静态变量 — 重力补偿前馈
 *
 * 由 ArmUpdateGravityFeedforward() 逐帧更新, 通过指针注入电机串级控制。
 * CAN 中断回调中电机控制器直接读取这些值作为力矩/电流前馈。
 * ================================================================ */

/** @brief J1-J3 DM 电机重力前馈值 (N·m), 直接由速度环输出端注入 */
static float grav_torque_dm[3];

/** @brief J4 GM6020 重力前馈值 (DJI 电流单位), 由 N·m 经 ARM_DJI6020_TORQUE_TO_CURRENT 转换 */
static float grav_torque_dji_j4;

/** @brief J5-J6 M2006 差动腕部重力前馈值 (DJI 电流单位),
 *         由关节力矩 N·m 经差动映射 + ARM_M2006_TORQUE_TO_CURRENT 转换 */
static float grav_torque_dji_wrist[2];

/** @brief 运行时是否已完成首拍目标同步 (电机反馈就绪后置位) */
static uint8_t arm_runtime_ready = 0;

/* ================================================================
 * 电机反馈就绪检查
 * ================================================================ */

/**
 * @brief 检查全部 6 个电机是否均收到足够帧数的 CAN 反馈
 *
 * 上电后 CAN 中断需要数帧才能稳定, 反馈不足时电机角度读数不可靠。
 * 若任一电机未就绪, 整个机械臂进入离线保护 (零力矩 + APP_OFFLINE)。
 *
 * @return 1=全部就绪, 0=有关节离线
 */
static uint8_t ArmMotorFeedbackReady(void)
{
    return dm_j1->feedback_count >= ARM_MIN_FEEDBACK_COUNT &&
           dm_j2->feedback_count >= ARM_MIN_FEEDBACK_COUNT &&
           dm_j3->feedback_count >= ARM_MIN_FEEDBACK_COUNT &&
           dji_j4->feedback_count >= ARM_MIN_FEEDBACK_COUNT &&
           dji_j5->feedback_count >= ARM_MIN_FEEDBACK_COUNT &&
           dji_j6->feedback_count >= ARM_MIN_FEEDBACK_COUNT;
}

/* ================================================================
 * 电机启停
 * ================================================================ */

/** @brief 急停所有 6 个电机 (零力矩输出), 用于 ZERO_FORCE 或离线保护 */
static void ArmStopAllMotors(void)
{
    DMMotorStop(dm_j1); DMMotorStop(dm_j2); DMMotorStop(dm_j3);
    DJIMotorStop(dji_j4); DJIMotorStop(dji_j5); DJIMotorStop(dji_j6);
}

/** @brief 使能所有 6 个电机 (恢复串级 PID 控制), 用于 POSITION / CARTESIAN 模式 */
static void ArmEnableAllMotors(void)
{
    DMMotorEnable(dm_j1); DMMotorEnable(dm_j2); DMMotorEnable(dm_j3);
    DJIMotorEnable(dji_j4); DJIMotorEnable(dji_j5); DJIMotorEnable(dji_j6);
}

/* ================================================================
 * 电机 ↔ 运动学 坐标转换
 * ================================================================ */

/**
 * @brief 读取当前 6 个关节的实际角度 (rad)
 *
 * 数据通路:
 *   - J1-J3: DM measure.position (rad) → 直接使用
 *   - J4:    GM6020 total_angle (deg) → × DEGREE_2_RAD → rad
 *   - J5-J6: M2006 total_angle (deg) → DiffWrist_MotorToJoint → 虚拟关节 rad
 *   - 全部:  减去 home_offset (由 ArmKine_GetJointAnglesFromActuators 完成)
 *
 * @param[out] joints 6 个实际关节角度 (rad)
 */
static void ArmReadCurrentJoints(Vec6 joints)
{
    float dm_pos[3] = {dm_j1->measure.position, dm_j2->measure.position,
                       dm_j3->measure.position};
    float j4_angle_rad = dji_j4->measure.total_angle * DEGREE_2_RAD;
    float wrist_joints[2];

    DiffWrist_MotorToJoint(&diff_wrist, dji_j5->measure.total_angle, dji_j6->measure.total_angle);
    wrist_joints[0] = diff_wrist.j5_angle;
    wrist_joints[1] = diff_wrist.j6_angle;
    ArmKine_GetJointAnglesFromActuators(dm_pos, j4_angle_rad, wrist_joints, joints);
}

/**
 * @brief 将 6 个关节目标角度写入各电机的位置环参考值
 *
 * 数据通路 (逆过程):
 *   - ArmKine_JointToActuatorRefs: 关节角 → 各电机参考值 (rad)
 *   - J4: rad → × RAD_2_DEGREE → deg → DJIMotorSetRef
 *   - J5-J6: 虚拟关节 rad → DiffWrist_JointToMotor → M2006 deg → DJIMotorSetRef
 *
 * @param[in] joints 6 个关节目标角度 (rad)
 */
static void ArmSetJointRefs(const Vec6 joints)
{
    float dm_refs[3], j4_ref_rad, wrist_refs[2], dji_refs[2];
    ArmKine_JointToActuatorRefs(joints, dm_refs, &j4_ref_rad, wrist_refs);
    DiffWrist_JointToMotor(&diff_wrist, wrist_refs[0], wrist_refs[1], &dji_refs[0], &dji_refs[1]);
    DMMotorSetRef(dm_j1, dm_refs[0]);
    DMMotorSetRef(dm_j2, dm_refs[1]);
    DMMotorSetRef(dm_j3, dm_refs[2]);
    DJIMotorSetRef(dji_j4, j4_ref_rad * RAD_2_DEGREE);
    DJIMotorSetRef(dji_j5, dji_refs[0]);
    DJIMotorSetRef(dji_j6, dji_refs[1]);
}

/* ================================================================
 * 轨迹管理
 * ================================================================ */

/** @brief 重置关节 + 笛卡尔两套轨迹, 清零 IK 失败计数 */
static void ArmResetTrajectories(void)
{
    Traj_JointReset(&joint_traj);
    Traj_CartReset(&cart_traj);
    cart_ik_fail_count = 0;
}

/**
 * @brief 将当前目标同步到实际反馈值 (上电首拍或急停恢复时调用)
 *
 * 用实际关节角覆写 current_target_joints (防止启动跳变),
 * 并调用 FK 计算当前末端位姿作为笛卡尔目标基准 (防止误判目标变化)。
 * 置位 arm_runtime_ready = 1, 后续周期不再重入。
 */
static void ArmSyncTargetsToFeedback(void)
{
    ArmReadCurrentJoints(current_target_joints);
    ArmKine_FK(current_target_joints, &current_cart_target);
    ArmResetTrajectories();
    arm_runtime_ready = 1;
}

/* ================================================================
 * 目标变化检测
 * ================================================================ */

/**
 * @brief 判断两个 6 维关节目标向量的欧氏距离平方是否超过阈值
 * @param[in] a 上一次目标 (rad)
 * @param[in] b 本次目标 (rad)
 * @return 平方距离
 */
static float ArmJointTargetDiff(const Vec6 a, const Vec6 b)
{
    float diff = 0.0f;
    for (int i = 0; i < ARM_DOF; ++i) { float d = a[i]-b[i]; diff += d*d; }
    return diff;
}

/**
 * @brief 判断两个末端位姿目标的差异是否超过阈值
 *
 * 位置: 欧氏距离平方 > ARM_CART_POS_EPS_MM2
 * 姿态: Frobenius 范数平方 > ARM_CART_ORI_EPS
 *
 * @param[in] a 已缓存的目标位姿
 * @param[in] b 最新收到的目标位姿
 * @return 1=需要重建轨迹, 0=保持当前轨迹
 */
static uint8_t ArmPoseTargetChanged(const EndEffectorPose_t *a, const EndEffectorPose_t *b)
{
    float pos_diff = 0.0f, ori_diff = 0.0f;
    for (int i = 0; i < 3; ++i) { float d = a->position[i]-b->position[i]; pos_diff += d*d; }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float d = a->orientation[i][j]-b->orientation[i][j]; ori_diff += d*d;
        }
    return (pos_diff > ARM_CART_POS_EPS_MM2) || (ori_diff > ARM_CART_ORI_EPS);
}

/* ================================================================
 * 重力补偿前馈更新
 * ================================================================ */

/**
 * @brief 逐帧更新 6 个电机的重力补偿前馈值
 *
 * 完整数据通路:
 *
 *   1. 读当前关节角 → FKWithTransforms → 获得 transforms[6]
 *   2. GravComp_ComputeTorque → 得到 6 个关节重力矩 (N·m)
 *   3. 按电机类型分配前馈:
 *      - J1-J3 (DM):      直接 N·m → grav_torque_dm[0:2]
 *      - J4 (GM6020):     N·m × ARM_DJI6020_TORQUE_TO_CURRENT → grav_torque_dji_j4
 *      - J5-J6 (M2006):   N·m 经差动映射 + ARM_M2006_TORQUE_TO_CURRENT → grav_torque_dji_wrist
 *
 * 差动映射公式:
 * @code
 *   scale = ARM_M2006_TORQUE_TO_CURRENT / (2·bevel_ratio)
 *   τ_motorA = dir_A · (dir_J5·τ₅ + dir_J6·τ₆) · scale
 *   τ_motorB = dir_B · (dir_J5·τ₅ − dir_J6·τ₆) · scale
 * @endcode
 *
 * @note 该函数在每个控制周期调用一次, 在 PID 计算之前更新前馈值,
 *       保证本帧 PID 输出已经包含重力补偿分量。
 */
static void ArmUpdateGravityFeedforward(void)
{
    Vec6 grav_joints;
    ArmReadCurrentJoints(grav_joints);

    EndEffectorPose_t grav_pose;
    Mat4 transforms[ARM_DOF];
    ArmKine_FKWithTransforms(grav_joints, &grav_pose, transforms);

    Vec6 grav_torque_nm;
    GravComp_ComputeTorque(transforms, grav_torque_nm);

    // J1-J3: DM 直接使用 N·m
    grav_torque_dm[0] = grav_torque_nm[0];
    grav_torque_dm[1] = grav_torque_nm[1];
    grav_torque_dm[2] = grav_torque_nm[2];

    // J4: GM6020 直接驱动, N·m → 电流单位
    grav_torque_dji_j4 = grav_torque_nm[3] * ARM_DJI6020_TORQUE_TO_CURRENT;

    // J5-J6: M2006 差动腕部, 需要差动映射
    float wrist_current_scale = ARM_M2006_TORQUE_TO_CURRENT / (2.0f * diff_wrist.config.bevel_ratio);
    grav_torque_dji_wrist[0] = diff_wrist.config.motor_a_dir *
                               (diff_wrist.config.joint5_dir * grav_torque_nm[4] +
                                diff_wrist.config.joint6_dir * grav_torque_nm[5]) * wrist_current_scale;
    grav_torque_dji_wrist[1] = diff_wrist.config.motor_b_dir *
                               (diff_wrist.config.joint5_dir * grav_torque_nm[4] -
                                diff_wrist.config.joint6_dir * grav_torque_nm[5]) * wrist_current_scale;
}

/* ================================================================
 * 反馈数据填充
 * ================================================================ */

/**
 * @brief 填充 arm_feed_data 结构体, 准备向 robot_cmd 发布
 *
 * 速度反馈来源:
 *   - J1-J3: DM measure.velocity (rad/s, 内部已计算)
 *   - J4:    GM6020 speed_aps (deg/s) → × DEGREE_2_RAD → rad/s
 *   - J5-J6: M2006 speed_aps (deg/s) → DiffWrist_MotorSpeedToJointSpeed → 虚拟关节速度 (deg/s)
 *
 * 位置反馈:
 *   - J1-J4: 运动学内部统一 rad → 直接填充
 *   - J5-J6: 运动学内部统一 rad → × RAD_2_DEGREE → deg (与 robot_def.h 中的 cmd 单位一致)
 *
 * @param[in] state 当前机械臂应用状态 (APP_ONLINE / APP_OFFLINE / APP_ERROR)
 */
static void ArmFillFeedback(App_Status_e state)
{
    Vec6 joints;
    float joint5_speed, joint6_speed;
    ArmReadCurrentJoints(joints);

    // 位置 (J1-J4: rad, J5-J6: deg)
    arm_feed_data.joint1_pos = joints[0];
    arm_feed_data.joint2_pos = joints[1];
    arm_feed_data.joint3_pos = joints[2];
    arm_feed_data.joint4_pos = joints[3];
    arm_feed_data.joint5_pos = joints[4] * RAD_2_DEGREE;
    arm_feed_data.joint6_pos = joints[5] * RAD_2_DEGREE;

    // 速度 (J1-J3: rad/s, J4: rad/s 由 deg/s 转换, J5-J6: deg/s)
    arm_feed_data.joint1_vel = dm_j1->measure.velocity;
    arm_feed_data.joint2_vel = dm_j2->measure.velocity;
    arm_feed_data.joint3_vel = dm_j3->measure.velocity;
    arm_feed_data.joint4_vel = dji_j4->measure.speed_aps * DEGREE_2_RAD;
    DiffWrist_MotorSpeedToJointSpeed(&diff_wrist, dji_j5->measure.speed_aps, dji_j6->measure.speed_aps,
                                      &joint5_speed, &joint6_speed);
    arm_feed_data.joint5_vel = joint5_speed;
    arm_feed_data.joint6_vel = joint6_speed;

    arm_feed_data.gripper_state = arm_cmd_recv.gripper;
    arm_feed_data.arm_state = state;
}

/* ================================================================
 * 命令解析与轨迹运行
 * ================================================================ */

/**
 * @brief 从 arm_cmd_recv 提取 6 个关节目标角度, 统一量纲为 rad
 *
 * J5/J6 在 cmd 协议中是 deg (与 ARM_JOINT5/6_MIN/MAX 和 feedback 一致),
 * 此处转为 rad 后供运动学和轨迹规划使用。
 * 最终调用 ArmKine_ClampJointLimits 确保目标在软限位内。
 *
 * @param[out] target_joints 输出 6 个目标关节角 (rad)
 */
static void ArmBuildJointTargetFromCmd(Vec6 target_joints)
{
    target_joints[0] = arm_cmd_recv.joint1_angle;
    target_joints[1] = arm_cmd_recv.joint2_angle;
    target_joints[2] = arm_cmd_recv.joint3_angle;
    target_joints[3] = arm_cmd_recv.joint4_angle;
    target_joints[4] = arm_cmd_recv.joint5_angle * DEGREE_2_RAD;
    target_joints[5] = arm_cmd_recv.joint6_angle * DEGREE_2_RAD;
    ArmKine_ClampJointLimits(target_joints);
}

/**
 * @brief 关节空间位置控制: 梯形轨迹规划 + 电机参考写入
 *
 * 流程:
 *   1. 从 cmd 提取目标关节角
 *   2. 若模式切换或目标变化超过 ARM_TARGET_EPS, 新建 JointTrajectory
 *   3. 调用 Traj_JointStep 步进, 输出当前时刻的插值关节角度
 *   4. 写入 6 个电机参考值
 *
 * 防御: 在调用 Traj_JointStep 前用当前关节角预填充 output_joints,
 *       防止轨迹处于 IDLE 时 output_joints 未初始化导致电机跳变。
 *
 * @param[in] mode 控制模式 (POSITION 或 VELOCITY 回退), 用于检测模式切换
 */
static void ArmRunJointTrajectory(robotic_arm_mode_e mode)
{
    Vec6 target_joints;
    ArmBuildJointTargetFromCmd(target_joints);

    if (last_arm_mode != mode || ArmJointTargetDiff(target_joints, current_target_joints) > ARM_TARGET_EPS) {
        Vec6 start_joints;
        ArmReadCurrentJoints(start_joints);
        Traj_JointInit(&joint_traj, start_joints, target_joints,
                       ARM_TRAJ_V_NORM, ARM_TRAJ_A_NORM, ARM_TASK_DT);
        memcpy(current_target_joints, target_joints, sizeof(Vec6));
    }

    Vec6 output_joints;
    ArmReadCurrentJoints(output_joints);  // 防御: 若轨迹处于 IDLE, 保持当前位置
    Traj_JointStep(&joint_traj, output_joints);
    ArmSetJointRefs(output_joints);
}

/* ================================================================
 * 模块配置构建函数 (robot_def.h 宏 → Module Config 结构体)
 *
 * 这些 static 函数仅在 RoboticArmInit() 中调用一次,
 * 将分散在 robot_def.h 中的宏定义组装为各 Module 的配置结构体。
 * ================================================================ */

/**
 * @brief 从 robot_def.h 宏组装 ArmKine_Config_t
 *
 * 注意:
 *   - J5/J6 的关节限位和 home_offset 在 def 中是 deg, 此处 × DEGREE_2_RAD 转为 rad
 *   - IK 参数 (收敛阈值/阻尼/步长/迭代次数) 硬编码在此, 如需调参请修改本函数
 */
static void ArmBuildKinematicsConfig(ArmKine_Config_t *config)
{
    memset(config, 0, sizeof(ArmKine_Config_t));

    // DH 参数 (a: mm, alpha: rad, d: mm)
    config->dh[0] = (DH_Param_t){ARM_DH_a1, ARM_DH_alpha1, ARM_DH_d1, 0.0f};
    config->dh[1] = (DH_Param_t){ARM_DH_a2, ARM_DH_alpha2, ARM_DH_d2, 0.0f};
    config->dh[2] = (DH_Param_t){ARM_DH_a3, ARM_DH_alpha3, ARM_DH_d3, 0.0f};
    config->dh[3] = (DH_Param_t){ARM_DH_a4, ARM_DH_alpha4, ARM_DH_d4, 0.0f};
    config->dh[4] = (DH_Param_t){ARM_DH_a5, ARM_DH_alpha5, ARM_DH_d5, 0.0f};
    config->dh[5] = (DH_Param_t){ARM_DH_a6, ARM_DH_alpha6, ARM_DH_d6, 0.0f};

    // 关节限位 (J5/J6: deg → rad)
    config->joint_min[0] = ARM_JOINT1_MIN; config->joint_max[0] = ARM_JOINT1_MAX;
    config->joint_min[1] = ARM_JOINT2_MIN; config->joint_max[1] = ARM_JOINT2_MAX;
    config->joint_min[2] = ARM_JOINT3_MIN; config->joint_max[2] = ARM_JOINT3_MAX;
    config->joint_min[3] = ARM_JOINT4_MIN; config->joint_max[3] = ARM_JOINT4_MAX;
    config->joint_min[4] = ARM_JOINT5_MIN * DEGREE_2_RAD; config->joint_max[4] = ARM_JOINT5_MAX * DEGREE_2_RAD;
    config->joint_min[5] = ARM_JOINT6_MIN * DEGREE_2_RAD; config->joint_max[5] = ARM_JOINT6_MAX * DEGREE_2_RAD;

    // 零点偏置 (J5/J6: deg → rad)
    config->joint_home_offset[0] = ARM_JOINT1_HOME;
    config->joint_home_offset[1] = ARM_JOINT2_HOME;
    config->joint_home_offset[2] = ARM_JOINT3_HOME;
    config->joint_home_offset[3] = ARM_JOINT4_HOME;
    config->joint_home_offset[4] = ARM_JOINT5_HOME * DEGREE_2_RAD;
    config->joint_home_offset[5] = ARM_JOINT6_HOME * DEGREE_2_RAD;

    // IK 求解器参数
    config->ik.converge_threshold = 1.0f;    // 位置收敛 1 mm
    config->ik.orient_threshold   = 0.005f;  // 姿态收敛 ~0.3°
    config->ik.lambda             = 0.5f;    // DLS 阻尼
    config->ik.alpha              = 0.8f;    // 步长因子
    config->ik.max_iterations     = 100;
}

/**
 * @brief 从 robot_def.h 宏组装 GravComp_Config_t
 *
 * 连杆质量和质心坐标需通过 CAD 或实测标定填入 robot_def.h。
 * 重力向量设为基础坐标系 Z 轴向上: g = [0, 0, -9.81]。
 */
static void ArmBuildGravityConfig(GravComp_Config_t *config)
{
    memset(config, 0, sizeof(GravComp_Config_t));
    config->links[0] = (ArmLinkDyn_t){ARM_MASS_LINK1, {ARM_COM1_X, ARM_COM1_Y, ARM_COM1_Z}};
    config->links[1] = (ArmLinkDyn_t){ARM_MASS_LINK2, {ARM_COM2_X, ARM_COM2_Y, ARM_COM2_Z}};
    config->links[2] = (ArmLinkDyn_t){ARM_MASS_LINK3, {ARM_COM3_X, ARM_COM3_Y, ARM_COM3_Z}};
    config->links[3] = (ArmLinkDyn_t){ARM_MASS_LINK4, {ARM_COM4_X, ARM_COM4_Y, ARM_COM4_Z}};
    config->links[4] = (ArmLinkDyn_t){ARM_MASS_LINK5, {ARM_COM5_X, ARM_COM5_Y, ARM_COM5_Z}};
    config->links[5] = (ArmLinkDyn_t){ARM_MASS_LINK6, {ARM_COM6_X, ARM_COM6_Y, ARM_COM6_Z}};
    config->gravity[0] = 0.0f;
    config->gravity[1] = 0.0f;
    config->gravity[2] = -ARM_GRAVITY_MAG;
}

/**
 * @brief 从 robot_def.h 宏组装 DiffWrist_Config_t
 *
 * 方向系数 (±1) 用于匹配实际安装极性, 默认为全 +1。
 * 上板后若发现关节运动方向与预期相反, 修改 robot_def.h 中的对应宏即可。
 */
static void ArmBuildDiffWristConfig(DiffWrist_Config_t *config)
{
    config->motor_gear_ratio = ARM_DIFF_M2006_GEAR_RATIO;
    config->bevel_ratio      = ARM_DIFF_BEVEL_RATIO;
    config->motor_a_dir      = ARM_DIFF_MOTOR_A_DIR;
    config->motor_b_dir      = ARM_DIFF_MOTOR_B_DIR;
    config->joint5_dir       = ARM_DIFF_JOINT5_DIR;
    config->joint6_dir       = ARM_DIFF_JOINT6_DIR;
}

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief 机械臂应用初始化
 *
 * 调用顺序:
 *   1. 注册 Pub-Sub (arm_cmd 订阅 + arm_feed 发布)
 *   2. 初始化 3 个 DM 电机 (J1-J3, CAN2, 角度+速度串级 PID, 重力前馈)
 *   3. 初始化 J4 GM6020 (CAN1, 角度+速度串级 PID, 重力前馈)
 *   4. 初始化 2 个 M2006 (J5-J6 差动腕部, CAN1, 角度+速度串级 PID, 重力前馈)
 *   5. 初始化差动腕部解耦实例
 *   6. 初始化运动学引擎 (注入 DH 参数 + 关节限位 + IK 配置)
 *   7. 初始化重力补偿 (注入连杆质量/质心 + 重力向量)
 *
 * @note 必须在 RTOS 调度器启动前调用 (由 RobotInit() → RoboticArmInit())
 * @note 电机反馈在 RTOS + CAN 中断运行后才可靠, 实际目标同步推迟到 RoboticArmTask 首拍
 */
void RoboticArmInit(void)
{
    ArmKine_Config_t kine_config;
    GravComp_Config_t gravity_config;
    DiffWrist_Config_t diff_config;

    // ---- 1. 注册 Pub-Sub ----
    arm_cmd_sub  = SubRegister("arm_cmd",  sizeof(RoboticArm_Ctrl_Cmd_s));
    arm_feed_pub = PubRegister("arm_feed", sizeof(RoboticArm_Upload_Data_s));

    // ---- 2. 初始化 3 个 DM 电机 (J1-J3), CAN2, 角度+速度串级 PID ----
    Motor_Init_Config_s dm_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 1,
            .rx_id      = 0x11,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.1f,
                .IntegralLimit = 1.0f,  .MaxOut = 30.0f,     // 输出限幅 rad/s
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .Output_LPF_RC = 0.3,
            },
            .speed_PID = {
                .Kp = 0.5f, .Ki = 0.0f, .Kd = 0.0f,
                .IntegralLimit = 5.0f,  .MaxOut = 15.0f,     // 输出限幅 N·m
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .Output_LPF_RC = 0.3,
            },
            .current_PID = {.Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, .MaxOut = 0.0f},  // 不启用力矩环
        },
        .controller_setting_init_config = {
            .angle_feedback_source  = MOTOR_FEED,
            .speed_feedback_source  = MOTOR_FEED,
            .outer_loop_type        = ANGLE_LOOP,
            .close_loop_type        = ANGLE_AND_SPEED_LOOP,
        },
    };

    // J1: tx=1, rx=0x11, 前馈指针 → grav_torque_dm[0]
    dm_config.controller_param_init_config.current_feedforward_ptr = &grav_torque_dm[0];
    dm_config.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
    dm_config.can_init_config.tx_id = 1; dm_config.can_init_config.rx_id = 0x11;
    dm_j1 = DMMotorInit(&dm_config);

    // J2: tx=2, rx=0x12
    dm_config.controller_param_init_config.current_feedforward_ptr = &grav_torque_dm[1];
    dm_config.can_init_config.tx_id = 2; dm_config.can_init_config.rx_id = 0x12;
    dm_j2 = DMMotorInit(&dm_config);

    // J3: tx=3, rx=0x13
    dm_config.controller_param_init_config.current_feedforward_ptr = &grav_torque_dm[2];
    dm_config.can_init_config.tx_id = 3; dm_config.can_init_config.rx_id = 0x13;
    dm_j3 = DMMotorInit(&dm_config);

    // ---- 3. 初始化 J4 DJI GM6020, CAN1, 角度+速度串级 PID ----
    Motor_Init_Config_s j4_dji_config = {
        .can_init_config = {
            .can_handle = ARM_J4_CAN_HANDLE,   // &hcan1
            .tx_id      = ARM_J4_TX_ID,         // 7
            .rx_id      = ARM_J4_RX_ID,         // 0x20B
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = ARM_J4_ANGLE_KP, .Ki = 0.0f, .Kd = ARM_J4_ANGLE_KD,
                .IntegralLimit = 500,  .MaxOut = ARM_J4_ANGLE_MAXOUT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .Output_LPF_RC = 0.3,
            },
            .speed_PID = {
                .Kp = ARM_J4_SPEED_KP, .Ki = 0.0f, .Kd = 0.0f,
                .IntegralLimit = 2000, .MaxOut = ARM_J4_SPEED_MAXOUT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .Output_LPF_RC = 0.3,
            },
            .current_feedforward_ptr = &grav_torque_dji_j4,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_AND_SPEED_LOOP,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020,
    };
    dji_j4 = DJIMotorInit(&j4_dji_config);

    // ---- 4. 初始化 2 个 DJI M2006 差动腕部电机 (J5-J6), CAN1 ----
    Motor_Init_Config_s dji_config = {
        .can_init_config = { .can_handle = &hcan1 },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5.0f, .Ki = 0.0f, .Kd = 0.5f,
                .IntegralLimit = 500,  .MaxOut = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .Output_LPF_RC = 0.3,
            },
            .speed_PID = {
                .Kp = 3.0f, .Ki = 0.0f, .Kd = 0.0f,
                .IntegralLimit = 2000, .MaxOut = 10000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_AND_SPEED_LOOP,
        },
        .motor_type = M2006,
    };

    // J5 (M2006-A): tx=5, 反馈 0x205
    dji_config.controller_param_init_config.current_feedforward_ptr = &grav_torque_dji_wrist[0];
    dji_config.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
    dji_config.can_init_config.tx_id = 5;
    dji_j5 = DJIMotorInit(&dji_config);

    // J6 (M2006-B): tx=6, 反馈 0x206
    dji_config.controller_param_init_config.current_feedforward_ptr = &grav_torque_dji_wrist[1];
    dji_config.can_init_config.tx_id = 6;
    dji_j6 = DJIMotorInit(&dji_config);

    // ---- 5. 初始化差动腕部、运动学、重力补偿三个算法模块 ----
    ArmBuildDiffWristConfig(&diff_config);
    DiffWrist_Init(&diff_wrist, &diff_config);

    ArmBuildKinematicsConfig(&kine_config);
    ArmKine_Init(&kine_config);

    ArmBuildGravityConfig(&gravity_config);
    GravComp_Init(&gravity_config);

    // ---- 6. 运行时状态初始化 ----
    ArmResetTrajectories();
    arm_runtime_ready = 0;  // 首拍 RoboticArmTask 会完成目标同步

    LOGINFO("[robotic_arm] init done, 3xDM(J1-J3) + 1xGM6020(J4) + 2xM2006 diff wrist(J5-J6)");
}

/* ================================================================
 * 主控制任务 (200 Hz, 由 RobotTask 调用)
 * ================================================================ */

/**
 * @brief 机械臂主控制任务, 每个控制周期执行一次
 *
 * ## 执行流程
 *
 * @code
 * SubGetMessage(arm_cmd)  ← 获取最新控制指令
 *     │
 *     ├─ 电机未就绪? → ArmStopAllMotors + ArmFillFeedback(OFFLINE) → return
 *     │
 *     ├─ 首拍? → ArmSyncTargetsToFeedback (用实际关节角初始化目标, 防跳变)
 *     │
 *     ├─ ArmUpdateGravityFeedforward (更新 6 个前馈值)
 *     │
 *     └─ 模式分派:
 *         ├─ ZERO_FORCE  → 急停 + 同步目标 + APP_ONLINE
 *         ├─ CARTESIAN   → RPY→Rot → 可达性IK → CartTraj → IK逐点 → SetRefs
 *         ├─ POSITION    → JointTraj → SetRefs
 *         ├─ VELOCITY    → (回退到 POSITION)
 *         └─ 未知模式    → 急停 + OFFLINE
 *     │
 *     └─ PubPushMessage(arm_feed)
 * @endcode
 *
 * ## 笛卡尔模式特殊处理
 *
 *   - **目标不可达 (初始化 IK 失败)**: 将 cart_traj 置为 IDLE,
 *     记录目标位姿 (避免重复触发同一不可达目标), 保持当前关节角度。
 *   - **轨迹步进中 IK 失败**: cart_ik_fail_count 累加,
 *     超过 CART_IK_MAX_FAIL (5) 后调用 ArmSyncTargetsToFeedback 放弃当前轨迹。
 *     单次 IK 发散可能只是数值问题, 允许连续重试。
 *
 * @note 调用频率 200 Hz (每 5 ms), 由 RTOS 任务调度保证
 * @note 本函数不使用阻塞或延时, 全部计算在单拍内完成
 */
void RoboticArmTask(void)
{
    // ---- 1. 获取控制指令 ----
    SubGetMessage(arm_cmd_sub, &arm_cmd_recv);

    // ---- 2. 离线保护: 任一电机反馈不足 → 停全部电机 ----
    if (!ArmMotorFeedbackReady()) {
        ArmStopAllMotors();
        ArmFillFeedback(APP_OFFLINE);
        PubPushMessage(arm_feed_pub, &arm_feed_data);
        return;
    }

    // ---- 3. 首拍初始化: 用实际关节角同步所有内部目标 (防上电跳变) ----
    if (!arm_runtime_ready)
        ArmSyncTargetsToFeedback();

    // ---- 4. 更新重力前馈 (在所有模式之前, 保证本帧 PID 已含前馈) ----
    ArmUpdateGravityFeedforward();

    // ---- 5. 控制模式分派 ----

    if (arm_cmd_recv.arm_mode == ARM_ZERO_FORCE) {
        // ===== 零力矩模式: 急停 + 保持目标同步 =====
        ArmStopAllMotors();
        ArmSyncTargetsToFeedback();
        ArmFillFeedback(APP_ONLINE);
    }
    else if (arm_cmd_recv.arm_mode == ARM_CARTESIAN_MODE) {
        // ===== 笛卡尔空间控制 =====
        ArmEnableAllMotors();

        // 5a. 构造目标位姿 (RPY → 旋转矩阵)
        EndEffectorPose_t target_pose;
        target_pose.position[0] = arm_cmd_recv.target_x;
        target_pose.position[1] = arm_cmd_recv.target_y;
        target_pose.position[2] = arm_cmd_recv.target_z;
        Vec3 target_rpy = {arm_cmd_recv.target_roll, arm_cmd_recv.target_pitch, arm_cmd_recv.target_yaw};
        RobotMath_RPYToRot(target_rpy, target_pose.orientation);

        // 5b. 目标变化 → 尝试新建轨迹
        if (last_arm_mode != ARM_CARTESIAN_MODE ||
            ArmPoseTargetChanged(&target_pose, &current_cart_target))
        {
            Vec6 current_joints;
            ArmReadCurrentJoints(current_joints);

            Vec6 target_joints;
            int ik_ret = ArmKine_IK(&target_pose, current_joints, target_joints);
            if (ik_ret < 0) {
                // 目标不可达: 记录目标 (避免下帧重复触发), 保持当前位置
                LOGWARNING("[robotic_arm] IK target unreachable, holding current pose");
                Traj_CartReset(&cart_traj);
                memcpy(&current_cart_target, &target_pose, sizeof(EndEffectorPose_t));
                memcpy(current_target_joints, current_joints, sizeof(Vec6));
                cart_ik_fail_count = 0;
            } else {
                // 目标可达: 用当前 FK 作为轨迹起点, 创建笛卡尔直线轨迹
                EndEffectorPose_t start_pose;
                ArmKine_FK(current_joints, &start_pose);
                Traj_CartInit(&cart_traj, &start_pose, &target_pose,
                              ARM_CART_MAX_LIN_VEL, ARM_CART_MAX_LIN_ACC, current_joints, ARM_TASK_DT);
                memcpy(&current_cart_target, &target_pose, sizeof(EndEffectorPose_t));
            }
        }

        // 5c. 轨迹步进 (若非 IDLE)
        if (cart_traj.state == TRAJ_IDLE) {
            // 目标已判定不可达, 保持当前关节目标
        } else {
            Vec6 output_joints;
            int ret = Traj_CartStep(&cart_traj, ArmKine_IK, output_joints);
            if (ret < 0) {
                cart_ik_fail_count++;
                if (cart_ik_fail_count >= CART_IK_MAX_FAIL) {
                    LOGWARNING("[robotic_arm] IK failed %d times consecutively, holding", cart_ik_fail_count);
                    ArmSyncTargetsToFeedback();
                }
            } else {
                cart_ik_fail_count = 0;
                ArmKine_ClampJointLimits(output_joints);
                ArmSetJointRefs(output_joints);
                memcpy(current_target_joints, output_joints, sizeof(Vec6));
            }
        }
        ArmFillFeedback(APP_ONLINE);
    }
    else if (arm_cmd_recv.arm_mode == ARM_POSITION_MODE) {
        // ===== 关节空间位置控制 =====
        ArmEnableAllMotors();
        ArmRunJointTrajectory(ARM_POSITION_MODE);
        ArmFillFeedback(APP_ONLINE);
    }
    else {
        // ===== 其他模式 =====
        if (arm_cmd_recv.arm_mode == ARM_VELOCITY_MODE) {
            // VELOCITY_MODE 暂未实现, 回退到位置模式 (仅警告一次)
            static uint8_t vel_warn_once = 0;
            if (!vel_warn_once) {
                LOGWARNING("[robotic_arm] VELOCITY_MODE not implemented, falling back to POSITION_MODE");
                vel_warn_once = 1;
            }
            ArmEnableAllMotors();
            ArmRunJointTrajectory(ARM_VELOCITY_MODE);
            ArmFillFeedback(APP_ONLINE);
        } else {
            // 未知模式: 急停保护
            ArmStopAllMotors();
            ArmSyncTargetsToFeedback();
            ArmFillFeedback(APP_OFFLINE);
        }
    }

    // ---- 6. 保存模式 + 推送反馈 ----
    last_arm_mode = arm_cmd_recv.arm_mode;
    PubPushMessage(arm_feed_pub, &arm_feed_data);
}
