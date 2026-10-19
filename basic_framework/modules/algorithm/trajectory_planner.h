/**
 * @file trajectory_planner.h
 * @brief 机械臂轨迹规划模块: 关节空间梯形速度曲线 + 笛卡尔空间直线插补
 *
 * 关节空间:
 *   - 归一化梯形速度剖面, 所有关节同步启停
 *   - 位移最大的关节决定总时间 (关键关节法)
 *
 * 笛卡尔空间:
 *   - 末端沿直线运动, 位置线性插值, 姿态 SLERP
 *   - 每步调用外部 IK 求解器 (通过函数指针注入) 得到关节角度
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "arm_kinematics.h"

/* ================================================================
 * IK 求解器回调类型
 * ================================================================ */

/**
 * @brief IK 求解器回调函数签名
 *
 * 用于将 ArmKine_IK 等求解器以依赖注入方式传入轨迹步进函数,
 * 使轨迹规划模块不直接依赖具体的 IK 实现。
 *
 * @param  target        目标末端位姿
 * @param  initial_guess 初始关节角度 (rad)
 * @param  result        输出求解得到的关节角度 (rad)
 * @return ≥0=成功 (迭代次数); -1=失败
 */
typedef int (*Traj_IKSolverFunc)(const EndEffectorPose_t *target,
                                  const Vec6 initial_guess, Vec6 result);

/* ================================================================
 * 轨迹状态枚举
 * ================================================================ */

/** @brief 轨迹生命周期状态 */
typedef enum {
    TRAJ_IDLE     = 0,  /**< 空闲, 无轨迹 */
    TRAJ_RUNNING,       /**< 轨迹执行中 */
    TRAJ_FINISHED,      /**< 轨迹已完成 */
} TrajectoryState_e;

/* ================================================================
 * 关节空间轨迹
 * ================================================================ */

/**
 * @brief 关节空间梯形速度轨迹
 *
 * 所有关节同步运动 (同时启停)。
 * 以位移最大的关节为关键关节, 规划归一化剖面, 其余关节按比例缩放。
 *
 * v_norm 和 a_norm 是归一化参数, 对应归一化位移 s∈[0,1] 的速度和加速度。
 * 实际运动时间 T ≈ max_displacement / v_norm (若可达匀速)。
 */
typedef struct {
    Vec6  start_pos;      /**< 起始关节角度 (rad) */
    Vec6  end_pos;        /**< 目标关节角度 (rad) */
    Vec6  displacement;   /**< 各关节位移量 = end − start */

    float v_max_norm;     /**< 归一化最大速度 (1/s), 典型 0.5~2.0 */
    float a_max_norm;     /**< 归一化最大加速度 (1/s²), 典型 1.0~5.0 */

    float total_time;     /**< 规划总时间 (s) */
    float elapsed_time;   /**< 已执行时间 (s) */
    float ta;             /**< 加速段时间 (s) */
    float td;             /**< 减速段时间 (s) */
    int   has_cruise;     /**< 是否有匀速段 (1=有) */
    float dt;             /**< 控制周期 (s), 由 init 存储, step 使用 */

    TrajectoryState_e state;  /**< 当前轨迹状态 */
} JointTrajectory_t;

/* ================================================================
 * 笛卡尔空间轨迹
 * ================================================================ */

/**
 * @brief 笛卡尔空间直线轨迹
 *
 * 末端沿直线从起点运动到终点。
 * 位置: 归一化梯形剖面线性插值。
 * 姿态: 四元数 SLERP 球面插值, 保证中间姿态在 SO(3) 上。
 *
 * 路径度量采用线性长度和旋转角度的 max, 使纯旋转运动也能正常规划。
 *
 * 每步调用 IK 求解器, 因此实际运动精度取决于 IK 收敛性。
 */
typedef struct {
    EndEffectorPose_t start_pose;  /**< 起始末端位姿 */
    EndEffectorPose_t end_pose;   /**< 目标末端位姿 */

    float max_linear_vel;          /**< 最大线速度 (mm/s) */
    float max_linear_acc;          /**< 最大线加速度 (mm/s²) */

    float total_time;              /**< 规划总时间 (s) */
    float elapsed_time;            /**< 已执行时间 (s) */
    float ta, td;
    int   has_cruise;

    float path_length;             /**< 路径等效长度 (mm), = max(直线距离, 旋转等效距离) */
    Vec6  last_joint_angles;       /**< 上次 IK 解算结果, 作为下一次的初始猜测 */
    float dt;                      /**< 控制周期 (s) */

    TrajectoryState_e state;
} CartesianTrajectory_t;

/* ================================================================
 * 关节空间 API
 * ================================================================ */

void Traj_JointInit(JointTrajectory_t *traj, const Vec6 start, const Vec6 end,
                     float v_norm, float a_norm, float dt);
void Traj_JointStep(JointTrajectory_t *traj, Vec6 output);
void Traj_JointReset(JointTrajectory_t *traj);

/* ================================================================
 * 笛卡尔空间 API
 * ================================================================ */

void Traj_CartInit(CartesianTrajectory_t *traj, const EndEffectorPose_t *start,
                    const EndEffectorPose_t *end, float max_lin_vel, float max_lin_acc,
                    const Vec6 initial_joints, float dt);
int  Traj_CartStep(CartesianTrajectory_t *traj, Traj_IKSolverFunc ik_solver, Vec6 output);
void Traj_CartReset(CartesianTrajectory_t *traj);

#endif /* TRAJECTORY_PLANNER_H */
