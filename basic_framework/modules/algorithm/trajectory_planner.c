/**
 * @file trajectory_planner.c
 * @brief 机械臂轨迹规划实现
 *
 * 关节空间: 基于归一化梯形速度曲线的同步关节运动。
 *   所有关节同时启停, 位移最大的关键关节决定运动总时间。
 *
 * 笛卡尔空间: 末端直线路径 + SLERP 姿态插值, 逐点 IK 解算。
 *   位置和姿态变化统一到路径等效长度, 使用相同的梯形速度剖面。
 *
 * 梯形速度曲线:
 *   加速段 t∈[0, ta]:      s(t) = ½·a·t²
 *   匀速段 t∈[ta, T−td]:   s(t) = s(ta) + v_max·(t−ta)
 *   减速段 t∈[T−td, T]:   s(t) = 1 − ½·a·(T−t)²
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#include "trajectory_planner.h"
#include "arm_math.h"       // arm_sqrt_f32
#include "bsp_log.h"
#include <string.h>
#include <math.h>

/* ================================================================
 * 内部常量
 * ================================================================ */

/** @brief 笛卡尔轨迹位置变化判定阈值 (mm), 小于此值视为静止 */
#define TRAJ_CART_MIN_LIN_LENGTH_MM      0.1f

/** @brief 笛卡尔轨迹姿态变化判定阈值 (rad), 小于此值视为无旋转 */
#define TRAJ_CART_MIN_ORIENT_ANGLE_RAD   0.001f

/** @brief 姿态角差 → 等效线位移的折算半径 (mm),
 *         用于将旋转量纳入直线路径度量, 使纯旋转运动也能正常规划 */
#define TRAJ_ORIENT_EQUIV_RADIUS_MM      100.0f

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/**
 * @brief 计算归一化 (s∈[0,1]) 梯形速度曲线的参数
 *
 * 判断梯形 vs 三角形:
 *   - 若 2·v_max²/a < 1 → 梯形 (有匀速段)
 *   - 否则            → 三角形 (无匀速段, 峰值速度 = √a)
 *
 * @param[in]  v_max      归一化最大速度
 * @param[in]  a          归一化加速度 (恒定, 加速和减速对称)
 * @param[out] ta         输出: 加速段时间
 * @param[out] has_cruise 输出: 1=有匀速段, 0=三角形
 * @param[out] total_time 输出: 总时间
 */
static void compute_trap_params(float v_max, float a, float *ta, int *has_cruise, float *total_time)
{
    float s_acc = v_max * v_max / (2.0f * a);  // 加速段覆盖的归一化距离

    if (2.0f * s_acc < 1.0f) {
        // 梯形: 可达 v_max
        *has_cruise = 1;
        *ta = v_max / a;
        float t_cruise = (1.0f - 2.0f * s_acc) / v_max;
        *total_time = 2.0f * (*ta) + t_cruise;
    } else {
        // 三角形: 无法到 v_max, 峰值速度 = √a, ta = 0.5T
        *has_cruise = 0;
        float v_peak;
        arm_sqrt_f32(a, &v_peak);
        *ta = v_peak / a;
        *total_time = 2.0f * (*ta);
    }
}

/**
 * @brief 根据归一化时间计算归一化位移 (梯形 / 三角形剖面)
 *
 * 推导:
 *   梯形速度曲线下面积 = 1 ⇒ v_max·(T−ta) = 1 ⇒ v_max = 1/(T−ta)
 *
 *   加速段 τ∈[0, τa]:       σ = τ² / (2·τa·(1−τa))
 *   匀速段 τ∈[τa, 1−τa]:   σ = (2τ − τa) / (2·(1−τa))
 *   减速段 τ∈[1−τa, 1]:    σ = 1 − (1−τ)² / (2·τa·(1−τa))
 *
 *   三角形 (τa=0.5):       加速 σ=2τ², 减速 σ=1−2(1−τ)²
 *
 * @param[in] t_norm     归一化时间 τ = t/T, ∈ [0,1]
 * @param[in] ta_norm    归一化加速时间 τa = ta/T
 * @param[in] has_cruise  是否有匀速段
 * @return 归一化位移 σ ∈ [0,1]
 */
static float trap_interpolate(float t_norm, float ta_norm, int has_cruise)
{
    if (t_norm >= 1.0f) return 1.0f;
    if (t_norm <= 0.0f) return 0.0f;

    if (has_cruise) {
        float denom = 2.0f * ta_norm * (1.0f - ta_norm);
        if (t_norm < ta_norm)
            return (t_norm * t_norm) / denom;                          // 加速段
        else if (t_norm < 1.0f - ta_norm)
            return (2.0f * t_norm - ta_norm) / (2.0f * (1.0f - ta_norm)); // 匀速段
        else {
            float dt = 1.0f - t_norm;
            return 1.0f - (dt * dt) / denom;                           // 减速段
        }
    } else {
        // 三角形: τa = 0.5 恒成立
        if (t_norm < 0.5f)
            return 2.0f * t_norm * t_norm;
        else {
            float dt = 1.0f - t_norm;
            return 1.0f - 2.0f * dt * dt;
        }
    }
}

/* ================================================================
 * 关节空间轨迹
 * ================================================================ */

/**
 * @brief 初始化关节空间梯形速度轨迹
 *
 * 以位移最大的关键关节为基准, 计算归一化梯形剖面参数。
 * 若所有关节位移量均 < 1e-6 rad, 直接标记为 FINISHED。
 *
 * @param[out] traj   轨迹实例 (会被 memset 清零)
 * @param[in]  start  起始关节角度 (6 维, rad)
 * @param[in]  end    目标关节角度 (6 维, rad)
 * @param[in]  v_norm 归一化最大速度 (1/s)
 * @param[in]  a_norm 归一化最大加速度 (1/s²)
 * @param[in]  dt     控制周期 (s), 用于后续 step
 */
void Traj_JointInit(JointTrajectory_t *traj, const Vec6 start, const Vec6 end,
                     float v_norm, float a_norm, float dt)
{
    memset(traj, 0, sizeof(JointTrajectory_t));
    memcpy(traj->start_pos, start, sizeof(Vec6));
    memcpy(traj->end_pos,   end,   sizeof(Vec6));
    traj->v_max_norm = v_norm;
    traj->a_max_norm = a_norm;

    // 找位移最大的关键关节
    float max_disp = 0.0f;
    for (int i = 0; i < ARM_DOF; ++i) {
        traj->displacement[i] = end[i] - start[i];
        float d = fabsf(traj->displacement[i]);
        if (d > max_disp) max_disp = d;
    }

    if (max_disp < 1e-6f) {
        traj->state = TRAJ_FINISHED;
        return;
    }

    // 归一化: v_eff/a_eff 对应等效归一化位移 s∈[0,1]
    float v_eff = v_norm / max_disp;
    float a_eff = a_norm / max_disp;
    compute_trap_params(v_eff, a_eff, &traj->ta, &traj->has_cruise, &traj->total_time);
    traj->td = traj->ta;  // 对称加速/减速

    traj->dt = dt;
    traj->elapsed_time = 0.0f;
    traj->state = TRAJ_RUNNING;

    LOGINFO("[arm_traj] joint traj init: max_disp=%.3f, T=%.3fs, ta=%.3fs, cruise=%d",
            max_disp, traj->total_time, traj->ta, traj->has_cruise);
}

/**
 * @brief 关节空间轨迹步进: 前进一个控制周期 dt
 *
 * 调用频率应与初始化时的 dt 匹配。
 * 到达终点后标记 FINISHED 并持续输出 end_pos。
 * IDLE 状态下不输出 (调用方应在调用前检查状态或预先初始化 output)。
 *
 * @param[in,out] traj   轨迹实例
 * @param[out]    output 输出当前时刻的 6 个关节角度 (rad)
 */
void Traj_JointStep(JointTrajectory_t *traj, Vec6 output)
{
    if (traj->state == TRAJ_FINISHED || traj->state == TRAJ_IDLE) {
        if (traj->state == TRAJ_FINISHED) memcpy(output, traj->end_pos, sizeof(Vec6));
        return;
    }

    traj->elapsed_time += traj->dt;

    if (traj->elapsed_time >= traj->total_time) {
        memcpy(output, traj->end_pos, sizeof(Vec6));
        traj->state = TRAJ_FINISHED;
        return;
    }

    float t_norm  = traj->elapsed_time / traj->total_time;
    float ta_norm = traj->ta / traj->total_time;
    float s = trap_interpolate(t_norm, ta_norm, traj->has_cruise);

    for (int i = 0; i < ARM_DOF; ++i)
        output[i] = traj->start_pos[i] + s * traj->displacement[i];
}

/** @brief 重置关节轨迹, 回到 IDLE 状态 */
void Traj_JointReset(JointTrajectory_t *traj)
{
    memset(traj, 0, sizeof(JointTrajectory_t));
    traj->state = TRAJ_IDLE;
}

/* ================================================================
 * 笛卡尔空间轨迹
 * ================================================================ */

/**
 * @brief 初始化笛卡尔空间直线轨迹
 *
 * 路径度量: path_length = max(直线距离, 旋转角 × 等效半径)
 * 这样纯旋转轨迹 (位置不变) 也能正常规划总时间。
 *
 * 若平移距离和旋转角都小于最小阈值, 直接标记 FINISHED。
 *
 * @param[out] traj           轨迹实例
 * @param[in]  start          起始末端位姿
 * @param[in]  end            目标末端位姿
 * @param[in]  max_lin_vel    最大线速度 (mm/s)
 * @param[in]  max_lin_acc    最大线加速度 (mm/s²)
 * @param[in]  initial_joints 当前关节角度 (rad), 作为 IK 初始猜测
 * @param[in]  dt             控制周期 (s)
 */
void Traj_CartInit(CartesianTrajectory_t *traj, const EndEffectorPose_t *start,
                    const EndEffectorPose_t *end, float max_lin_vel, float max_lin_acc,
                    const Vec6 initial_joints, float dt)
{
    memset(traj, 0, sizeof(CartesianTrajectory_t));
    memcpy(&traj->start_pose, start, sizeof(EndEffectorPose_t));
    memcpy(&traj->end_pose,   end,   sizeof(EndEffectorPose_t));
    traj->max_linear_vel = max_lin_vel;
    traj->max_linear_acc = max_lin_acc;

    // 直线距离
    float dx = end->position[0]-start->position[0];
    float dy = end->position[1]-start->position[1];
    float dz = end->position[2]-start->position[2];
    float linear_length_sq = dx*dx + dy*dy + dz*dz;
    float path_len_sqrt;
    arm_sqrt_f32(linear_length_sq, &path_len_sqrt);

    // 姿态角差, 折算为等效线位移
    float orient_angle = RobotMath_RotAngleBetween(start->orientation, end->orientation);
    float orient_equiv_length = orient_angle * TRAJ_ORIENT_EQUIV_RADIUS_MM;

    // 路径度量取 max(平移, 旋转等效), 保证纯旋转也有非零路径
    traj->path_length = (path_len_sqrt > orient_equiv_length) ? path_len_sqrt : orient_equiv_length;

    // 动静判定
    if (path_len_sqrt < TRAJ_CART_MIN_LIN_LENGTH_MM &&
        orient_angle  < TRAJ_CART_MIN_ORIENT_ANGLE_RAD) {
        traj->state = TRAJ_FINISHED;
        memcpy(traj->last_joint_angles, initial_joints, sizeof(Vec6));
        return;
    }

    // 梯形速度参数
    float v_norm = max_lin_vel / traj->path_length;
    float a_norm = max_lin_acc / traj->path_length;
    compute_trap_params(v_norm, a_norm, &traj->ta, &traj->has_cruise, &traj->total_time);
    traj->td = traj->ta;

    memcpy(traj->last_joint_angles, initial_joints, sizeof(Vec6));
    traj->dt = dt;
    traj->elapsed_time = 0.0f;
    traj->state = TRAJ_RUNNING;

    LOGINFO("[arm_traj] cart traj init: lin=%.1fmm, rot=%.3frad, metric=%.1fmm, T=%.3fs, ta=%.3fs",
            path_len_sqrt, orient_angle, traj->path_length, traj->total_time, traj->ta);
}

/**
 * @brief 笛卡尔空间轨迹步进: 前进一个控制周期, 通过 IK 得到关节角度
 *
 * 插值:
 *   - 位置: 归一化梯形剖面线性插值
 *   - 姿态: RotSlerp (底层四元数 SLERP), 保证中间姿态正交
 *
 * 然后调用注入的 IK 求解器将插值位姿转为关节角度。
 * IK 失败时保持上一帧关节角度, 调用方负责累积失败计数和错误处理。
 *
 * @param[in,out] traj      轨迹实例
 * @param[in]     ik_solver IK 求解器回调函数 (通常为 ArmKine_IK)
 * @param[out]    output    输出当前时刻的 6 个关节角度 (rad)
 * @return 0=成功; -1=失败 (轨迹 IDLE 或 IK 失败)
 */
int Traj_CartStep(CartesianTrajectory_t *traj, Traj_IKSolverFunc ik_solver, Vec6 output)
{
    if (traj->state == TRAJ_FINISHED || traj->state == TRAJ_IDLE) {
        memcpy(output, traj->last_joint_angles, sizeof(Vec6));
        return (traj->state == TRAJ_FINISHED) ? 0 : -1;
    }

    traj->elapsed_time += traj->dt;

    EndEffectorPose_t interp_pose;
    if (traj->elapsed_time >= traj->total_time) {
        // 到达终点, 直接输出终点位姿
        memcpy(&interp_pose, &traj->end_pose, sizeof(EndEffectorPose_t));
        traj->state = TRAJ_FINISHED;
    } else {
        float t_norm  = traj->elapsed_time / traj->total_time;
        float ta_norm = traj->ta / traj->total_time;
        float s = trap_interpolate(t_norm, ta_norm, traj->has_cruise);

        // 位置: 线性插值
        for (int i = 0; i < 3; ++i)
            interp_pose.position[i] = traj->start_pose.position[i]
                                    + s * (traj->end_pose.position[i] - traj->start_pose.position[i]);

        // 姿态: SLERP 球面插值, 保证中间姿态仍在 SO(3) 上
        RobotMath_RotSlerp(traj->start_pose.orientation, traj->end_pose.orientation,
                           s, interp_pose.orientation);
    }

    if (ik_solver == NULL) {
        memcpy(output, traj->last_joint_angles, sizeof(Vec6));
        return -1;
    }

    int ret = ik_solver(&interp_pose, traj->last_joint_angles, output);
    if (ret >= 0) {
        // IK 收敛: 保存为下一次初始猜测
        memcpy(traj->last_joint_angles, output, sizeof(Vec6));
        return 0;
    } else {
        // IK 失败: 保持上一帧关节角度, 不更新 last_joint_angles (下次用同一初始猜测再试)
        memcpy(output, traj->last_joint_angles, sizeof(Vec6));
        LOGWARNING("[arm_traj] IK failed at t=%.3f, s=%.3f",
                   traj->elapsed_time, traj->elapsed_time / traj->total_time);
        return -1;
    }
}

/** @brief 重置笛卡尔轨迹, 回到 IDLE 状态 */
void Traj_CartReset(CartesianTrajectory_t *traj)
{
    memset(traj, 0, sizeof(CartesianTrajectory_t));
    traj->state = TRAJ_IDLE;
}
