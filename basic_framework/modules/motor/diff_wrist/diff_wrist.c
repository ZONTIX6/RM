/**
 * @file diff_wrist.c
 * @brief 差动腕部解耦实现
 *
 * 两个 M2006 电机通过锥齿轮耦合, 映射为两个正交的虚拟关节:
 *   - J5: 腕部俯仰 (pitch) — 两电机同向旋转的和
 *   - J6: 工具自转 (roll)  — 两电机反向旋转的差
 *
 * 正反解公式见 diff_wrist.h 文件头注释。
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#include "diff_wrist.h"
#include "general_def.h"   // DEGREE_2_RAD, RAD_2_DEGREE
#include <string.h>

/* ================================================================
 * 默认配置
 * ================================================================ */

/** @brief 默认参数: M2006 减速比 36, 锥齿轮 1:1, 全正向安装 */
static const DiffWrist_Config_t diff_wrist_default_config = {
    .motor_gear_ratio = 36.0f,
    .bevel_ratio      = 1.0f,
    .motor_a_dir      = 1.0f,
    .motor_b_dir      = 1.0f,
    .joint5_dir       = 1.0f,
    .joint6_dir       = 1.0f,
};

/* ================================================================
 * 初始化和配置
 * ================================================================ */

/**
 * @brief 初始化差动腕部实例
 *
 * 若 config 为 NULL 则使用默认参数 (36:1, 1:1, 全正向)。
 * 零值字段自动回退为默认值 / +1, 防止除零。
 *
 * @param[out] diff   腕部实例 (先被 memset 清零)
 * @param[in]  config 配置参数, 可为 NULL
 */
void DiffWrist_Init(DiffWrist_t *diff, const DiffWrist_Config_t *config)
{
    memset(diff, 0, sizeof(DiffWrist_t));
    if (config == NULL)
        diff->config = diff_wrist_default_config;
    else
        diff->config = *config;

    // 零值防御: 关键参数不允许为 0, 否则正反解会除零或恒为零
    if (diff->config.motor_gear_ratio == 0.0f) diff->config.motor_gear_ratio = diff_wrist_default_config.motor_gear_ratio;
    if (diff->config.bevel_ratio      == 0.0f) diff->config.bevel_ratio      = diff_wrist_default_config.bevel_ratio;
    if (diff->config.motor_a_dir      == 0.0f) diff->config.motor_a_dir      = 1.0f;
    if (diff->config.motor_b_dir      == 0.0f) diff->config.motor_b_dir      = 1.0f;
    if (diff->config.joint5_dir       == 0.0f) diff->config.joint5_dir       = 1.0f;
    if (diff->config.joint6_dir       == 0.0f) diff->config.joint6_dir       = 1.0f;
}

/* ================================================================
 * 电机 ↔ 关节 正反解
 * ================================================================ */

/**
 * @brief 电机角度 → 虚拟关节角度 (正解)
 *
 * 从两个 M2006 累计角度 (CAN 总线 total_angle, 单位 deg) 计算 J5, J6 虚拟关节角 (rad)。
 * 中间值和最终结果均写入 diff 实例, 供后续读取。
 *
 * @param[in,out] diff           腕部实例 (更新 motorA/B_out_angle 和 j5/j6_angle)
 * @param[in]     m2006a_total   电机 A 累计角度 (deg)
 * @param[in]     m2006b_total   电机 B 累计角度 (deg)
 */
void DiffWrist_MotorToJoint(DiffWrist_t *diff, float m2006a_total, float m2006b_total)
{
    const DiffWrist_Config_t *cfg = &diff->config;

    // 1. 电机端 → 输出端 (除以减速比, 乘以安装方向)
    diff->motorA_out_angle = cfg->motor_a_dir * m2006a_total / cfg->motor_gear_ratio;
    diff->motorB_out_angle = cfg->motor_b_dir * m2006b_total / cfg->motor_gear_ratio;

    float motor_out_sum  = diff->motorA_out_angle + diff->motorB_out_angle;
    float motor_out_diff = diff->motorA_out_angle - diff->motorB_out_angle;

    // 2. 差动解耦: 和→J5, 差→J6, 除以 2·锥齿轮比, 再 deg→rad
    diff->j5_angle = cfg->joint5_dir * (motor_out_sum  / (2.0f * cfg->bevel_ratio)) * DEGREE_2_RAD;
    diff->j6_angle = cfg->joint6_dir * (motor_out_diff / (2.0f * cfg->bevel_ratio)) * DEGREE_2_RAD;
}

/**
 * @brief 虚拟关节角度 → 电机参考角度 (逆解)
 *
 * 给定目标 J5, J6 (rad), 反算两个 M2006 电机的累计角度参考值 (deg)。
 * 用于将运动学输出的关节目标转换为电机位置环的 setpoint。
 *
 * @param[in,out] diff         腕部实例 (更新 j5/j6_angle 和 motorA/B_out_angle)
 * @param[in]     j5_target    目标 J5 角度 (rad)
 * @param[in]     j6_target    目标 J6 角度 (rad)
 * @param[out]    m2006a_ref   电机 A 累计角度参考值 (deg)
 * @param[out]    m2006b_ref   电机 B 累计角度参考值 (deg)
 */
void DiffWrist_JointToMotor(DiffWrist_t *diff, float j5_target, float j6_target,
                             float *m2006a_ref, float *m2006b_ref)
{
    const DiffWrist_Config_t *cfg = &diff->config;

    // 1. 关节 rad → deg, 乘以方向系数
    float j5_deg = cfg->joint5_dir * j5_target * RAD_2_DEGREE;
    float j6_deg = cfg->joint6_dir * j6_target * RAD_2_DEGREE;

    // 2. 虚拟关节 → 电机输出端: sum=J5, diff=J6, 乘以锥齿轮比
    float motorA_out_target = (j5_deg + j6_deg) * cfg->bevel_ratio;
    float motorB_out_target = (j5_deg - j6_deg) * cfg->bevel_ratio;

    // 3. 更新实例状态
    diff->j5_angle = j5_target;
    diff->j6_angle = j6_target;
    diff->motorA_out_angle = motorA_out_target;
    diff->motorB_out_angle = motorB_out_target;

    // 4. 电机输出端 → 电机累计角度 (乘以减速比和方向)
    *m2006a_ref = cfg->motor_a_dir * motorA_out_target * cfg->motor_gear_ratio;
    *m2006b_ref = cfg->motor_b_dir * motorB_out_target * cfg->motor_gear_ratio;
}

/* ================================================================
 * 电机速度 → 关节速度
 * ================================================================ */

/**
 * @brief 电机速度 → 虚拟关节速度 (用于速度反馈填充)
 *
 * 速度转换不含减速比和差动映射以外的额外积分, 与 MotorToJoint 的位置关系一致。
 * 输出速度单位与输入速度单位相同 (都是 deg/s 或都是 rad/s)。
 *
 * @param[in]  diff           腕部实例 (只读)
 * @param[in]  m2006a_speed   电机 A 速度 (如 dji_j5->measure.speed_aps, 单位 deg/s)
 * @param[in]  m2006b_speed   电机 B 速度 (单位同 A)
 * @param[out] j5_speed       输出 J5 虚拟关节速度 (单位与输入相同)
 * @param[out] j6_speed       输出 J6 虚拟关节速度
 */
void DiffWrist_MotorSpeedToJointSpeed(const DiffWrist_t *diff, float m2006a_speed, float m2006b_speed,
                                       float *j5_speed, float *j6_speed)
{
    const DiffWrist_Config_t *cfg = &diff->config;

    // 电机端 → 输出端速度 (除以减速比)
    float motorA_out_speed = cfg->motor_a_dir * m2006a_speed / cfg->motor_gear_ratio;
    float motorB_out_speed = cfg->motor_b_dir * m2006b_speed / cfg->motor_gear_ratio;

    // 差动解耦
    *j5_speed = cfg->joint5_dir * (motorA_out_speed + motorB_out_speed) / (2.0f * cfg->bevel_ratio);
    *j6_speed = cfg->joint6_dir * (motorA_out_speed - motorB_out_speed) / (2.0f * cfg->bevel_ratio);
}
