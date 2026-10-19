/**
 * @file diff_wrist.h
 * @brief 差动腕部解耦模块: 两个 M2006 电机 ↔ 两个虚拟关节 (J5 腕俯仰, J6 工具自转)
 *
 * 物理结构:
 *   两个 M2006 电机 (A, B) 通过 1:1 锥齿轮耦合, 产生两个自由度的运动:
 *     - 同向旋转 → J5 腕部俯仰 (wrist pitch)
 *     - 反向旋转 → J6 末端自转 (tool roll)
 *
 * 坐标系:
 *   - 电机输入: M2006 累计角度 (deg)
 *   - 中间量:   电机输出轴角度 = 电机累计角度 / 减速比 (deg)
 *   - 关节输出: J5/J6 虚拟关节角度 (rad)
 *
 * 正解 (Motor → Joint):
 *   θ_motorA_out = dir_A · θ_motorA_total / gear_ratio
 *   θ_motorB_out = dir_B · θ_motorB_total / gear_ratio
 *   J5(rad) = dir_J5 · (θ_motorA_out + θ_motorB_out) / (2·bevel_ratio) · π/180
 *   J6(rad) = dir_J6 · (θ_motorA_out - θ_motorB_out) / (2·bevel_ratio) · π/180
 *
 * 逆解 (Joint → Motor):
 *   J5_deg = dir_J5 · J5(rad) · 180/π
 *   J6_deg = dir_J6 · J6(rad) · 180/π
 *   θ_motorA = dir_A · (J5_deg + J6_deg) · bevel_ratio · gear_ratio
 *   θ_motorB = dir_B · (J5_deg - J6_deg) · bevel_ratio · gear_ratio
 *
 * @note 方向系数 (±1) 用于匹配实际安装极性, 默认全为 +1
 * @note 减速比 gear_ratio=36 (M2006), 锥齿轮比 bevel_ratio=1 (1:1)
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#ifndef DIFF_WRIST_H
#define DIFF_WRIST_H

#include <stdint.h>

/* ================================================================
 * 数据结构
 * ================================================================ */

/**
 * @brief 差动腕部配置参数
 *
 * 所有方向系数为 +1 或 −1, 用于补偿电机安装方向和关节定义方向的不一致。
 * 零值在 Init() 中自动回退为 +1。
 */
typedef struct {
    float motor_gear_ratio;  /**< 电机减速比 (转子角 / 输出轴角), M2006 = 36 */
    float bevel_ratio;       /**< 锥齿轮传动比 (电机输出 / 虚拟关节), 默认 1 */
    float motor_a_dir;       /**< 电机 A 安装方向, +1 或 −1 */
    float motor_b_dir;       /**< 电机 B 安装方向, +1 或 −1 */
    float joint5_dir;        /**< J5 (腕俯仰) 虚拟关节方向, +1 或 −1 */
    float joint6_dir;        /**< J6 (工具自转) 虚拟关节方向, +1 或 −1 */
} DiffWrist_Config_t;

/**
 * @brief 差动腕部运行时状态
 *
 * 保存最近一次 Motor↔Joint 解算的中间值和结果。
 * 可以在控制循环中直接读取 diff_wrist.j5_angle/j6_angle。
 */
typedef struct {
    DiffWrist_Config_t config;        /**< 配置参数 (方向系数、减速比) */
    float motorA_out_angle;           /**< 电机 A 输出轴角度 (deg, 已除减速比) */
    float motorB_out_angle;           /**< 电机 B 输出轴角度 (deg, 已除减速比) */
    float j5_angle;                   /**< J5 虚拟腕部俯仰角 (rad) */
    float j6_angle;                   /**< J6 虚拟工具自转角 (rad) */
} DiffWrist_t;

/* ================================================================
 * API 函数
 * ================================================================ */

void DiffWrist_Init(DiffWrist_t *diff, const DiffWrist_Config_t *config);
void DiffWrist_MotorToJoint(DiffWrist_t *diff, float m2006a_total, float m2006b_total);
void DiffWrist_JointToMotor(DiffWrist_t *diff, float j5_target, float j6_target,
                             float *m2006a_ref, float *m2006b_ref);
void DiffWrist_MotorSpeedToJointSpeed(const DiffWrist_t *diff, float m2006a_speed, float m2006b_speed,
                                       float *j5_speed, float *j6_speed);

#endif /* DIFF_WRIST_H */
