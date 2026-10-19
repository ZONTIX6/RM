/**
 * @file gravity_compensation.h
 * @brief 机械臂重力补偿模块: 基于 FK 中间变换计算各关节重力矩前馈
 *
 * 算法:
 *   τ_i = Σ_{k=i}^{ARM_DOF} m_k · gᵀ · (z_i × (p_com_k − p_i))
 *
 *   其中:
 *     m_k:      连杆 k 的质量 (kg)
 *     g:        基座坐标系下的重力加速度向量 (m/s²), 默认 [0, 0, −9.81]
 *     z_i:      关节 i 的旋转轴 (从 T_{0,i} 的 Z 轴列提取, Craig 改进 DH)
 *     p_i:      关节 i 坐标系原点 (从 T_{0,i} 的平移列提取)
 *     p_com_k:  连杆 k 质心在基座坐标系下的位置 = T_{0,k} · com_offset_k
 *
 * 步进约定:
 *   1. GravComp_Init() 加载质量/质心参数
 *   2. 控制循环中先调用 ArmKine_FKWithTransforms() 获取 transforms[6]
 *   3. 调用 GravComp_ComputeTorque(transforms, torques) 得到 6 个关节重力矩 (N·m)
 *   4. 由应用层按各关节电机类型转换前馈单位:
 *      - J1-J3 (DM):     直接写 torques[0:2] 到 current_feedforward_ptr
 *      - J4 (GM6020):    torques[3] × ARM_DJI6020_TORQUE_TO_CURRENT
 *      - J5-J6 (M2006):  torques[4:5] 经差动映射 + ARM_M2006_TORQUE_TO_CURRENT
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include "arm_kinematics.h"

/* ================================================================
 * 数据结构
 * ================================================================ */

/**
 * @brief 单个连杆的动力学参数
 *
 * 所有参数定义在连杆自身坐标系中 (由 DH 约定确定)。
 * 质心坐标在运动学计算中通过 T_{0,k} 变换到基座坐标系。
 */
typedef struct {
    float mass;  /**< 连杆质量 (kg) */
    Vec3  com;   /**< 质心在连杆坐标系中的位置 (mm) */
} ArmLinkDyn_t;

/**
 * @brief 重力补偿模块完整配置
 *
 * 由应用层从 robot_def.h 宏组装, 在 RoboticArmInit() 中传入。
 * gravity 向量模长通常 ≈ 9.81, 方向为重力在基座坐标系中的指向
 * (基座 Z 轴向上时, gravity = [0, 0, −9.81])。
 */
typedef struct {
    ArmLinkDyn_t links[ARM_DOF];  /**< 6 个连杆的动力学参数 */
    Vec3         gravity;         /**< 基座坐标系下的重力加速度向量 (m/s²) */
} GravComp_Config_t;

/* ================================================================
 * API 函数
 * ================================================================ */

void GravComp_Init(const GravComp_Config_t *config);
void GravComp_ComputeTorque(const Mat4 transforms[ARM_DOF], Vec6 torques);
const ArmLinkDyn_t *GravComp_GetLinkDynamics(void);
void GravComp_SetGravityVector(const Vec3 grav_dir);

#endif /* GRAVITY_COMPENSATION_H */
