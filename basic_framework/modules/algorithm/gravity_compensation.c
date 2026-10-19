/**
 * @file gravity_compensation.c
 * @brief 机械臂重力补偿实现
 *
 * 算法:
 *   对每个关节 i, 累加所有下游连杆 k (k ≥ i) 在该关节处产生的重力矩:
 *   τ_i = Σ_{k=i}^{ARM_DOF} m_k · gᵀ · (z_i × (p_com_k − p_i))
 *
 * 推导:
 *   连杆 k 的重力在基座系下为 f_k = m_k · g。
 *   该力对关节 i 原点的力矩为 (p_com_k − p_i) × f_k。
 *   该力矩在关节 i 旋转轴上的投影 (驱动关节的等效标量力矩) 为:
 *     τ_{i,k} = z_iᵀ · ((p_com_k − p_i) × f_k) = gᵀ · (z_i × (p_com_k − p_i)) · m_k
 *
 * 帧索引约定 (Craig 改进 DH):
 *   transforms[k] = T_{0,k+1}
 *   关节 i+1 (0-indexed i) 的旋转轴 z = transforms[i] 的 Z 轴列
 *   关节 i+1 的原点坐标 p = transforms[i] 的平移列
 *   连杆 k+1 的质心 p_com = transforms[k] * link_dyn[k].com
 *
 * 数值细节:
 *   - DH 参数位置单位 mm, 质量 kg, 重力加速度 m/s²
 *   - 中间计算结果 N·mm, 最终除以 1000 转为 N·m
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#include "gravity_compensation.h"
#include "bsp_log.h"
#include <string.h>
#include <math.h>    // fabsf
#include <stdint.h>  // uint8_t

/* ================================================================
 * 模块级静态变量
 * ================================================================ */

/** @brief 6 个连杆的动力学参数: 质量 (kg)、质心在连杆坐标系中的位置 (mm) */
static ArmLinkDyn_t link_dyn[ARM_DOF];

/** @brief 基座坐标系下的重力加速度向量, 默认 [0, 0, 0] (未初始化) */
static Vec3 gravity_vec = {0.0f, 0.0f, 0.0f};

/** @brief 重力补偿模块是否已初始化 (防止空跑) */
static uint8_t grav_initialized = 0;

/* ================================================================
 * 重力矩计算
 * ================================================================ */

/**
 * @brief 计算当前姿态下全部 6 个关节的重力补偿前馈力矩
 *
 * Step 0: 将各连杆质心从连杆坐标系变换到基座坐标系
 * Step 1: 对每个关节 i 累加下游连杆 k ≥ i 的贡献
 *
 * @param[in]  transforms ArmKine_FKWithTransforms() 输出的 6 个中间变换
 *                        transforms[0]=T_{0,1}, ..., transforms[5]=T_{0,6}
 * @param[out] torques    输出 6 个关节重力补偿力矩 (N·m)
 * @note 若模块未初始化则输出全零
 */
void GravComp_ComputeTorque(const Mat4 transforms[ARM_DOF], Vec6 torques)
{
    if (!grav_initialized) { memset(torques, 0, sizeof(Vec6)); return; }

    // ---- Step 0: 质心坐标变换到基座坐标系 ----
    // transforms[k] = T_{0,k+1}, link_dyn[k] 存连杆 k+1 的 com
    Vec3 com_base[ARM_DOF];
    for (int k = 0; k < ARM_DOF; ++k)
        RobotMath_TransformPoint(transforms[k], link_dyn[k].com, com_base[k]);

    // ---- Step 1: 逐关节累加 ----
    for (int i = 0; i < ARM_DOF; ++i) {
        Vec3 z_i, p_i;

        // 获取关节 i+1 的旋转轴和原点 (在基座坐标系)
        if (i == 0) {
            // 关节 1: 基座关节, α₀=0 时 transforms[0]=I, z=(0,0,1), p=(0,0,0)
            z_i[0] = 0.0f; z_i[1] = 0.0f; z_i[2] = 1.0f;
            p_i[0] = 0.0f; p_i[1] = 0.0f; p_i[2] = 0.0f;
        } else {
            // 关节 i+1: transforms[i] = T_{0,i+1}, 取 Z 轴列和位移列
            z_i[0] = transforms[i][0][2]; z_i[1] = transforms[i][1][2]; z_i[2] = transforms[i][2][2];
            p_i[0] = transforms[i][0][3]; p_i[1] = transforms[i][1][3]; p_i[2] = transforms[i][2][3];
        }

        // 累加所有下游连杆 k ≥ i
        float tau_i_nmm = 0.0f;  // 力矩累积, N·mm

        for (int k = i; k < ARM_DOF; ++k) {
            // Δp = p_com_(k+1)_base − p_i
            Vec3 dp = {com_base[k][0]-p_i[0], com_base[k][1]-p_i[1], com_base[k][2]-p_i[2]};

            // z_i × Δp
            Vec3 z_cross_dp;
            RobotMath_Vec3Cross(z_i, dp, z_cross_dp);

            // gᵀ · (z_i × Δp)
            float g_dot_cross = RobotMath_Vec3Dot(gravity_vec, z_cross_dp);

            // m_k · gᵀ · (z_i × Δp)
            tau_i_nmm += link_dyn[k].mass * g_dot_cross;
        }

        // N·mm → N·m (÷1000)
        torques[i] = tau_i_nmm / 1000.0f;
    }
}

/* ================================================================
 * 初始化和配置接口
 * ================================================================ */

/**
 * @brief 初始化重力补偿模块
 *
 * 从配置结构体加载 6 个连杆的质量、质心以及重力向量。
 * 必须在控制循环启动前调用一次。
 *
 * @param[in] config 重力补偿配置; 若 NULL 则直接返回
 */
void GravComp_Init(const GravComp_Config_t *config)
{
    if (config == NULL) return;
    memcpy(link_dyn,    config->links,   sizeof(link_dyn));
    memcpy(gravity_vec, config->gravity, sizeof(gravity_vec));
    grav_initialized = 1;
    LOGINFO("[arm_grav] gravity params loaded, g=(%.2f, %.2f, %.2f)",
            gravity_vec[0], gravity_vec[1], gravity_vec[2]);
}

/**
 * @brief 获取当前已加载的连杆动力学参数 (调试/监控用)
 * @return ArmLinkDyn_t[6] 只读指针
 */
const ArmLinkDyn_t *GravComp_GetLinkDynamics(void) { return link_dyn; }

/**
 * @brief 运行时修改重力方向向量 (用于移动基座 / 姿态变化场景)
 *
 * 典型用法: 当基座 (底盘) 倾斜时, 从 IMU 姿态推算新的 g 向本函数写入。
 *
 * @param[in] grav_dir 基座坐标系下的新重力方向向量 (m/s²), 模长应 ≈ 9.81
 */
void GravComp_SetGravityVector(const Vec3 grav_dir)
{
    gravity_vec[0] = grav_dir[0];
    gravity_vec[1] = grav_dir[1];
    gravity_vec[2] = grav_dir[2];
}
