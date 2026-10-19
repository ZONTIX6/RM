/**
 * @file arm_kinematics.c
 * @brief 6 自由度机械臂正逆运动学实现
 *
 * 正运动学 (FK):
 *   从基座向末端逐级连乘 DH 变换矩阵, 得到末端 4×4 齐次位姿。
 *   ArmKine_FKWithTransforms() 同时返回全部 6 个中间帧, 供雅可比和重力补偿复用。
 *
 * 逆运动学 (IK):
 *   基于几何雅可比的 DLS (阻尼最小二乘) 迭代求解。
 *   每次迭代: FK → PoseError → Jacobian → (JᵀJ+λ²I)·Δθ = Jᵀe → θ+=α·Δθ
 *   变阻尼: 检测到奇异时 λ ← 2λ, 上限 100; 超过上限则放弃。
 *
 * 几何雅可比:
 *   对旋转关节 i (Craig 改进 DH): J_col_i = [z_{i+1} × (p_n − p_{i+1}); z_{i+1}]
 *   z_{i+1} 和 p_{i+1} 分别从 transforms[i] (= T_{0,i+1}) 的 Z 轴列和平移列提取
 *
 * @note 参考: "Introduction to Robotics: Mechanics and Control" by John J. Craig
 *       参考: "Robot Modeling and Control" by Spong, Hutchinson, Vidyasagar
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#include "arm_kinematics.h"
#include "arm_math.h"       // arm_cos_f32, arm_sin_f32, arm_sqrt_f32
#include "bsp_log.h"
#include <string.h>
#include <math.h>           // fabsf, atan2f, asinf
#include <stdint.h>         // uint8_t

/* ================================================================
 * 模块级静态变量 (由 ArmKine_Init() 初始化)
 * ================================================================ */

/** @brief DH 参数表, 6 行 Craig 改进 DH */
static DH_Param_t arm_dh[ARM_DOF];

/** @brief DLS-IK 求解器配置 (可通过 SetIKConfig 运行时调整) */
static IK_Config_t ik_config = {
    .converge_threshold = 1.0f,
    .orient_threshold   = 0.005f,
    .lambda             = 0.5f,
    .alpha              = 0.8f,
    .max_iterations     = 100,
};

/** @brief 最近一次 IK 求解的迭代次数 (调试/性能监控) */
static int ik_iterations = 0;

/** @brief 最近一次 IK 求解的最终位姿误差平方和 */
static float ik_final_error = 0.0f;

/** @brief 各关节软限位 [下限, 上限], 单位 rad */
static float joint_min[ARM_DOF];
static float joint_max[ARM_DOF];

/** @brief 各关节零点偏置 (rad), 实际关节角 = 电机读数 − home_offset */
static float joint_home_offset[ARM_DOF];

/** @brief DH 参数表是否已初始化 */
static uint8_t dh_initialized = 0;

/* ================================================================
 * DH 变换计算
 * ================================================================ */

/**
 * @brief 计算单个 Craig 改进 DH 变换矩阵: T = Rx(α)·Tx(a)·Rz(θ)·Tz(d)
 *
 * 展开得到 4×4 齐次矩阵:
 * @code
 *   T = [ cθ       -sθ        0      a         ]
 *       [ sθ·cα    cθ·cα    -sα    -sα·d      ]
 *       [ sθ·sα    cθ·sα     cα     cα·d      ]
 *       [ 0         0         0      1         ]
 * @endcode
 *
 * @param[in]  dh DH 参数 (a/α/d 为常量, θ 为变量)
 * @param[out] T  输出 4×4 齐次变换矩阵
 */
void ArmKine_DHTransform(const DH_Param_t *dh, Mat4 T)
{
    float ct, st, ca, sa;
    ct = arm_cos_f32(dh->theta);   st = arm_sin_f32(dh->theta);
    ca = arm_cos_f32(dh->alpha);   sa = arm_sin_f32(dh->alpha);

    T[0][0] = ct;        T[0][1] = -st;       T[0][2] = 0.0f;   T[0][3] = dh->a;
    T[1][0] = st * ca;   T[1][1] = ct * ca;   T[1][2] = -sa;    T[1][3] = -sa * dh->d;
    T[2][0] = st * sa;   T[2][1] = ct * sa;   T[2][2] = ca;     T[2][3] =  ca * dh->d;
    T[3][0] = 0.0f;      T[3][1] = 0.0f;      T[3][2] = 0.0f;   T[3][3] = 1.0f;
}

/* ================================================================
 * 正运动学
 * ================================================================ */

/**
 * @brief 正运动学: 6 个关节角度 → 末端位姿
 *
 * 计算过程: T_0_6 = T_0_1 · T_1_2 · ... · T_5_6
 *
 * @param[in]  joint_angles 6 个关节角度 (rad)
 * @param[out] pose         输出末端位姿 (位置 mm, 姿态 3×3 旋转矩阵)
 */
void ArmKine_FK(const Vec6 joint_angles, EndEffectorPose_t *pose)
{
    DH_Param_t dh[ARM_DOF];
    memcpy(dh, arm_dh, sizeof(DH_Param_t) * ARM_DOF);
    for (int i = 0; i < ARM_DOF; ++i) dh[i].theta = joint_angles[i];

    Mat4 T_total;
    RobotMath_Mat4Identity(T_total);
    for (int i = 0; i < ARM_DOF; ++i) {
        Mat4 T_joint;
        ArmKine_DHTransform(&dh[i], T_joint);
        RobotMath_Mat4Mul(T_total, T_joint, T_total);
    }

    pose->position[0] = T_total[0][3];
    pose->position[1] = T_total[1][3];
    pose->position[2] = T_total[2][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            pose->orientation[i][j] = T_total[i][j];
}

/**
 * @brief 正运动学 (带中间变换): 同 FK, 并返回所有中间帧
 *
 * 输出:
 *   transforms[0] = T_0_1, transforms[1] = T_0_2, ..., transforms[5] = T_0_6
 *
 * 用途: 雅可比矩阵计算 (需要 z_i, p_i)、重力补偿 (需要各连杆坐标系) 等
 *
 * @param[in]  joint_angles 6 个关节角度 (rad)
 * @param[out] pose         输出末端位姿
 * @param[out] transforms   输出 6 个中间齐次变换矩阵, 调用方分配空间
 */
void ArmKine_FKWithTransforms(const Vec6 joint_angles, EndEffectorPose_t *pose,
                               Mat4 transforms[ARM_DOF])
{
    DH_Param_t dh[ARM_DOF];
    memcpy(dh, arm_dh, sizeof(DH_Param_t) * ARM_DOF);

    Mat4 T_current;
    RobotMath_Mat4Identity(T_current);
    for (int i = 0; i < ARM_DOF; ++i) {
        dh[i].theta = joint_angles[i];
        Mat4 T_joint;
        ArmKine_DHTransform(&dh[i], T_joint);
        RobotMath_Mat4Mul(T_current, T_joint, T_current);
        memcpy(transforms[i], T_current, sizeof(Mat4));  // T_0_(i+1)
    }

    pose->position[0] = T_current[0][3];
    pose->position[1] = T_current[1][3];
    pose->position[2] = T_current[2][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            pose->orientation[i][j] = T_current[i][j];
}

/* ================================================================
 * 位姿误差计算
 * ================================================================ */

/**
 * @brief 计算 6 维位姿误差向量
 *
 * 位置误差 (mm): e_pos = p_target − p_current
 *
 * 姿态误差 (rad): 采用轴列向量叉积法
 * @code
 *   e_ori = 0.5 · (n_c × n_t + s_c × s_t + a_c × a_t)
 * @endcode
 * 其中 (n, s, a) 分别是末端坐标系 x, y, z 轴在基座系下的列向量。
 * 对小角度情况, e_ori ≈ 旋转向量。
 *
 * @param[in]  current 当前末端位姿
 * @param[in]  target  目标末端位姿
 * @param[out] error   输出 6 维误差: [e_x, e_y, e_z, e_rx, e_ry, e_rz]
 */
void ArmKine_PoseError(const EndEffectorPose_t *current, const EndEffectorPose_t *target, Vec6 error)
{
    // 位置误差
    error[0] = target->position[0] - current->position[0];
    error[1] = target->position[1] - current->position[1];
    error[2] = target->position[2] - current->position[2];

    // 提取 current 和目标姿态的列向量 (n=x轴, s=y轴, a=z轴)
    Vec3 n_c, s_c, a_c, n_t, s_t, a_t;
    for (int i = 0; i < 3; ++i) {
        n_c[i] = current->orientation[i][0]; s_c[i] = current->orientation[i][1]; a_c[i] = current->orientation[i][2];
        n_t[i] = target->orientation[i][0];  s_t[i] = target->orientation[i][1];  a_t[i] = target->orientation[i][2];
    }

    Vec3 cross_n, cross_s, cross_a;
    RobotMath_Vec3Cross(n_c, n_t, cross_n);
    RobotMath_Vec3Cross(s_c, s_t, cross_s);
    RobotMath_Vec3Cross(a_c, a_t, cross_a);

    for (int i = 0; i < 3; ++i)
        error[3 + i] = 0.5f * (cross_n[i] + cross_s[i] + cross_a[i]);
}

/* ================================================================
 * 几何雅可比矩阵
 * ================================================================ */

/**
 * @brief 计算 6×6 几何雅可比矩阵 (static, 仅 IK 内部使用)
 *
 * Craig 改进 DH 约定下, 旋转关节 i 的雅可比列为:
 *   J_col_i = [ z_{i+1} × (p_n − p_{i+1}) ; z_{i+1} ]
 *
 * 其中:
 *   - z_{i+1} = transforms[i] 的 Z 轴列 (第 3 列): 关节 i+1 的旋转轴在基座系下的方向
 *   - p_{i+1} = transforms[i] 的平移列 (第 4 列): 关节 i+1 坐标系原点在基座系下的位置
 *   - p_n     = 末端位置
 *
 * 特例: i=0 (关节1) 时 z_1 来自 transforms[0] (=T_{0,1}) 的 Z 轴
 *
 * @param[in]  transforms FKWithTransforms 输出的 6 个中间变换 T_0_1 ~ T_0_6
 * @param[in]  end_pos    末端位置 T_0_6 的平移分量
 * @param[out] J          输出 6×6 雅可比矩阵
 */
static void compute_jacobian(const Mat4 transforms[ARM_DOF], const Vec3 end_pos, float J[6][6])
{
    Vec3 p_n = {end_pos[0], end_pos[1], end_pos[2]};

    for (int i = 0; i < ARM_DOF; ++i) {
        Vec3 z_i, p_i;

        if (i == 0) {
            // 关节 1: 基底关节, z₁ = transforms[0] 的 Z 轴, p₁ = transforms[0] 的平移
            // 在 α₀=0 的情况下 transforms[0]=I, z=(0,0,1), p=(0,0,0)
            z_i[0] = 0.0f; z_i[1] = 0.0f; z_i[2] = 1.0f;
            p_i[0] = 0.0f; p_i[1] = 0.0f; p_i[2] = 0.0f;
        } else {
            // 关节 i+1: 从 transforms[i] (= T_{0,i+1}) 提取
            z_i[0] = transforms[i][0][2]; z_i[1] = transforms[i][1][2]; z_i[2] = transforms[i][2][2];
            p_i[0] = transforms[i][0][3]; p_i[1] = transforms[i][1][3]; p_i[2] = transforms[i][2][3];
        }

        Vec3 diff = {p_n[0]-p_i[0], p_n[1]-p_i[1], p_n[2]-p_i[2]};
        Vec3 cross;
        RobotMath_Vec3Cross(z_i, diff, cross);
        J[0][i] = cross[0]; J[1][i] = cross[1]; J[2][i] = cross[2];  // 线速度分量
        J[3][i] = z_i[0];   J[4][i] = z_i[1];   J[5][i] = z_i[2];    // 角速度分量
    }
}

/* ================================================================
 * DLS 逆运动学
 * ================================================================ */

/**
 * @brief DLS 逆运动学: 末端目标位姿 → 关节角度
 *
 * 迭代流程:
 *   1. FK: 计算当前 θ 的末端位姿
 *   2. PoseError: 得到 6 维误差 e
 *   3. 判断收敛: ||e_pos|| < threshold_pos && ||e_ori|| < threshold_ori
 *   4. Jacobian: 计算几何雅可比 J
 *   5. 构建 DLS 正规方程: (JᵀJ + λ²I) · Δθ = Jᵀe
 *   6. 求解 6×6 线性系统 → Δθ
 *   7. 更新: θ ← θ + α · Δθ
 *   8. 限位钳制, 继续下一轮
 *
 * 变阻尼策略:
 *   - 若 SolveLinear6x6 返回奇异 (矩阵秩亏), λ ← λ×2 重新求解
 *   - λ 上限 100, 超过则放弃本迭代并返回 -1
 *
 * @param[in]  target        目标末端位姿 (位置 mm, 姿态 3×3 旋转矩阵)
 * @param[in]  initial_guess 初始关节角度猜测 (rad), 通常使用当前关节角度
 * @param[out] result        输出求解得到的关节角度 (rad)
 * @return 迭代次数 (≥0 成功); -1 未收敛 (DH 未初始化 / 奇异不可解 / 达最大迭代)
 */
int ArmKine_IK(const EndEffectorPose_t *target, const Vec6 initial_guess, Vec6 result)
{
    if (!dh_initialized) {
        LOGERROR("[arm_ik] DH parameters not initialized");
        return -1;
    }

    Vec6 theta;
    memcpy(theta, initial_guess, sizeof(Vec6));
    ik_iterations = 0;
    ik_final_error = 1e6f;
    float local_lambda = ik_config.lambda;  // 局部阻尼, 不污染全局配置

    for (int iter = 0; iter < ik_config.max_iterations; ++iter) {
        ik_iterations = iter + 1;

        // 1. FK + 中间变换
        Mat4 transforms[ARM_DOF];
        EndEffectorPose_t current;
        ArmKine_FKWithTransforms(theta, &current, transforms);

        // 2. 位姿误差
        Vec6 error;
        ArmKine_PoseError(&current, target, error);

        // 3. 收敛判断
        float pos_error = 0.0f, ori_error = 0.0f;
        for (int i = 0; i < 3; ++i) pos_error += error[i] * error[i];
        for (int i = 3; i < 6; ++i) ori_error += error[i] * error[i];
        ik_final_error = pos_error + ori_error;

        if (pos_error < ik_config.converge_threshold * ik_config.converge_threshold &&
            ori_error < ik_config.orient_threshold * ik_config.orient_threshold) {
            memcpy(result, theta, sizeof(Vec6));
            return iter + 1;
        }

        // 4. 几何雅可比
        Vec3 end_pos = {transforms[ARM_DOF-1][0][3],
                        transforms[ARM_DOF-1][1][3],
                        transforms[ARM_DOF-1][2][3]};
        float J[6][6];
        memset(J, 0, sizeof(J));
        compute_jacobian(transforms, end_pos, J);

        // 5. 构建 DLS: (JᵀJ + λ²I)
        float JTJ[6][6];
        memset(JTJ, 0, sizeof(JTJ));
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    JTJ[i][j] += J[k][i] * J[k][j];  // JᵀJ

        float lambda_sq = local_lambda * local_lambda;
        for (int i = 0; i < 6; ++i) JTJ[i][i] += lambda_sq;

        // Jᵀ·e
        Vec6 JTe;
        memset(JTe, 0, sizeof(Vec6));
        for (int i = 0; i < 6; ++i)
            for (int k = 0; k < 6; ++k)
                JTe[i] += J[k][i] * error[k];

        // 6. 求解 (JᵀJ+λ²I)·Δθ = Jᵀe
        Vec6 dtheta;
        if (RobotMath_SolveLinear6x6(JTJ, JTe, dtheta) != 0) {
            // 奇异: 增大阻尼重试
            LOGWARNING("[arm_ik] singular at iter %d, increasing lambda", iter);
            local_lambda *= 2.0f;
            if (local_lambda > 100.0f) {
                LOGERROR("[arm_ik] too singular, giving up at iter %d", iter);
                return -1;
            }
            continue;
        }

        // 7. 更新 θ ← θ + α·Δθ
        for (int i = 0; i < ARM_DOF; ++i) theta[i] += ik_config.alpha * dtheta[i];

        // 8. 限位钳制
        ArmKine_ClampJointLimits(theta);
    }

    // 最大迭代次数
    LOGWARNING("[arm_ik] max iterations (%d) reached, final error: %f", ik_config.max_iterations, ik_final_error);
    memcpy(result, theta, sizeof(Vec6));
    return -1;
}

/* ================================================================
 * 关节限位
 * ================================================================ */

/**
 * @brief 检查所有 6 个关节角度是否在软限位内
 * @param[in] angles 关节角度 (rad)
 * @return 0=全部在限位内; -1=有关节超限
 */
int ArmKine_CheckJointLimits(const Vec6 angles)
{
    for (int i = 0; i < ARM_DOF; ++i)
        if (angles[i] < joint_min[i] || angles[i] > joint_max[i]) return -1;
    return 0;
}

/**
 * @brief 将所有 6 个关节角度钳制到软限位内
 * @param[in,out] angles 关节角度 (rad), 超限分量被就地修改
 */
void ArmKine_ClampJointLimits(Vec6 angles)
{
    for (int i = 0; i < ARM_DOF; ++i) {
        if (angles[i] < joint_min[i]) angles[i] = joint_min[i];
        if (angles[i] > joint_max[i]) angles[i] = joint_max[i];
    }
}

/* ================================================================
 * RPY 欧拉角 ↔ 旋转矩阵 (兼容转发, 实际运算在 robot_math_utils)
 * ================================================================ */

/** @brief ZYX 欧拉角 → 旋转矩阵, 转发至 RobotMath_RPYToRot */
void ArmKine_RPYToRotation(const Vec3 rpy, float R[3][3]) { RobotMath_RPYToRot(rpy, R); }

/** @brief 旋转矩阵 → ZYX 欧拉角, 转发至 RobotMath_RotToRPY */
void ArmKine_RotationToRPY(const float R[3][3], Vec3 rpy) { RobotMath_RotToRPY(R, rpy); }

/* ================================================================
 * 电机 ↔ 运动学坐标转换
 * ================================================================ */

/**
 * @brief 电机反馈值 → 关节角度 (减去零点偏置)
 *
 * 输入:
 *   dm_positions[3]: J1-J3 DM 电机 position 反馈 (rad)
 *   j4_angle_rad:    J4 GM6020 total_angle 转为 rad 后的值
 *   wrist_joints[2]: J5-J6 差动解算后的虚拟关节角度 (rad)
 *
 * 输出:
 *   joint_angles[6]: 6 个实际关节角 = 电机读数 − home_offset
 *
 * @param[in]  dm_positions J1-J3 电机反馈角度 (rad)
 * @param[in]  j4_angle_rad J4 电机反馈角度 (rad, 已从 deg 转换)
 * @param[in]  wrist_joints J5-J6 差动解算后的虚拟关节角度 (rad)
 * @param[out] joint_angles 输出 6 个实际关节角度 (rad)
 */
void ArmKine_GetJointAnglesFromActuators(const float dm_positions[3], float j4_angle_rad,
                                         const float wrist_joints[2], Vec6 joint_angles)
{
    joint_angles[0] = dm_positions[0]   - joint_home_offset[0];
    joint_angles[1] = dm_positions[1]   - joint_home_offset[1];
    joint_angles[2] = dm_positions[2]   - joint_home_offset[2];
    joint_angles[3] = j4_angle_rad      - joint_home_offset[3];
    joint_angles[4] = wrist_joints[0]   - joint_home_offset[4];
    joint_angles[5] = wrist_joints[1]   - joint_home_offset[5];
}

/**
 * @brief 关节角度 → 各电机参考值 (加上零点偏置)
 *
 * 逆过程: 实际关节角 → 电机期望值 = 关节角 + home_offset
 *
 * @param[in]  joint_angles 6 个关节角度 (rad)
 * @param[out] dm_refs      J1-J3 DM 电机参考角度 (rad)
 * @param[out] j4_ref_rad   J4 GM6020 参考角度 (rad, 调用方再转 deg)
 * @param[out] wrist_refs   J5-J6 差动虚拟关节参考角度 (rad, 调用方再经 diff_wrist→M2006)
 */
void ArmKine_JointToActuatorRefs(const Vec6 joint_angles, float dm_refs[3],
                                  float *j4_ref_rad, float wrist_refs[2])
{
    dm_refs[0]   = joint_angles[0] + joint_home_offset[0];
    dm_refs[1]   = joint_angles[1] + joint_home_offset[1];
    dm_refs[2]   = joint_angles[2] + joint_home_offset[2];
    *j4_ref_rad  = joint_angles[3] + joint_home_offset[3];
    wrist_refs[0] = joint_angles[4] + joint_home_offset[4];
    wrist_refs[1] = joint_angles[5] + joint_home_offset[5];
}

/* ================================================================
 * 配置接口
 * ================================================================ */

/** @brief 直接覆盖 DH 参数表 (调试/标定用) */
void ArmKine_SetDHParams(const DH_Param_t dh[ARM_DOF])
{
    memcpy(arm_dh, dh, sizeof(DH_Param_t) * ARM_DOF);
    dh_initialized = 1;
}

/** @brief 获取当前 DH 参数表只读指针 */
const DH_Param_t *ArmKine_GetDHParams(void) { return arm_dh; }

/** @brief 运行时更新 IK 求解器配置 */
void ArmKine_SetIKConfig(const IK_Config_t *config) { memcpy(&ik_config, config, sizeof(IK_Config_t)); }

/** @brief 重置 IK 内部状态 (迭代计数和最终误差清零) */
void ArmKine_IKReset(void) { ik_iterations = 0; ik_final_error = 0.0f; }

/**
 * @brief 初始化运动学引擎 (注入完整配置)
 *
 * 将应用层从 robot_def.h 宏组装的配置一次性加载到模块内部。
 * 必须在任何 FK/IK 调用之前执行。
 *
 * @param[in] config 运动学配置 (DH参数表、关节限位、零偏、IK参数)
 * @note 若 config==NULL 则记录错误并返回, DH 保持未初始化状态
 */
void ArmKine_Init(const ArmKine_Config_t *config)
{
    if (config == NULL) { LOGERROR("[arm_kine] init config is null"); return; }
    memcpy(arm_dh,            config->dh,                sizeof(arm_dh));
    memcpy(joint_min,         config->joint_min,         sizeof(joint_min));
    memcpy(joint_max,         config->joint_max,         sizeof(joint_max));
    memcpy(joint_home_offset, config->joint_home_offset, sizeof(joint_home_offset));
    memcpy(&ik_config,        &config->ik,               sizeof(IK_Config_t));
    dh_initialized = 1;
}
