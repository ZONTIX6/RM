/**
 * @file arm_kinematics.h
 * @brief 6自由度机械臂运动学模块: DH参数定义、正运动学(FK)、逆运动学(IK)接口
 *
 * DH约定:
 *   - 使用改进DH(Craig)约定: T = Rot_x(α_{i-1}) · Trans_x(a_{i-1}) · Rot_z(θ_i) · Trans_z(d_i)
 *   - transforms[k] = T_{0,k+1} (从基座到连杆 k+1 的齐次变换)
 *
 * 关节配置:
 *   - J1-J3: DM 电机 (position 反馈, 单位 rad)
 *   - J4:    DJI GM6020 (total_angle 反馈, 单位 deg → 内部转 rad)
 *   - J5-J6: 两个 M2006 经差动机构合成的虚拟关节 (通过 diff_wrist 解算)
 *
 * IK算法:
 *   - 基于几何雅可比的 DLS (阻尼最小二乘) 迭代求解
 *   - 变阻尼策略: 遇到奇异时 λ 指数翻倍
 *   - 每步迭代后自动钳制到关节限位
 *
 * @note 运动学模块内部统一使用 rad 作为角度单位
 * @note 位置/长度统一使用 mm 作为单位 (与 DH 参数约定一致)
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include "robot_math_utils.h"

/* ================================================================
 * 数据结构
 * ================================================================ */

/**
 * @brief 单个关节的 Craig 改进 DH 参数
 *
 * 变换矩阵: T_{i-1,i} = Rx(α) · Tx(a) · Rz(θ) · Tz(d)
 * 对旋转关节: θ 是变量, a/α/d 是常量
 */
typedef struct {
    float a;      /**< 连杆长度 (mm) */
    float alpha;  /**< 连杆扭转角 (rad) */
    float d;      /**< 连杆偏距 (mm) */
    float theta;  /**< 关节角 (rad, 变量) */
} DH_Param_t;

/**
 * @brief 末端执行器位姿
 *
 * position: 末端原点在基座坐标系中的坐标 (mm)
 * orientation: 3×3 旋转矩阵, 列向量分别为末端坐标系 x/y/z 轴在基座系下的方向
 */
typedef struct {
    Vec3  position;            /**< 末端位置 (mm) */
    float orientation[3][3];  /**< 末端姿态旋转矩阵, 列主序语义 (每列为轴方向) */
} EndEffectorPose_t;

/**
 * @brief DLS 逆运动学求解器配置
 *
 * 阻尼参数 λ 决定 DLS 行为:
 *   - λ 太小: 接近标准 Gauss-Newton, 收敛快但奇异性敏感
 *   - λ 太大: 接近最速下降, 鲁棒但收敛慢
 *   - 典型值: 0.1 ~ 1.0
 *
 * 步长 α: 控制每次迭代的更新幅度, 典型值 0.5 ~ 1.0
 */
typedef struct {
    float converge_threshold; /**< 位置收敛阈值 (mm), 建议 0.5~2.0 */
    float orient_threshold;   /**< 姿态收敛阈值 (rad), 建议 0.001~0.01 (~0.06°~0.6°) */
    float lambda;             /**< DLS 初始阻尼系数, 典型 0.5 */
    float alpha;              /**< 步长缩放因子, 典型 0.8; 1.0=全步长, 越小越保守 */
    int   max_iterations;     /**< 最大迭代次数, 典型 50~200 */
} IK_Config_t;

/**
 * @brief 运动学模块完整配置
 *
 * 由应用层从 robot_def.h 宏组装, 在 RoboticArmInit() 中调用 ArmKine_Init() 注入。
 * 所有角度量纲统一为 rad (J5/J6 的 deg→rad 由应用层在传入前转换)。
 */
typedef struct {
    DH_Param_t dh[ARM_DOF];           /**< 6 行改进 DH 参数表 */
    float      joint_min[ARM_DOF];     /**< 各关节软下限 (rad) */
    float      joint_max[ARM_DOF];     /**< 各关节软上限 (rad) */
    float      joint_home_offset[ARM_DOF]; /**< 各关节零点偏置 (rad), 实际关节角 = 电机读数 - home_offset */
    IK_Config_t ik;                    /**< DLS-IK 求解器配置 */
} ArmKine_Config_t;

/* ================================================================
 * 配置与初始化
 * ================================================================ */

void ArmKine_Init(const ArmKine_Config_t *config);
void ArmKine_SetDHParams(const DH_Param_t dh[ARM_DOF]);
const DH_Param_t *ArmKine_GetDHParams(void);
void ArmKine_SetIKConfig(const IK_Config_t *config);
void ArmKine_IKReset(void);

/* ================================================================
 * DH 变换与正运动学 (FK)
 * ================================================================ */

void ArmKine_DHTransform(const DH_Param_t *dh, Mat4 T);
void ArmKine_FK(const Vec6 joint_angles, EndEffectorPose_t *pose);
void ArmKine_FKWithTransforms(const Vec6 joint_angles, EndEffectorPose_t *pose,
                               Mat4 transforms[ARM_DOF]);

/* ================================================================
 * 逆运动学 (IK) 与位姿误差
 * ================================================================ */

int  ArmKine_IK(const EndEffectorPose_t *target, const Vec6 initial_guess, Vec6 result);
void ArmKine_PoseError(const EndEffectorPose_t *current, const EndEffectorPose_t *target, Vec6 error);

/* ================================================================
 * 关节限位
 * ================================================================ */

int  ArmKine_CheckJointLimits(const Vec6 angles);
void ArmKine_ClampJointLimits(Vec6 angles);

/* ================================================================
 * 电机 ↔ 运动学坐标转换
 * ================================================================ */

void ArmKine_GetJointAnglesFromActuators(const float dm_positions[3], float j4_angle_rad,
                                         const float wrist_joints[2], Vec6 joint_angles);
void ArmKine_JointToActuatorRefs(const Vec6 joint_angles, float dm_refs[3],
                                  float *j4_ref_rad, float wrist_refs[2]);

/* ================================================================
 * RPY 欧拉角 ↔ 旋转矩阵 (已迁移至 robot_math_utils, 此处保留兼容转发)
 * ================================================================ */

void ArmKine_RPYToRotation(const Vec3 rpy, float R[3][3]);
void ArmKine_RotationToRPY(const float R[3][3], Vec3 rpy);

#endif /* ARM_KINEMATICS_H */
