/**
 * @file robot_math_utils.h
 * @brief 机器人学通用数学类型与函数声明
 *
 * 涵盖范围:
 *   - 基础类型: 矩阵(Mat4/Mat3)、向量(Vec3/Vec6)、四元数(Quat)、轴角(AxisAngle)、
 *     多项式(CubicPoly/QuinticPoly)、低通滤波器(LowPassFilter1P)
 *   - 4×4 齐次变换: 乘法、恒等、求逆、构建、分解、点/向量变换
 *   - 3×3 旋转矩阵: 乘法、恒等、转置、行列式、求逆(通用)、矩阵-向量乘法
 *   - 3 维向量: 加减、标量乘、叉积、点积、模长、归一化、Hat、Lerp、距离、
 *     投影、正交分量、限幅
 *   - 角度工具: 规范化到[-π,π]/[0,2π)、最短差、平滑插值
 *   - 四元数: 归一化、Hamilton乘、共轭、求逆、旋转向量、双向矩阵互换、Slerp
 *   - 旋转表示互转: RPY↔矩阵、轴角↔矩阵↔四元数、角速度积分
 *   - 常用控制工具: 三次多项式轨迹规划(系数求解+取值+导数)、一阶低通滤波器
 *   - 线性求解: 6×6高斯列主元消元
 *
 * @note 所有函数均使用单精度浮点(float)，适配嵌入式ARM Cortex-M4 FPU
 * @note 命名统一前缀 RobotMath_，公共API采用清晰的动宾结构
 * @note 输出参数遵循"先输入后输出"的顺序惯例
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#ifndef ROBOT_MATH_UTILS_H
#define ROBOT_MATH_UTILS_H

#include <stdint.h>

/* ================================================================
 * 常量定义
 * ================================================================ */

/** @brief 机械臂自由度 (保留, 供旧代码兼容; 通用工具函数不依赖此数值) */
#define ARM_DOF 6

/* ================================================================
 * 基础数学类型
 * ================================================================ */

/** @brief 4×4 齐次变换矩阵, 行优先存储: M[行][列] */
typedef float Mat4[4][4];

/** @brief 3×3 矩阵, 行优先存储: M[行][列] */
typedef float Mat3[3][3];

/** @brief 3 维向量 (位置/速度/角速度等) */
typedef float Vec3[3];

/** @brief 6 维向量 (关节角度/速度/力矩/位姿误差等) */
typedef float Vec6[6];

/** @brief 单位四元数, 存储格式 [w, x, y, z], 满足 w²+x²+y²+z²=1 */
typedef float Quat[4];

/* ================================================================
 * 复合数据结构
 * ================================================================ */

/**
 * @brief 轴角旋转表示
 *
 * 将旋转表示为绕单位向量 axis 旋转 angle 弧度。
 * 等价关系: R = exp( angle · hat(axis) )  (Rodrigues 公式)
 */
typedef struct {
    float angle;   /**< 旋转角度 (rad) */
    Vec3  axis;    /**< 旋转轴单位向量, 使用前建议调用 Vec3Normalize */
} AxisAngle_t;

/**
 * @brief 三次多项式系数
 *
 * 多项式: p(t) = a0 + a1·t + a2·t² + a3·t³
 * 通常由 CubicInterpPlan() 根据起止点位置和速度边界条件计算得到
 */
typedef struct {
    float a0;      /**< 常数项 */
    float a1;      /**< 一次项系数 */
    float a2;      /**< 二次项系数 */
    float a3;      /**< 三次项系数 */
} CubicPoly_t;

/**
 * @brief 五次多项式系数
 *
 * 多项式: p(t) = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
 * 用于需要 jerk 连续 (C⁴光滑) 的高阶平滑轨迹
 */
typedef struct {
    float a0;      /**< 常数项 */
    float a1;      /**< 一次项系数 */
    float a2;      /**< 二次项系数 */
    float a3;      /**< 三次项系数 */
    float a4;      /**< 四次项系数 */
    float a5;      /**< 五次项系数 */
} QuinticPoly_t;

/**
 * @brief 一阶低通滤波器状态
 *
 * 差分方程: y[n] = alpha·x[n] + (1-alpha)·y[n-1]
 * 其中 alpha = dt / (RC + dt), RC = 1/(2π·f_cutoff)
 */
typedef struct {
    float   alpha;       /**< 滤波系数, 范围 [0,1]; 越接近 1 则响应越快、平滑越少 */
    float   y_prev;      /**< 上一拍输出值 y[n-1] */
    uint8_t initialized; /**< 初始化标志: 0=需要首拍置位, 1=正常运行 */
} LowPassFilter1P_t;

/* ================================================================
 * 4×4 矩阵 / 齐次变换
 * ================================================================ */

void RobotMath_Mat4Mul(const Mat4 A, const Mat4 B, Mat4 C);
void RobotMath_Mat4Identity(Mat4 M);
void RobotMath_RigidInv(const Mat4 T, Mat4 T_inv);
void RobotMath_MakeTransform(const Quat q, const Vec3 pos, Mat4 T);
void RobotMath_MakeTransformFromRot(const Mat3 R, const Vec3 pos, Mat4 T);

/** @brief 从4×4齐次变换矩阵提取平移分量 (static inline) */
static inline void RobotMath_GetPos(const Mat4 T, Vec3 pos)
{
    pos[0] = T[0][3];
    pos[1] = T[1][3];
    pos[2] = T[2][3];
}

/** @brief 从4×4齐次变换矩阵提取3×3旋转分量 (static inline) */
static inline void RobotMath_GetRot(const Mat4 T, Mat3 R)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = T[i][j];
}

/** @brief 齐次变换链式乘法的语义别名, 等价于 Mat4Mul(A, B, C) */
#define RobotMath_TransformChain(A, B, C) RobotMath_Mat4Mul((A), (B), (C))

/* ================================================================
 * 3×3 矩阵
 * ================================================================ */

void  RobotMath_Mat3Mul(const Mat3 A, const Mat3 B, Mat3 C);
void  RobotMath_Mat3Vec3Mul(const Mat3 A, const Vec3 x, Vec3 y);
void  RobotMath_Mat3Identity(Mat3 M);
void  RobotMath_Mat3Transpose(const Mat3 M, Mat3 M_t);
float RobotMath_Mat3Det(const Mat3 M);
int   RobotMath_Mat3Inv(const Mat3 M, Mat3 M_inv);

/* ================================================================
 * 3 维向量
 * ================================================================ */

void  RobotMath_Vec3Add(const Vec3 a, const Vec3 b, Vec3 c);
void  RobotMath_Vec3Sub(const Vec3 a, const Vec3 b, Vec3 c);
void  RobotMath_Vec3Scale(const Vec3 a, float k, Vec3 c);
void  RobotMath_Vec3Cross(const Vec3 a, const Vec3 b, Vec3 c);
float RobotMath_Vec3Dot(const Vec3 a, const Vec3 b);
float RobotMath_Vec3Norm(const Vec3 v);
void  RobotMath_Vec3Normalize(Vec3 v);
void  RobotMath_Vec3Hat(const Vec3 v, Mat3 hat);
void  RobotMath_Vec3Lerp(const Vec3 a, const Vec3 b, float t, Vec3 c);
float RobotMath_Vec3Dist(const Vec3 a, const Vec3 b);
void  RobotMath_Vec3Proj(const Vec3 v, const Vec3 unit_axis, Vec3 proj);
void  RobotMath_Vec3Reject(const Vec3 v, const Vec3 unit_axis, Vec3 rej);
void  RobotMath_Vec3ClampNorm(Vec3 v, float max_norm);

/* ================================================================
 * 角度运算
 * ================================================================ */

float RobotMath_NormAnglePI(float angle_rad);
float RobotMath_NormAngle2PI(float angle_rad);
float RobotMath_AngleDiff(float from_rad, float to_rad);
float RobotMath_AngleLerp(float from_rad, float to_rad, float t);

/* ================================================================
 * 空间变换
 * ================================================================ */

void RobotMath_TransformPoint(const Mat4 T, const Vec3 p_in, Vec3 p_out);
void RobotMath_TransformVec3(const Mat4 T, const Vec3 v_in, Vec3 v_out);
void RobotMath_Mat4ToQuat(const Mat4 T, Quat q);

/* ================================================================
 * 四元数
 * ================================================================ */

void  RobotMath_QuatNormalize(Quat q);
void  RobotMath_QuatMul(const Quat q1, const Quat q2, Quat q_out);
void  RobotMath_QuatConj(const Quat q, Quat q_conj);
void  RobotMath_QuatInv(const Quat q, Quat q_inv);
void  RobotMath_QuatRotateVec3(const Quat q, const Vec3 v, Vec3 v_out);
void  RobotMath_RotToQuat(const Mat3 R, Quat q);
void  RobotMath_QuatToRot(const Quat q, Mat3 R);
void  RobotMath_QuatSlerp(const Quat q0, const Quat q1, float s, Quat q_out);

/* ================================================================
 * 旋转矩阵高级操作
 * ================================================================ */

float RobotMath_RotAngleBetween(const Mat3 R0, const Mat3 R1);
void  RobotMath_RotSlerp(const Mat3 R0, const Mat3 R1, float s, Mat3 R_out);
void  RobotMath_RPYToRot(const Vec3 rpy, Mat3 R);
void  RobotMath_RotToRPY(const Mat3 R, Vec3 rpy);

/* ================================================================
 * 轴角法
 * ================================================================ */

void RobotMath_AxisAngleToRot(const AxisAngle_t *aa, Mat3 R);
void RobotMath_RotToAxisAngle(const Mat3 R, AxisAngle_t *aa);
void RobotMath_AxisAngleToQuat(const AxisAngle_t *aa, Quat q);
void RobotMath_QuatToAxisAngle(const Quat q, AxisAngle_t *aa);
void RobotMath_RotIntegrateOmega(const Mat3 R, const Vec3 omega, float dt, Mat3 R_new);

/* ================================================================
 * 多项式插值
 * ================================================================ */

void  RobotMath_CubicInterpPlan(float x0, float xT, float v0, float vT,
                                float T, CubicPoly_t *poly);
float RobotMath_CubicInterpEval(const CubicPoly_t *poly, float t);
float RobotMath_CubicInterpEvalDeriv(const CubicPoly_t *poly, float t);

/* ================================================================
 * 低通滤波器
 * ================================================================ */

void  RobotMath_LowPassFilterInit(LowPassFilter1P_t *filt, float alpha);
void  RobotMath_LowPassFilterInitByFreq(LowPassFilter1P_t *filt,
                                         float cutoff_hz, float dt);
float RobotMath_LowPassFilterStep(LowPassFilter1P_t *filt, float x);
void  RobotMath_LowPassFilterReset(LowPassFilter1P_t *filt);

/* ================================================================
 * 线性求解
 * ================================================================ */

int RobotMath_SolveLinear6x6(const float A[6][6], const Vec6 b, Vec6 x);

#endif /* ROBOT_MATH_UTILS_H */
