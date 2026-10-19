/**
 * @file robot_math_utils.c
 * @brief 机器人学通用数学函数实现
 *
 * 包含:
 *   - 4×4 / 3×3 矩阵运算 (乘法、求逆、行列式、分解)
 *   - 3 维向量运算 (基础运算、空间变换、几何工具)
 *   - 角度工具 (规范化、最短差、插值)
 *   - 四元数运算 (Hamilton乘积、Slerp、与旋转矩阵/轴角互转)
 *   - 多项式插值与低通滤波
 *   - 6×6 线性方程组求解
 *
 * 数值约定:
 *   - 所有角度与三角函数使用弧度 (rad)
 *   - 齐次变换矩阵为刚性变换 (无缩放/剪切), 逆矩阵用解析公式
 *   - 四元数遵循 {w, x, y, z} 存储顺序
 *   - RPY 欧拉角为 ZYX 固定角, 即 R = Rz(yaw)·Ry(pitch)·Rx(roll)
 *
 * @note 全程单精度 float, 浮点容错阈值见 ROBOT_MATH_EPSILON
 * @note 参考 "上海交通大学交龙战队数学运算库"
 *       参考 "福建理工大学苍侠战队数学运算库"
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#include "robot_math_utils.h"
#include <math.h>
#include <string.h>

/* ================================================================
 * 内部常量
 * ================================================================ */

/** @brief π (单精度) */
#define ROBOT_MATH_PI        3.14159265358979323846f

/** @brief 2π (单精度) */
#define ROBOT_MATH_TWO_PI    6.28318530717958647692f

/** @brief 浮点零判定阈值, 小于此值的绝对值视为零 */
#define ROBOT_MATH_EPSILON   1e-10f

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/**
 * @brief 浮点值限幅, 将 value 强制约束在 [min_value, max_value] 区间
 * @param value     原始输入值
 * @param min_value 下限
 * @param max_value 上限
 * @return 钳制后的值
 * @note static 函数, 仅本文件可见
 */
static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/* ================================================================
 * 4×4 矩阵 / 齐次变换
 * ================================================================ */

/**
 * @brief 4×4 矩阵乘法: C = A × B
 * @param[in]  A 左乘矩阵 (行优先)
 * @param[in]  B 右乘矩阵 (行优先)
 * @param[out] C 乘积结果矩阵
 * @note 内部使用临时矩阵, 支持 C 与 A 或 B 为同一块内存 (原地操作安全)
 * @note 矩阵乘法不满足交换律, 注意乘序
 */
void RobotMath_Mat4Mul(const Mat4 A, const Mat4 B, Mat4 C)
{
    Mat4 tmp;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
        {
            tmp[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k)
                tmp[i][j] += A[i][k] * B[k][j];
        }
    memcpy(C, tmp, sizeof(Mat4));
}

/**
 * @brief 初始化 4×4 单位矩阵 (齐次恒等变换)
 * @param[out] M 输出单位矩阵
 */
void RobotMath_Mat4Identity(Mat4 M)
{
    memset(M, 0, sizeof(Mat4));
    M[0][0] = 1.0f; M[1][1] = 1.0f;
    M[2][2] = 1.0f; M[3][3] = 1.0f;
}

/**
 * @brief 初始化 3×3 单位矩阵 (无旋转)
 * @param[out] M 输出单位矩阵
 */
void RobotMath_Mat3Identity(Mat3 M)
{
    memset(M, 0, sizeof(Mat3));
    M[0][0] = 1.0f; M[1][1] = 1.0f; M[2][2] = 1.0f;
}

/**
 * @brief 3×3 矩阵转置: M_t = M^T
 * @param[in]  M   输入矩阵
 * @param[out] M_t 输出转置矩阵
 * @note 内部使用临时矩阵, 支持 M 与 M_t 重叠
 */
void RobotMath_Mat3Transpose(const Mat3 M, Mat3 M_t)
{
    Mat3 tmp;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            tmp[i][j] = M[j][i];
    memcpy(M_t, tmp, sizeof(Mat3));
}

/**
 * @brief 3×3 矩阵乘法: C = A × B
 * @param[in]  A 左乘矩阵
 * @param[in]  B 右乘矩阵
 * @param[out] C 乘积结果矩阵
 * @note 内部使用临时矩阵, 原地操作安全
 */
void RobotMath_Mat3Mul(const Mat3 A, const Mat3 B, Mat3 C)
{
    Mat3 tmp;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            tmp[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
                tmp[i][j] += A[i][k] * B[k][j];
        }
    memcpy(C, tmp, sizeof(Mat3));
}

/**
 * @brief 3×3 矩阵左乘 3 维列向量: y = A · x
 * @param[in]  A 3×3 矩阵 (行优先)
 * @param[in]  x 输入 3 维向量
 * @param[out] y 输出 3 维向量
 * @note 内部使用临时向量, 支持 x 与 y 为同一内存
 */
void RobotMath_Mat3Vec3Mul(const Mat3 A, const Vec3 x, Vec3 y)
{
    Vec3 tmp;
    for (int i = 0; i < 3; ++i)
        tmp[i] = A[i][0] * x[0] + A[i][1] * x[1] + A[i][2] * x[2];
    memcpy(y, tmp, sizeof(Vec3));
}

/**
 * @brief 3×3 矩阵行列式
 * @param[in] M 输入矩阵
 * @return 行列式值
 */
float RobotMath_Mat3Det(const Mat3 M)
{
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
         - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
         + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

/**
 * @brief 3×3 通用矩阵求逆 (伴随矩阵法)
 * @param[in]  M     输入矩阵 (可为任意非奇异 3×3 矩阵)
 * @param[out] M_inv 输出逆矩阵
 * @return 0=成功; -1=矩阵奇异 (行列式绝对值 < ROBOT_MATH_EPSILON)
 * @note 不同于 RigidInv 专用的 R^T 取逆, 本函数支持缩放/剪切等任意 3×3 矩阵
 * @note 内部使用临时矩阵, M 与 M_inv 可重叠
 */
int RobotMath_Mat3Inv(const Mat3 M, Mat3 M_inv)
{
    float det = RobotMath_Mat3Det(M);
    if (fabsf(det) < ROBOT_MATH_EPSILON) return -1;

    float inv_det = 1.0f / det;
    Mat3 tmp;
    tmp[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * inv_det;
    tmp[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) * inv_det;
    tmp[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * inv_det;
    tmp[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) * inv_det;
    tmp[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * inv_det;
    tmp[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) * inv_det;
    tmp[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * inv_det;
    tmp[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) * inv_det;
    tmp[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * inv_det;

    memcpy(M_inv, tmp, sizeof(Mat3));
    return 0;
}

/* ================================================================
 * 3 维向量基础运算
 * ================================================================ */

/**
 * @brief 向量加法: c = a + b
 * @param[in]  a 加数向量
 * @param[in]  b 加数向量
 * @param[out] c 和向量
 * @note 使用临时数组确保 a/c 或 b/c 重叠时正确
 */
void RobotMath_Vec3Add(const Vec3 a, const Vec3 b, Vec3 c)
{
    Vec3 tmp = {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    memcpy(c, tmp, sizeof(Vec3));
}

/**
 * @brief 向量减法: c = a - b
 * @param[in]  a 被减向量
 * @param[in]  b 减数向量
 * @param[out] c 差向量
 */
void RobotMath_Vec3Sub(const Vec3 a, const Vec3 b, Vec3 c)
{
    Vec3 tmp = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    memcpy(c, tmp, sizeof(Vec3));
}

/**
 * @brief 向量标量乘法: c = k · a
 * @param[in]  a 输入向量
 * @param[in]  k 标量系数
 * @param[out] c 缩放后向量
 */
void RobotMath_Vec3Scale(const Vec3 a, float k, Vec3 c)
{
    Vec3 tmp = {a[0] * k, a[1] * k, a[2] * k};
    memcpy(c, tmp, sizeof(Vec3));
}

/**
 * @brief 向量叉积: c = a × b
 * @param[in]  a 左向量
 * @param[in]  b 右向量
 * @param[out] c 叉积结果 (右手定则)
 * @note 叉积为原地不安全操作 (c 不能与 a 或 b 重叠),
 *       调用方需确保输出独立
 */
void RobotMath_Vec3Cross(const Vec3 a, const Vec3 b, Vec3 c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * @brief 向量点积: a · b
 * @param[in] a 向量 a
 * @param[in] b 向量 b
 * @return 标量点积值
 */
float RobotMath_Vec3Dot(const Vec3 a, const Vec3 b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief 向量欧氏模长: ||v||
 * @param[in] v 输入向量
 * @return 标量模长
 */
float RobotMath_Vec3Norm(const Vec3 v)
{
    return sqrtf(RobotMath_Vec3Dot(v, v));
}

/**
 * @brief 向量原地归一化: v = v / ||v||
 * @param[in,out] v 输入向量, 调用后变为单位向量
 * @note 模长 < 1e-12 时输出零向量, 避免除零 NaN
 */
void RobotMath_Vec3Normalize(Vec3 v)
{
    float norm = RobotMath_Vec3Norm(v);
    if (norm < 1e-12f) { v[0] = v[1] = v[2] = 0.0f; return; }
    v[0] /= norm; v[1] /= norm; v[2] /= norm;
}

/**
 * @brief 三维向量线性插值: c = a + t·(b - a)
 * @param[in]  a 起点向量
 * @param[in]  b 终点向量
 * @param[in]  t 插值系数, 自动钳位到 [0,1]
 * @param[out] c 插值结果
 */
void RobotMath_Vec3Lerp(const Vec3 a, const Vec3 b, float t, Vec3 c)
{
    t = clampf(t, 0.0f, 1.0f);
    Vec3 tmp;
    for (int i = 0; i < 3; ++i) tmp[i] = a[i] + t * (b[i] - a[i]);
    memcpy(c, tmp, sizeof(Vec3));
}

/**
 * @brief 两点欧氏距离: ||a - b||
 * @param[in] a 第一个点
 * @param[in] b 第二个点
 * @return 标量距离
 */
float RobotMath_Vec3Dist(const Vec3 a, const Vec3 b)
{
    Vec3 diff;
    RobotMath_Vec3Sub(a, b, diff);
    return RobotMath_Vec3Norm(diff);
}

/**
 * @brief 向量投影: proj = (v · unit_axis) · unit_axis
 * @param[in]  v         输入向量
 * @param[in]  unit_axis 投影轴 (调用方保证已归一化)
 * @param[out] proj      投影分量
 */
void RobotMath_Vec3Proj(const Vec3 v, const Vec3 unit_axis, Vec3 proj)
{
    float dot = RobotMath_Vec3Dot(v, unit_axis);
    Vec3 tmp = {dot * unit_axis[0], dot * unit_axis[1], dot * unit_axis[2]};
    memcpy(proj, tmp, sizeof(Vec3));
}

/**
 * @brief 向量正交分量: rej = v - proj(v, unit_axis)
 * @param[in]  v         输入向量
 * @param[in]  unit_axis 参考轴 (调用方保证已归一化)
 * @param[out] rej       正交于 unit_axis 的剩余分量
 */
void RobotMath_Vec3Reject(const Vec3 v, const Vec3 unit_axis, Vec3 rej)
{
    Vec3 proj;
    RobotMath_Vec3Proj(v, unit_axis, proj);
    RobotMath_Vec3Sub(v, proj, rej);
}

/**
 * @brief 将向量模长裁切到上限以内, 零向量或 max_norm≤0 时输出零向量
 * @param[in,out] v        输入向量, 可能被就地缩放
 * @param[in]     max_norm 允许的最大模长
 * @note 模长 < 1e-12 时直接置零
 */
void RobotMath_Vec3ClampNorm(Vec3 v, float max_norm)
{
    float norm = RobotMath_Vec3Norm(v);
    if (max_norm <= 0.0f || norm < 1e-12f) { v[0] = v[1] = v[2] = 0.0f; return; }
    if (norm > max_norm) {
        float scale = max_norm / norm;
        v[0] *= scale; v[1] *= scale; v[2] *= scale;
    }
}

/**
 * @brief 向量反对称 Hat 矩阵: hat(v) 满足 a×b = hat(a)·b
 * @param[in]  v   输入 3 维向量
 * @param[out] hat 输出 3×3 反对称矩阵
 */
void RobotMath_Vec3Hat(const Vec3 v, Mat3 hat)
{
    hat[0][0] =  0.0f;  hat[0][1] = -v[2];  hat[0][2] =  v[1];
    hat[1][0] =  v[2];  hat[1][1] =  0.0f;  hat[1][2] = -v[0];
    hat[2][0] = -v[1];  hat[2][1] =  v[0];  hat[2][2] =  0.0f;
}

/* ================================================================
 * 角度运算
 * ================================================================ */

/**
 * @brief 将任意角度规范化到 [-π, +π]
 * @param[in] angle_rad 原始角度 (rad)
 * @return 规范化后的角度, ∈ [-π, +π]
 * @note 不依赖 fmod, 用 while 循环实现, 兼容裸机嵌入式环境
 */
float RobotMath_NormAnglePI(float angle_rad)
{
    while (angle_rad >  ROBOT_MATH_PI) angle_rad -= ROBOT_MATH_TWO_PI;
    while (angle_rad < -ROBOT_MATH_PI) angle_rad += ROBOT_MATH_TWO_PI;
    return angle_rad;
}

/**
 * @brief 将任意角度规范化到 [0, 2π)
 * @param[in] angle_rad 原始角度 (rad)
 * @return 规范化后的角度, ∈ [0, 2π)
 */
float RobotMath_NormAngle2PI(float angle_rad)
{
    while (angle_rad >= ROBOT_MATH_TWO_PI) angle_rad -= ROBOT_MATH_TWO_PI;
    while (angle_rad <  0.0f)             angle_rad += ROBOT_MATH_TWO_PI;
    return angle_rad;
}

/**
 * @brief 角度最短差: 从 from_rad 到 to_rad 的最小旋转角, 带符号
 * @param[in] from_rad 起始角度 (rad)
 * @param[in] to_rad   目标角度 (rad)
 * @return 带符号的角度差, ∈ [-π, +π]; 正值表示从 from 逆时针旋转到 to
 */
float RobotMath_AngleDiff(float from_rad, float to_rad)
{
    return RobotMath_NormAnglePI(to_rad - from_rad);
}

/**
 * @brief 角度沿最短路径线性插值
 * @param[in] from_rad 起始角度 (rad)
 * @param[in] to_rad   目标角度 (rad)
 * @param[in] t        插值系数, 自动钳位到 [0,1]
 * @return 插值后的角度 (rad)
 * @note 内部自动走最短弧, 例如 170°→-170° 的 50% 插值为 180° (经过 180°, 不经过 0°)
 */
float RobotMath_AngleLerp(float from_rad, float to_rad, float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return RobotMath_NormAnglePI(from_rad + t * RobotMath_AngleDiff(from_rad, to_rad));
}

/* ================================================================
 * 空间变换
 * ================================================================ */

/**
 * @brief 4×4 齐次变换作用于三维点: p_out = T · p_in
 *        (等价于 R·p_in + t, 其中 R = T[0:2][0:2], t = T[0:2][3])
 * @param[in]  T     齐次变换矩阵
 * @param[in]  p_in  输入三维点
 * @param[out] p_out 变换后三维点
 * @note 不支持原地 (p_in 和 p_out 不可重叠)
 */
void RobotMath_TransformPoint(const Mat4 T, const Vec3 p_in, Vec3 p_out)
{
    p_out[0] = T[0][0]*p_in[0] + T[0][1]*p_in[1] + T[0][2]*p_in[2] + T[0][3];
    p_out[1] = T[1][0]*p_in[0] + T[1][1]*p_in[1] + T[1][2]*p_in[2] + T[1][3];
    p_out[2] = T[2][0]*p_in[0] + T[2][1]*p_in[1] + T[2][2]*p_in[2] + T[2][3];
}

/**
 * @brief 4×4 齐次变换作用于三维方向向量: v_out = R · v_in
 *        (仅取旋转部分, 忽略平移分量)
 * @param[in]  T     齐次变换矩阵
 * @param[in]  v_in  输入方向向量
 * @param[out] v_out 旋转后的方向向量
 */
void RobotMath_TransformVec3(const Mat4 T, const Vec3 v_in, Vec3 v_out)
{
    Vec3 tmp;
    tmp[0] = T[0][0]*v_in[0] + T[0][1]*v_in[1] + T[0][2]*v_in[2];
    tmp[1] = T[1][0]*v_in[0] + T[1][1]*v_in[1] + T[1][2]*v_in[2];
    tmp[2] = T[2][0]*v_in[0] + T[2][1]*v_in[1] + T[2][2]*v_in[2];
    memcpy(v_out, tmp, sizeof(Vec3));
}

/**
 * @brief 4×4 刚体齐次变换矩阵求逆: T_inv = T⁻¹
 *
 * 解析公式 (利用旋转矩阵正交性 R⁻¹ = R^T):
 * @code
 *   T = [ R   t ]      T⁻¹ = [ R^T   -R^T·t ]
 *       [ 0   1 ]            [  0       1    ]
 * @endcode
 *
 * @param[in]  T     输入 4×4 刚性齐次变换 (仅旋转+平移, 无缩放/剪切)
 * @param[out] T_inv 输出逆变换
 * @note 内部使用临时矩阵, 支持 T 与 T_inv 为同一内存
 */
void RobotMath_RigidInv(const Mat4 T, Mat4 T_inv)
{
    Mat4 tmp;
    RobotMath_Mat4Identity(tmp);
    // 左上角 3×3: R^T
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            tmp[i][j] = T[j][i];
    // 平移项: -R^T · t
    for (int i = 0; i < 3; ++i) {
        tmp[i][3] = 0.0f;
        for (int j = 0; j < 3; ++j)
            tmp[i][3] -= tmp[i][j] * T[j][3];
    }
    memcpy(T_inv, tmp, sizeof(Mat4));
}

/**
 * @brief 从旋转矩阵和平移向量构建 4×4 齐次变换: T = [R | t; 0 | 1]
 * @param[in]  R   3×3 旋转矩阵
 * @param[in]  pos 平移向量 (3 维)
 * @param[out] T   输出 4×4 齐次变换矩阵
 */
void RobotMath_MakeTransformFromRot(const Mat3 R, const Vec3 pos, Mat4 T)
{
    Mat4 tmp;
    RobotMath_Mat4Identity(tmp);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) tmp[i][j] = R[i][j];
        tmp[i][3] = pos[i];
    }
    memcpy(T, tmp, sizeof(Mat4));
}

/* ================================================================
 * 四元数
 * ================================================================ */

/**
 * @brief 四元数原地归一化: q = q / ||q||
 * @param[in,out] q 四元数 {w,x,y,z}, 调用后模长为 1
 * @note 模长 < 1e-12 时重置为恒等四元数 (1,0,0,0)
 */
void RobotMath_QuatNormalize(Quat q)
{
    float norm_sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (norm_sq < 1e-12f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    float norm = sqrtf(norm_sq);
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}

/**
 * @brief 四元数 Hamilton 乘法: q_out = q1 ⊗ q2
 *
 * 公式:
 * @code
 *   w = w1*w2 - x1*x2 - y1*y2 - z1*z2
 *   x = w1*x2 + x1*w2 + y1*z2 - z1*y2
 *   y = w1*y2 - x1*z2 + y1*w2 + z1*x2
 *   z = w1*z2 + x1*y2 - y1*x2 + z1*w2
 * @endcode
 *
 * @param[in]  q1    左四元数 (先施加的旋转)
 * @param[in]  q2    右四元数 (后施加的旋转)
 * @param[out] q_out 乘积四元数 (等效于先旋转 q1 再旋转 q2)
 * @note 内部使用临时数组, q_out 可与 q1 或 q2 重叠
 */
void RobotMath_QuatMul(const Quat q1, const Quat q2, Quat q_out)
{
    Quat tmp;
    tmp[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    tmp[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    tmp[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    tmp[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    memcpy(q_out, tmp, sizeof(Quat));
}

/**
 * @brief 四元数共轭: q_conj = [w, -x, -y, -z]
 * @param[in]  q      输入四元数
 * @param[out] q_conj 共轭四元数 (逆旋转)
 * @note 对单位四元数, 共轭 = 逆
 */
void RobotMath_QuatConj(const Quat q, Quat q_conj)
{
    Quat tmp = {q[0], -q[1], -q[2], -q[3]};
    memcpy(q_conj, tmp, sizeof(Quat));
}

/**
 * @brief 四元数求逆: q_inv = q_conj / ||q||²
 * @param[in]  q     输入四元数
 * @param[out] q_inv 逆四元数
 * @note 对单位四元数, 逆 = 共轭; 对非单位四元数, 需除以模长平方
 * @note 模长趋近 0 时输出恒等四元数
 */
void RobotMath_QuatInv(const Quat q, Quat q_inv)
{
    float norm_sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (norm_sq < 1e-12f) { q_inv[0]=1.0f; q_inv[1]=q_inv[2]=q_inv[3]=0.0f; return; }
    float inv_norm_sq = 1.0f / norm_sq;
    Quat tmp = {q[0]*inv_norm_sq, -q[1]*inv_norm_sq, -q[2]*inv_norm_sq, -q[3]*inv_norm_sq};
    memcpy(q_inv, tmp, sizeof(Quat));
}

/**
 * @brief 用四元数旋转三维向量: v_out = q · v · q⁻¹
 *
 * 使用高效公式避免两次四元数乘法:
 * @code
 *   t = 2 · q_vec × v
 *   v_out = v + q_w · t + q_vec × t
 * @endcode
 *
 * @param[in]  q     单位四元数 (内部自动归一化)
 * @param[in]  v     待旋转的三维向量
 * @param[out] v_out 旋转后的三维向量
 */
void RobotMath_QuatRotateVec3(const Quat q_in, const Vec3 v, Vec3 v_out)
{
    Quat q = {q_in[0], q_in[1], q_in[2], q_in[3]};
    RobotMath_QuatNormalize(q);
    Vec3 q_vec = {q[1], q[2], q[3]};
    Vec3 t, cross_tmp;
    RobotMath_Vec3Cross(q_vec, v, t);
    RobotMath_Vec3Scale(t, 2.0f, t);
    RobotMath_Vec3Cross(q_vec, t, cross_tmp);
    Vec3 tmp = {
        v[0] + q[0]*t[0] + cross_tmp[0],
        v[1] + q[0]*t[1] + cross_tmp[1],
        v[2] + q[0]*t[2] + cross_tmp[2],
    };
    memcpy(v_out, tmp, sizeof(Vec3));
}

/**
 * @brief 3×3 旋转矩阵 → 单位四元数 (Shepperd 四分支稳定算法)
 *
 * 根据矩阵对角线最大值选择主元, 避免传统单分支公式在迹趋近 0 时的精度丢失。
 *
 * 分支规则:
 *   1. trace > 0         → w 为主元
 *   2. R[0][0] 对角线最大 → x 为主元
 *   3. R[1][1] 对角线最大 → y 为主元
 *   4. R[2][2] 对角线最大 → z 为主元
 *
 * @param[in]  R 输入 3×3 正交旋转矩阵
 * @param[out] q 输出单位四元数 {w, x, y, z}
 * @note 最终强制归一化, 消除浮点舍入误差
 */
void RobotMath_RotToQuat(const Mat3 R, Quat q)
{
    float trace = R[0][0] + R[1][1] + R[2][2];

    if (trace > 0.0f) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        q[0] = 0.25f * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        float s = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25f * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        float s = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25f * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        float s = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25f * s;
    }
    RobotMath_QuatNormalize(q);
}

/**
 * @brief 单位四元数 → 3×3 旋转矩阵
 *
 * 标准公式:
 * @code
 *   R = [ 1-2(y²+z²)   2(xy-wz)     2(xz+wy)  ]
 *       [ 2(xy+wz)     1-2(x²+z²)   2(yz-wx)   ]
 *       [ 2(xz-wy)     2(yz+wx)     1-2(x²+y²) ]
 * @endcode
 *
 * @param[in]  q_in 输入四元数 {w,x,y,z} (内部会自动归一化)
 * @param[out] R    输出 3×3 旋转矩阵
 */
void RobotMath_QuatToRot(const Quat q_in, Mat3 R)
{
    Quat q = {q_in[0], q_in[1], q_in[2], q_in[3]};
    RobotMath_QuatNormalize(q);
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;
    R[0][0] = 1.0f - 2.0f*(yy+zz); R[0][1] = 2.0f*(xy-wz); R[0][2] = 2.0f*(xz+wy);
    R[1][0] = 2.0f*(xy+wz);        R[1][1] = 1.0f-2.0f*(xx+zz); R[1][2] = 2.0f*(yz-wx);
    R[2][0] = 2.0f*(xz-wy);        R[2][1] = 2.0f*(yz+wx);       R[2][2] = 1.0f-2.0f*(xx+yy);
}

/**
 * @brief 从四元数和平移向量构建 4×4 齐次变换矩阵
 * @param[in]  q   单位四元数 (表示旋转)
 * @param[in]  pos 平移向量 (3 维)
 * @param[out] T   输出 4×4 齐次变换矩阵
 */
void RobotMath_MakeTransform(const Quat q, const Vec3 pos, Mat4 T)
{
    Mat3 R;
    RobotMath_QuatToRot(q, R);
    RobotMath_MakeTransformFromRot(R, pos, T);
}

/**
 * @brief 从 4×4 齐次变换提取四元数 (取左上角 3×3 旋转块)
 * @param[in]  T 齐次变换矩阵
 * @param[out] q 输出单位四元数
 */
void RobotMath_Mat4ToQuat(const Mat4 T, Quat q)
{
    Mat3 R;
    RobotMath_GetRot(T, R);
    RobotMath_RotToQuat(R, q);
}

/**
 * @brief 四元数球面线性插值 (Slerp)
 *
 * 沿四维球面大圆弧在两个单位四元数之间平滑插值。
 * 自动处理:
 *   - 最短路径: dot<0 时翻转 q1 取最近弧
 *   - 退化保护: 夹角极小 (dot>0.9995) 时退化为 Lerp 避免除零
 *   - 越界保护: s 自动钳位 [0,1], dot 钳位 [-1,1] 防 NaN
 *
 * @param[in]  q0_in 起点单位四元数
 * @param[in]  q1_in 终点单位四元数
 * @param[in]  s     插值系数, ∈ [0,1], 越界自动钳位
 * @param[out] q_out 插值结果 (已归一化)
 */
void RobotMath_QuatSlerp(const Quat q0_in, const Quat q1_in, float s, Quat q_out)
{
    Quat q0 = {q0_in[0], q0_in[1], q0_in[2], q0_in[3]};
    Quat q1 = {q1_in[0], q1_in[1], q1_in[2], q1_in[3]};
    RobotMath_QuatNormalize(q0);
    RobotMath_QuatNormalize(q1);
    s = clampf(s, 0.0f, 1.0f);

    float dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];
    if (dot < 0.0f) { dot = -dot; q1[0]=-q1[0]; q1[1]=-q1[1]; q1[2]=-q1[2]; q1[3]=-q1[3]; }
    dot = clampf(dot, -1.0f, 1.0f);

    if (dot > 0.9995f) {
        for (int i = 0; i < 4; ++i) q_out[i] = q0[i] + s*(q1[i] - q0[i]);
        RobotMath_QuatNormalize(q_out);
        return;
    }

    float theta_0 = acosf(dot), theta = theta_0 * s;
    float sin_theta = sinf(theta), sin_theta_0 = sinf(theta_0);
    float scale0 = cosf(theta) - dot*sin_theta/sin_theta_0;
    float scale1 = sin_theta / sin_theta_0;
    for (int i = 0; i < 4; ++i) q_out[i] = scale0*q0[i] + scale1*q1[i];
    RobotMath_QuatNormalize(q_out);
}

/* ================================================================
 * 旋转矩阵高级操作
 * ================================================================ */

/**
 * @brief 计算两个旋转矩阵间的最小旋转角
 * @param[in] R0 第一个 3×3 旋转矩阵
 * @param[in] R1 第二个 3×3 旋转矩阵
 * @return 最小旋转角度 (rad), ∈ [0, π]
 * @note 内部转为四元数, 取点积绝对值以得到最短弧
 */
float RobotMath_RotAngleBetween(const Mat3 R0, const Mat3 R1)
{
    Quat q0, q1;
    RobotMath_RotToQuat(R0, q0);
    RobotMath_RotToQuat(R1, q1);
    float dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];
    dot = fabsf(clampf(dot, -1.0f, 1.0f));
    return 2.0f * acosf(dot);
}

/**
 * @brief 旋转矩阵球面线性插值
 *        底层使用四元数 Slerp, 自动保证输出矩阵正交性
 * @param[in]  R0    起点 3×3 旋转矩阵
 * @param[in]  R1    终点 3×3 旋转矩阵
 * @param[in]  s     插值系数 ∈ [0,1]
 * @param[out] R_out 插值后的 3×3 旋转矩阵
 */
void RobotMath_RotSlerp(const Mat3 R0, const Mat3 R1, float s, Mat3 R_out)
{
    Quat q0, q1, q_interp;
    RobotMath_RotToQuat(R0, q0);
    RobotMath_RotToQuat(R1, q1);
    RobotMath_QuatSlerp(q0, q1, s, q_interp);
    RobotMath_QuatToRot(q_interp, R_out);
}

/**
 * @brief ZYX 欧拉角 (RPY) → 3×3 旋转矩阵
 *
 * 旋转顺序: R = Rz(yaw) · Ry(pitch) · Rx(roll)
 * 适用于固定角 (相对于固定基座依次旋转)
 *
 * @param[in]  rpy 欧拉角向量 [roll, pitch, yaw] (rad)
 * @param[out] R   输出 3×3 旋转矩阵
 */
void RobotMath_RPYToRot(const Vec3 rpy, Mat3 R)
{
    float cr=cosf(rpy[0]), sr=sinf(rpy[0]);  // roll
    float cp=cosf(rpy[1]), sp=sinf(rpy[1]);  // pitch
    float cy=cosf(rpy[2]), sy=sinf(rpy[2]);  // yaw
    R[0][0] = cy*cp; R[0][1] = cy*sp*sr - sy*cr; R[0][2] = cy*sp*cr + sy*sr;
    R[1][0] = sy*cp; R[1][1] = sy*sp*sr + cy*cr; R[1][2] = sy*sp*cr - cy*sr;
    R[2][0] = -sp;   R[2][1] = cp*sr;            R[2][2] = cp*cr;
}

/**
 * @brief 3×3 旋转矩阵 → ZYX 欧拉角 (RPY)
 *
 * 从 R = Rz(yaw)·Ry(pitch)·Rx(roll) 中提取 roll/pitch/yaw。
 * 搜索 sp = -R[2][0] = sin(pitch), pitch ∈ [-π/2, +π/2]
 *
 * @param[in]  R   输入 3×3 旋转矩阵
 * @param[out] rpy 输出欧拉角向量 [roll, pitch, yaw] (rad)
 * @note pitch = ±90° 时 (万向节死锁), yaw 强制置 0, roll 与 yaw 耦合
 */
void RobotMath_RotToRPY(const Mat3 R, Vec3 rpy)
{
    float sp_neg = clampf(R[2][0], -1.0f, 1.0f); // -sin(pitch)
    if (fabsf(sp_neg) < 0.99999f) {
        rpy[1] = asinf(-sp_neg);                          // pitch
        rpy[2] = atan2f(R[1][0], R[0][0]);               // yaw
        rpy[0] = atan2f(R[2][1], R[2][2]);               // roll
    } else {
        rpy[2] = 0.0f; // 万向节死锁: yaw 固定
        if (sp_neg < 0.0f) {                               // pitch ≈ +90°
            rpy[1] =  ROBOT_MATH_PI * 0.5f;
            rpy[0] = atan2f(R[0][1], R[1][1]);
        } else {                                           // pitch ≈ -90°
            rpy[1] = -ROBOT_MATH_PI * 0.5f;
            rpy[0] = atan2f(-R[0][1], R[1][1]);
        }
    }
}

/* ================================================================
 * 轴角法
 * ================================================================ */

/**
 * @brief 轴角 → 四元数: q = [cos(θ/2), sin(θ/2)·axis]
 * @param[in]  aa 轴角数据 (axis 会自动归一化)
 * @param[out] q  输出单位四元数; 若 aa==NULL 则输出恒等四元数
 */
void RobotMath_AxisAngleToQuat(const AxisAngle_t *aa, Quat q)
{
    if (aa == NULL) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    Vec3 axis = {aa->axis[0], aa->axis[1], aa->axis[2]};
    RobotMath_Vec3Normalize(axis);
    float half = 0.5f * aa->angle;
    float s = sinf(half);
    q[0] = cosf(half);
    q[1] = axis[0]*s; q[2] = axis[1]*s; q[3] = axis[2]*s;
    RobotMath_QuatNormalize(q);
}

/**
 * @brief 四元数 → 轴角
 * @param[in]  q_in 输入四元数 (自动归一化)
 * @param[out] aa   输出轴角; 若 aa==NULL 则直接返回; 纯零旋转时轴取 (1,0,0)
 * @note 内部自动固定 w≥0, 保证 angle ∈ [0, π]
 */
void RobotMath_QuatToAxisAngle(const Quat q_in, AxisAngle_t *aa)
{
    if (aa == NULL) return;
    Quat q = {q_in[0], q_in[1], q_in[2], q_in[3]};
    RobotMath_QuatNormalize(q);
    if (q[0] < 0.0f) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }
    q[0] = clampf(q[0], -1.0f, 1.0f);
    aa->angle = 2.0f * acosf(q[0]);
    float s = sqrtf(1.0f - q[0]*q[0]);
    if (s < 1e-6f) { aa->axis[0]=1.0f; aa->axis[1]=0.0f; aa->axis[2]=0.0f; }
    else           { aa->axis[0]=q[1]/s; aa->axis[1]=q[2]/s; aa->axis[2]=q[3]/s; }
}

/**
 * @brief 轴角 → 旋转矩阵 (Rodrigues 公式)
 *
 * 公式: R = I + sin(θ)·K + (1-cos(θ))·K², 其中 K = hat(axis)
 * 等效展开式 (避免两次 3×3 乘法, 更高效):
 * @code
 *   R[i][j] = δ[i][j]*c + (1-c)*axis[i]*axis[j] + s*K[i][j]
 * @endcode
 *
 * @param[in]  aa 轴角数据 (axis 会自动归一化); 若 NULL 则输出单位矩阵
 * @param[out] R  输出 3×3 旋转矩阵
 */
void RobotMath_AxisAngleToRot(const AxisAngle_t *aa, Mat3 R)
{
    if (aa == NULL) { RobotMath_Mat3Identity(R); return; }
    Vec3 axis = {aa->axis[0], aa->axis[1], aa->axis[2]};
    RobotMath_Vec3Normalize(axis);
    float x=axis[0], y=axis[1], z=axis[2];
    float c=cosf(aa->angle), s=sinf(aa->angle), v=1.0f-c;
    R[0][0]=c+x*x*v; R[0][1]=x*y*v - z*s; R[0][2]=x*z*v + y*s;
    R[1][0]=y*x*v + z*s; R[1][1]=c+y*y*v; R[1][2]=y*z*v - x*s;
    R[2][0]=z*x*v - y*s; R[2][1]=z*y*v + x*s; R[2][2]=c+z*z*v;
}

/**
 * @brief 旋转矩阵 → 轴角
 *        内部走 RotToQuat → QuatToAxisAngle
 * @param[in]  R  输入 3×3 旋转矩阵
 * @param[out] aa 输出轴角
 */
void RobotMath_RotToAxisAngle(const Mat3 R, AxisAngle_t *aa)
{
    Quat q;
    RobotMath_RotToQuat(R, q);
    RobotMath_QuatToAxisAngle(q, aa);
}

/**
 * @brief 角速度积分更新旋转矩阵: R_new = R · exp(hat(ω·dt))
 *
 * 用于 IMU 姿态传播或仿真步进。
 * 若角速度向量的模长极小 (< 1e-8), 微旋转矩阵退化为单位矩阵, R_new = R。
 *
 * @param[in]  R     当前 3×3 旋转矩阵
 * @param[in]  omega 角速度向量 (rad/s, 在基座坐标系)
 * @param[in]  dt    积分时间步长 (s)
 * @param[out] R_new 更新后的旋转矩阵
 */
void RobotMath_RotIntegrateOmega(const Mat3 R, const Vec3 omega, float dt, Mat3 R_new)
{
    Vec3 delta = {omega[0]*dt, omega[1]*dt, omega[2]*dt};
    float angle = RobotMath_Vec3Norm(delta);
    AxisAngle_t aa;
    Mat3 dR;
    if (angle < 1e-8f) {
        RobotMath_Mat3Identity(dR);
    } else {
        aa.angle = angle;
        aa.axis[0] = delta[0]/angle; aa.axis[1] = delta[1]/angle; aa.axis[2] = delta[2]/angle;
        RobotMath_AxisAngleToRot(&aa, dR);
    }
    RobotMath_Mat3Mul(R, dR, R_new);
}

/* ================================================================
 * 多项式插值
 * ================================================================ */

/**
 * @brief 计算三次多项式系数, 满足位置和速度边界条件
 *
 * 给定:
 *   p(0)=x0, p(T)=xT, p'(0)=v0, p'(T)=vT
 *
 * 解出多项式 p(t) = a0 + a1·t + a2·t² + a3·t³ 的系数:
 * @code
 *   a0 = x0
 *   a1 = v0
 *   a2 = (3*(xT-x0) - (2*v0+vT)*T) / T²
 *   a3 = (2*(x0-xT) + (v0+vT)*T) / T³
 * @endcode
 *
 * @param[in]  x0   起始位置
 * @param[in]  xT   终点位置
 * @param[in]  v0   起始速度
 * @param[in]  vT   终点速度
 * @param[in]  T    总时间 (s), T≤0 时输出恒值 xT
 * @param[out] poly 输出多项式系数; 若 NULL 则直接返回
 */
void RobotMath_CubicInterpPlan(float x0, float xT, float v0, float vT,
                               float T, CubicPoly_t *poly)
{
    if (poly == NULL) return;
    if (T <= 1e-6f) {
        poly->a0 = xT; poly->a1 = 0.0f; poly->a2 = 0.0f; poly->a3 = 0.0f;
        return;
    }
    float T2 = T*T, T3 = T2*T;
    poly->a0 = x0;
    poly->a1 = v0;
    poly->a2 = (3.0f*(xT-x0) - (2.0f*v0+vT)*T) / T2;
    poly->a3 = (2.0f*(x0-xT) + (v0+vT)*T) / T3;
}

/**
 * @brief 计算三次多项式在 t 时刻的值: p(t)
 * @param[in] poly 多项式系数
 * @param[in] t    时刻 (s)
 * @return p(t) 的值; 若 poly==NULL 返回 0
 * @note 使用 Horner 法高效求值: ((a3·t + a2)·t + a1)·t + a0
 */
float RobotMath_CubicInterpEval(const CubicPoly_t *poly, float t)
{
    if (poly == NULL) return 0.0f;
    return ((poly->a3*t + poly->a2)*t + poly->a1)*t + poly->a0;
}

/**
 * @brief 计算三次多项式在 t 时刻的一阶导数: p'(t)
 * @param[in] poly 多项式系数
 * @param[in] t    时刻 (s)
 * @return p'(t) 的值; 若 poly==NULL 返回 0
 */
float RobotMath_CubicInterpEvalDeriv(const CubicPoly_t *poly, float t)
{
    if (poly == NULL) return 0.0f;
    return (3.0f*poly->a3*t + 2.0f*poly->a2)*t + poly->a1;
}

/* ================================================================
 * 低通滤波器
 * ================================================================ */

/**
 * @brief 初始化一阶低通滤波器 (直接指定 alpha)
 *
 * 差分方程: y[n] = alpha·x[n] + (1-alpha)·y[n-1]
 *
 * @param[out] filt  滤波器实例
 * @param[in]  alpha 滤波系数 ∈ [0,1]; 1=无滤波, 0=永不更新 (自动钳位)
 * @note 调用后需在第一拍调用 LowPassFilterStep 完成首拍置位
 */
void RobotMath_LowPassFilterInit(LowPassFilter1P_t *filt, float alpha)
{
    if (filt == NULL) return;
    filt->alpha = clampf(alpha, 0.0f, 1.0f);
    filt->y_prev = 0.0f;
    filt->initialized = 0;
}

/**
 * @brief 按截止频率初始化一阶低通滤波器
 *
 * 公式: RC = 1/(2π·f_cutoff), alpha = dt/(RC+dt)
 *
 * @param[out] filt       滤波器实例
 * @param[in]  cutoff_hz  截止频率 (Hz), ≤0 时退化为 alpha=1 (无滤波)
 * @param[in]  dt         采样周期 (s)
 */
void RobotMath_LowPassFilterInitByFreq(LowPassFilter1P_t *filt, float cutoff_hz, float dt)
{
    if (cutoff_hz <= 0.0f || dt <= 0.0f) { RobotMath_LowPassFilterInit(filt, 1.0f); return; }
    float rc = 1.0f / (ROBOT_MATH_TWO_PI * cutoff_hz);
    RobotMath_LowPassFilterInit(filt, dt / (rc + dt));
}

/**
 * @brief 一阶低通滤波器步进: 输入原始值, 返回滤波后值
 *
 * 首拍自动将 y_prev 初始化为当前 x (避免 0→x 的阶跃响应)
 *
 * @param[in,out] filt 滤波器实例
 * @param[in]     x    当前原始采样值
 * @return 滤波后输出值; 若 filt==NULL 则原样返回 x
 */
float RobotMath_LowPassFilterStep(LowPassFilter1P_t *filt, float x)
{
    if (filt == NULL) return x;
    if (!filt->initialized) { filt->y_prev = x; filt->initialized = 1; return x; }
    filt->y_prev = filt->alpha*x + (1.0f - filt->alpha)*filt->y_prev;
    return filt->y_prev;
}

/**
 * @brief 重置一阶低通滤波器状态 (清零历史值)
 * @param[in,out] filt 滤波器实例; 若 NULL 则忽略
 */
void RobotMath_LowPassFilterReset(LowPassFilter1P_t *filt)
{
    if (filt == NULL) return;
    filt->y_prev = 0.0f;
    filt->initialized = 0;
}

/* ================================================================
 * 6×6 线性方程组求解 (高斯列主元消元法)
 * ================================================================ */

/**
 * @brief 求解 6 阶线性方程组: A · x = b
 *
 * 算法: 高斯消元 + 列主元选择 (partial pivoting) + 回代
 *
 * @param[in]  A 6×6 系数矩阵 (仅使用, 内部复制)
 * @param[in]  b 6 维常数项向量 (仅使用, 内部复制)
 * @param[out] x 6 维解向量
 * @return 0=求解成功; -1=系数矩阵奇异 (某列主元绝对值 < 1e-10)
 * @note 内部创建完整副本, 不修改原始输入 A、b
 * @note 固定 6 阶, 无动态内存分配, 适合嵌入式实时控制
 */
int RobotMath_SolveLinear6x6(const float A[6][6], const Vec6 b, Vec6 x)
{
    float work_A[6][6];
    float work_b[6];
    memcpy(work_A, A, sizeof(float)*36);
    memcpy(work_b, b, sizeof(Vec6));

    // --- 前向消元 (上三角化) ---
    for (int col = 0; col < 6; ++col) {
        // 1. 列主元选择: 找当前列绝对值最大的行
        int pivot = col;
        float max_val = fabsf(work_A[col][col]);
        for (int row = col+1; row < 6; ++row) {
            float val = fabsf(work_A[row][col]);
            if (val > max_val) { max_val = val; pivot = row; }
        }
        // 2. 奇异判定
        if (max_val < 1e-10f) return -1;
        // 3. 交换行 (非自身时)
        if (pivot != col) {
            for (int j = col; j < 6; ++j) {
                float tmp = work_A[col][j]; work_A[col][j] = work_A[pivot][j]; work_A[pivot][j] = tmp;
            }
            float tmp = work_b[col]; work_b[col] = work_b[pivot]; work_b[pivot] = tmp;
        }
        // 4. 消去下方行
        for (int row = col+1; row < 6; ++row) {
            float factor = work_A[row][col] / work_A[col][col];
            for (int j = col; j < 6; ++j) work_A[row][j] -= factor * work_A[col][j];
            work_b[row] -= factor * work_b[col];
        }
    }

    // --- 反向回代 ---
    for (int i = 5; i >= 0; --i) {
        float sum = work_b[i];
        for (int j = i+1; j < 6; ++j) sum -= work_A[i][j] * x[j];
        x[i] = sum / work_A[i][i];
    }
    return 0;
}
