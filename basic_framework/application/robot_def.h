/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) 2026 NTU NSE
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

#ifndef ROBOTIC_ARM_ENABLED
#define ROBOTIC_ARM_ENABLED 0
#endif

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 59             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

// [ADDED] ===== 机械臂标准DH参数(Craig约定, 6轴拟人臂+差动腕部) =====
// T = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
//
// 构型: J1底座旋转(竖直) → J2肩俯仰(水平∥J3) → J3肘俯仰(水平) → J4小臂自转(沿小臂)
//       → J5腕俯仰(⊥小臂) → J6末端自转(沿工具方向)
//
// 腕部差动机构: 两个M2006通过锥齿轮耦合, 同向→J5俯仰, 反向→J6自转
//              差动解算见 diff_wrist.h
//
//   i | α_{i-1} | a_{i-1} | d_i  | θ_i
//  ───┼──────────┼─────────┼──────┼─────
//   1 |    0     |    0    |   0  |  θ1   ← 底座旋转
//   2 |    π/2   |    0    |   0  |  θ2   ← 肩部俯仰
//   3 |    0     |   304   |   0  |  θ3   ← 肘部俯仰
//   4 |   -π/2   |    0    | 352.6|  θ4   ← 小臂自转
//   5 |   +π/2   |    0    |   0  |  θ5   ← 腕部俯仰(差动向合)
//   6 |   -π/2   |    0    |   0  |  θ6   ← 末端自转(差动差合)
//
// 单位: a/d为mm, alpha为rad, theta为rad(变量)
// ===== 以下为原有代码,已注释 =====
// #define ARM_DH_a1    0.0f         // 关节1: 连杆长度 (mm)
// #define ARM_DH_alpha1 0.0f        // 关节1: 连杆扭转角 (rad), 0°
// ===== 原有代码结束 =====
#define ARM_TOOL_TCP_OFFSET 0.0f  // 腕心到TCP的工具偏移(mm), 不属于DH; 当前FK仍输出腕心/法兰位姿

#define ARM_DH_alpha1 0.0f        // row1 alpha_{0}: 0
#define ARM_DH_a1     0.0f        // row1 a_{0}: 0
#define ARM_DH_d1     0.0f        // row1 d_1: 0

#define ARM_DH_alpha2 1.570796f   // row2 alpha_{1}: +pi/2
#define ARM_DH_a2     0.0f        // row2 a_{1}: 0
#define ARM_DH_d2     0.0f        // row2 d_2: 0

#define ARM_DH_alpha3 0.0f        // row3 alpha_{2}: 0
#define ARM_DH_a3     304.0f      // row3 a_{2}: 304
#define ARM_DH_d3     0.0f        // row3 d_3: 0

#define ARM_DH_alpha4 -1.570796f  // row4 alpha_{3}: -pi/2
#define ARM_DH_a4     0.0f        // row4 a_{3}: 0
#define ARM_DH_d4     352.6f      // row4 d_4: 349.8

#define ARM_DH_alpha5 1.570796f   // row5 alpha_{4}: +pi/2
#define ARM_DH_a5     0.0f        // row5 a_{4}: 0
#define ARM_DH_d5     0.0f        // row5 d_5: 0

#define ARM_DH_alpha6 -1.570796f  // row6 alpha_{5}: -pi/2
#define ARM_DH_a6     0.0f        // row6 a_{5}: 0
#define ARM_DH_d6     0.0f        // row6 d_6: 0

// [ADDED] ===== 机械臂关节软限位 =====
// 关节1-3(DM)和关节4(GM6020)命令单位为rad, 关节5-6为差速虚拟关节, 命令单位为deg
// 请根据实际机械结构设置, 防止关节碰撞!
#define ARM_JOINT1_MIN (-3.14f)   // 关节1下限, ±180°
#define ARM_JOINT1_MAX (3.14f)
#define ARM_JOINT2_MIN (-2.36f)   // 关节2下限, ±135°
#define ARM_JOINT2_MAX (2.36f)
#define ARM_JOINT3_MIN (-2.62f)   // 关节3下限, ±150°
#define ARM_JOINT3_MAX (2.62f)
#define ARM_JOINT4_MIN (-3.14f)   // 关节4下限, ±180°
#define ARM_JOINT4_MAX (3.14f)
#define ARM_JOINT5_MIN (-90.0f)   // 关节5下限(DJI电机单位为deg)
#define ARM_JOINT5_MAX (90.0f)
#define ARM_JOINT6_MIN (-180.0f)  // 关节6下限(DJI电机单位为deg)
#define ARM_JOINT6_MAX (180.0f)

// [ADDED] ===== 机械臂关节零点偏置(Home Offset) =====
// 上电时: 实际关节角 = 电机读数 - home_offset
// J1-J4单位rad, J5-J6为差速虚拟关节, 单位deg (与ARM_JOINT5/6_MIN/MAX一致)
// 请在机械臂标定后填入实测值!
#define ARM_JOINT1_HOME  0.0f
#define ARM_JOINT2_HOME  0.0f
#define ARM_JOINT3_HOME  0.0f
#define ARM_JOINT4_HOME  0.0f
#define ARM_JOINT5_HOME  0.0f   // deg
#define ARM_JOINT6_HOME  0.0f   // deg

// [ADDED] ===== 机械臂轨迹规划默认参数 =====
#define ARM_TRAJ_V_NORM  1.0f     // 关节空间默认归一化速度 (1/s)
#define ARM_TRAJ_A_NORM  2.0f     // 关节空间默认归一化加速度 (1/s²)
#define ARM_CART_MAX_LIN_VEL  50.0f   // 笛卡尔空间默认最大线速度 (mm/s)
#define ARM_CART_MAX_LIN_ACC  100.0f  // 笛卡尔空间默认最大线加速度 (mm/s²)

// [ADDED] ===== 腕部差速机构参数 =====
#define ARM_DIFF_M2006_GEAR_RATIO  36.0f
#define ARM_DIFF_BEVEL_RATIO       1.0f
#define ARM_DIFF_MOTOR_A_DIR       1.0f
#define ARM_DIFF_MOTOR_B_DIR       1.0f
#define ARM_DIFF_JOINT5_DIR        1.0f
#define ARM_DIFF_JOINT6_DIR        1.0f

// [ADDED] ===== J4关节DJI GM6020电机参数 =====
// GM6020反馈ID由DJIMotorInit按0x204 + tx_id自动计算; tx_id=7时为0x20B.
// CAN1上M3508(tx_id=1~4, rx=0x201~0x204)和M2006(tx_id=5~6, rx=0x205~0x206)不冲突.
#define ARM_J4_CAN_HANDLE    &hcan1
#define ARM_J4_TX_ID         7
#define ARM_J4_RX_ID         0x20B

#define ARM_J4_ANGLE_KP      5.0f
#define ARM_J4_ANGLE_KD      0.5f
#define ARM_J4_ANGLE_MAXOUT  5000.0f
#define ARM_J4_SPEED_KP      3.0f
#define ARM_J4_SPEED_MAXOUT  10000.0f

// J4 GM6020直接驱动扭矩(N·m)→DJI电流单位转换系数,需实测标定方向和比例.
#define ARM_DJI6020_TORQUE_TO_CURRENT  25000.0f

// [ADDED] ===== 机械臂动力学参数(重力补偿用) =====
// 重力加速度大小, 基座坐标系Z轴向上, 重力沿-Z方向
#define ARM_GRAVITY_MAG  9.81f

// 连杆质量 (kg), 需根据实际机械臂称重填入
#define ARM_MASS_LINK1  0.5f       // 底座旋转台质量
#define ARM_MASS_LINK2  1.2f       // 大臂质量
#define ARM_MASS_LINK3  0.8f       // 小臂/肘部质量
#define ARM_MASS_LINK4  0.3f       // 腕部旋转质量
#define ARM_MASS_LINK5  0.2f       // 腕部俯仰质量
#define ARM_MASS_LINK6  0.15f      // 末端法兰+工具质量

// 连杆质心在各自连杆坐标系中的位置 (mm), 需通过CAD或实测标定
// Link1: 底座旋转台 (绕Z轴旋转)
#define ARM_COM1_X  0.0f
#define ARM_COM1_Y  0.0f
#define ARM_COM1_Z  50.0f
// Link2: 大臂 (绕Y轴俯仰, X沿大臂方向)
#define ARM_COM2_X  150.0f
#define ARM_COM2_Y  0.0f
#define ARM_COM2_Z  0.0f
// Link3: 小臂/肘部 (绕Y轴俯仰)
#define ARM_COM3_X  0.0f
#define ARM_COM3_Y  0.0f
#define ARM_COM3_Z  0.0f
// Link4: 腕部旋转 (绕X轴旋转, 沿小臂方向)
#define ARM_COM4_X  0.0f
#define ARM_COM4_Y  0.0f
#define ARM_COM4_Z  50.0f
// Link5: 腕部俯仰 (绕Y轴俯仰)
#define ARM_COM5_X  0.0f
#define ARM_COM5_Y  0.0f
#define ARM_COM5_Z  0.0f
// Link6: 末端法兰+工具 (绕X轴旋转)
#define ARM_COM6_X  0.0f
#define ARM_COM6_Y  0.0f
#define ARM_COM6_Z  25.0f

// 腕部M2006差速电机扭矩→电流单位转换系数(仅用于J5/J6差速腕部)
// 该值需要结合M2006减速比、锥齿轮传动比和实测电流-扭矩关系标定
// 粗略估算: 30000 / 1.2 = 25000, 需实测标定后修改
#define ARM_M2006_TORQUE_TO_CURRENT  25000.0f

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// [ADDED] 机械臂模式设置
typedef enum
{
    ARM_ZERO_FORCE = 0, // 电流零输入
    ARM_POSITION_MODE,  // 位置控制模式,每个关节使用角度环
    ARM_VELOCITY_MODE,  // 速度控制模式,每个关节使用速度环(调试用)
    // [ADDED] 笛卡尔空间控制模式, cmd指定末端位姿 -> IK解算关节角 -> 轨迹规划 -> 电机
    ARM_CARTESIAN_MODE,
} robotic_arm_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

// [ADDED] cmd发布的机械臂控制数据,由roboticarm订阅
// 6个关节: 3个DM电机 + J4 DJI GM6020 + 2个DJI M2006差速腕部电机
typedef struct
{
    // ---- 关节空间(ARM_POSITION_MODE / ARM_VELOCITY_MODE) ----
    float joint1_angle; // 关节1(DM电机)目标角度,单位 rad
    float joint2_angle; // 关节2(DM电机)目标角度,单位 rad
    float joint3_angle; // 关节3(DM电机)目标角度,单位 rad
    float joint4_angle; // 关节4(DJI GM6020)目标角度,单位 rad
    float joint5_angle; // 关节5(DJI电机)目标角度,单位 deg
    float joint6_angle; // 关节6(DJI电机)目标角度,单位 deg

    // [ADDED] ---- 笛卡尔空间(ARM_CARTESIAN_MODE) ----
    float target_x;     // 末端目标位置X (mm), 相对于基座坐标系
    float target_y;     // 末端目标位置Y (mm)
    float target_z;     // 末端目标位置Z (mm)
    float target_roll;  // 末端目标姿态Roll (rad)
    float target_pitch; // 末端目标姿态Pitch (rad)
    float target_yaw;   // 末端目标姿态Yaw (rad)

    uint8_t gripper;    // 夹爪控制,0=松开 1=夹紧
    robotic_arm_mode_e arm_mode;
} RoboticArm_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 底盘真实速度,由底盘正运动学解算得到(EstimateSpeed),供上位机/其他模块使用
    float real_vx;
    float real_vy;
    float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

// [ADDED] roboticarm发布的反馈数据,由cmd订阅
typedef struct
{
    float joint1_pos;  // 关节1(DM电机)当前角度,单位 rad
    float joint2_pos;  // 关节2(DM电机)当前角度,单位 rad
    float joint3_pos;  // 关节3(DM电机)当前角度,单位 rad
    float joint4_pos;  // 关节4(DJI GM6020)当前角度,单位 rad
    float joint5_pos;  // 关节5(DJI电机)当前角度,单位 deg
    float joint6_pos;  // 关节6(DJI电机)当前角度,单位 deg
    float joint1_vel;  // 关节1当前速度,单位 rad/s
    float joint2_vel;  // 关节2当前速度,单位 rad/s
    float joint3_vel;  // 关节3当前速度,单位 rad/s
    float joint4_vel;  // 关节4当前速度,单位 rad/s
    float joint5_vel;  // 关节5当前速度,单位 deg/s
    float joint6_vel;  // 关节6当前速度,单位 deg/s
    uint8_t gripper_state; // 夹爪状态,0=松开 1=夹紧
    App_Status_e arm_state; // 机械臂整体状态
} RoboticArm_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;


#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
