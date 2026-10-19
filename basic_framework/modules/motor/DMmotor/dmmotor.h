/**
 * @file dmmotor.h
 * @author neozng
 * @brief  DM电机模块头文件
 * @version 0.2
 * @date 2023-xx-xx
 *
 * @todo 1. 完善三环串级PID
 *       2. 增加失能后复位逻辑
 *       3. 加入前馈控制
 *       4. 添加多圈角度累计
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 */
#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

/* DM电机反馈数据 */
typedef struct
{
    uint8_t id;
    uint8_t state;
    float velocity;        // 角速度,单位为 rad/s
    float last_position;   // 上一次读取的位置,用于多圈累计,单位为 rad
    float position;        // 当前编码器位置,范围为[-12.5,12.5],单位为 rad
    float torque;          // 实际力矩,单位为 Nm
    float T_Mos;           // 电机mos温度,单位为 ℃
    float T_Rotor;         // 转子温度,单位为 ℃
    int32_t total_round;   // 总圈数,用于多圈累计
}DM_Motor_Measure_s;

/* DM电机发送报文数据结构 */
typedef struct
{
    uint16_t position_des; // 期望位置, 0~65535 映射到 DM_P_MIN~DM_P_MAX
    uint16_t velocity_des; // 期望速度, 0~4095 映射到 DM_V_MIN~DM_V_MAX
    uint16_t torque_des;   // 期望力矩, 0~4095 映射到 DM_T_MIN~DM_T_MAX
    uint16_t Kp;           // 位置刚度,0~16383 映射到 0~500 (0为纯力矩控制)
    uint16_t Kd;           // 速度阻尼,0~16383 映射到 0~100 (0为纯力矩控制)
}DMMotor_Send_s;

typedef struct
{
    DM_Motor_Measure_s measure;                 // 电机测量值
    Motor_Control_Setting_s motor_settings;     // 电机设置

    // [MODIFIED] 将分散的PID实例和反馈指针统一为Motor_Controller_s,和DJI电机保持一致,
    // 便于在DMMotorControl中实现三环串级PID
    // ===== 以下为原有代码,已注释 =====
    // PIDInstance current_PID;
    // PIDInstance speed_PID;
    // PIDInstance angle_PID;
    // float *other_angle_feedback_ptr;
    // float *other_speed_feedback_ptr;
    // float *speed_feedforward_ptr;
    // float *current_feedforward_ptr;
    // float pid_ref;
    // ===== 原有代码结束 =====

    // [ADDED] 统一的电机控制器,包含三环PID实例、反馈数据指针和前馈指针
    Motor_Controller_s motor_controller;

    Motor_Working_Type_e stop_flag;             // 启停标志
    CANInstance *motor_can_instace;              // 电机CAN实例
    DaemonInstance* motor_daemon;                // 电机守护进程
    uint32_t lost_cnt;                           // 丢失计数

    // [ADDED] 用于计算控制周期dt,与DJI电机保持一致
    uint32_t feed_cnt;                           // 控制周期计数
    float dt;                                    // 控制周期,单位s
    uint32_t feedback_count;                     // Feedback frame count
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,      // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,      // 停止
    DM_CMD_ZERO_POSITION = 0xfe,   // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb      // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControl(void);
void DMMotorControlInit(void);

// [ADDED] 切换反馈数据来源,与其他电机接口保持一致
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

#endif // !DMMOTOR
