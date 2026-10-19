#ifndef JAM_DETECTOR_H
#define JAM_DETECTOR_H

#include "dji_motor.h"
#include "stdint.h"


/**
 * @brief 卡弹检测 FSM 状态
 *
 * 状态流转:
 * NORMAL ──(电流超阈值)──> SUSPECT ──(超时)──> CONFIRMED ──(反转)──> HANDLING ──(超时)──> NORMAL
 *   ↑                         │                      ↑                        │
 *   └─────────(电流恢复正常)───┘                      └────────────────────────┘
 */
typedef enum
{
    JAM_NORMAL    = 0, //常规状态
    JAM_SUSPECT   = 1, //嫌疑状态
    JAM_CONFIRMED = 2, //确认状态
    JAM_HANDLING  = 3, //处理状态

}JamState_e;

/**
 * @brief 卡弹检测器初始化配置
 *
 * 所有参数均有默认值, 不传则使用默认(直接 memset 为 0 后再赋值所需项)
 */
typedef struct 
{
    float current_threshold; //电流阈值，即CAN原始值
    float suspect_timeout_ms; //嫌疑状态超时时间（ms）
    float handling_timeout_ms; //处理状态超时时间（ms）
    float reverse_speed; //反转速度（°/s）

}JamDetector_Init_Config_s;


/**
 * @brief 卡弹检测器实例
 *
 * 每个拨盘电机对应一个实例, 在 shoot 应用初始化时创建.
 * 外部仅需通过 JamDetectorTask() 轮询, 通过返回值判断当前状态.
 */
typedef struct 
{
    JamState_e state; //当前状态
    float suspect_entry_time; //进入嫌疑状态时的时间戳（ms）
    float handling_entry_time; //进入处理状态时的时间戳（ms）

    float current_threshold;  //参数副本
    float suspect_timeout_ms; 
    float handling_timeout_ms; 
    float reverse_speed; 

     DJIMotorInstance *motor; //拨盘电机实例指针

}JamDetectorInstance;

/**
 * @brief 初始化卡弹检测器
 *
 * @param config 初始化配置, 填入各阈值和角度. 浮点值传 0 则使用默认值
 * @param motor  拨盘电机实例指针 (来自 DJIMotorInit)
 * @return JamDetectorInstance* 返回实例指针, 供 JamDetectorTask() 使用
 */
JamDetectorInstance *JamDetectorInit(JamDetector_Init_Config_s *config, DJIMotorInstance *motor);

/**
 * @brief 卡弹检测 FSM 核心, 每周期调用一次
 *
 * 在 ShootTask 中调用, 调用频率建议 ≥ 200Hz.
 * 调用者根据返回值决定是否跳过常规发射控制:
 *   - 返回 JAM_HANDLING: 跳过 loader 常规控制, 拨盘正在反转处理
 *   - 返回 JAM_CONFIRMED: 本周期已触发反转, 常规控制自然跳过
 *   - 返回 JAM_NORMAL / JAM_SUSPECT: 发射机构正常运行
 *
 * @param jam 卡弹检测器实例
 * @return JamState_e 当前状态
 */
JamState_e JamDetectorTask(JamDetectorInstance *jam);

/**
 * @brief 手动复位卡弹检测器到常规状态
 *
 * 在发射任务停止、遥控器急停等场景调用.
 *
 * @param jam 卡弹检测器实例
 */
void JamDetectorReset(JamDetectorInstance *jam);


#endif