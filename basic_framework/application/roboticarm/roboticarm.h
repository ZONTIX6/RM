/**
 * @file roboticarm.h
 * @brief 机械臂应用层 — 模块间协调与任务调度
 *
 * 本模块是 6 自由度机械臂的**唯一应用层入口**, 负责:
 *   - 初始化 6 个电机的底层驱动 (DM / DJI GM6020 / M2006)
 *   - 组装并注入各算法模块的配置 (运动学、重力补偿、差动腕部等)
 *   - 每个控制周期 (200 Hz) 从 robot_cmd 获取指令, 分派到对应控制模式
 *   - 填充关节状态反馈并通过 Pub-Sub 回传给 robot_cmd
 *
 * 控制模式:
 *   ARM_ZERO_FORCE   — 所有电机输出零力矩 (急停 / 待机)
 *   ARM_POSITION_MODE — 关节空间位置控制 (梯形轨迹 + 串级 PID + 重力前馈)
 *   ARM_CARTESIAN_MODE— 笛卡尔空间控制 (RPY 目标 → IK → 笛卡尔直线轨迹 → 关节 PID)
 *   ARM_VELOCITY_MODE — (暂未实现, 回退到 POSITION_MODE)
 *
 * 框架约束:
 *   - RoboticArmInit() 必须在 RTOS 调度器启动前调用 (由 RobotInit() 调用)
 *   - RoboticArmTask() 由 RobotTask() 以 200 Hz 周期调用
 *   - 本层是唯一可以 include "robot_def.h" 和电机驱动头文件的代码
 *   - 本层不实现任何算法逻辑, 仅做模块编排与单位转换
 *
 * @copyright Copyright (c) 2026 NTU NSE
 */

#ifndef roboticarm_H
#define roboticarm_H

void RoboticArmInit(void);
void RoboticArmTask(void);

#endif
