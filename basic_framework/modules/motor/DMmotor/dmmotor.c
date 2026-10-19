#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
// [ADDED] 引入DWT用于计算控制周期dt
#include "bsp_dwt.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT] = {NULL};

static void DMMotorConfigAssert(Motor_Init_Config_s *config)
{
    Motor_Control_Setting_s *setting;
    Motor_Controller_Init_s *controller;

    if (config == NULL)
        while (1)
            LOGERROR("[dm_motor] init config is null.");

    if (idx >= DM_MOTOR_CNT)
        while (1)
            LOGERROR("[dm_motor] registered motor count exceeds DM_MOTOR_CNT.");

    setting = &config->controller_setting_init_config;
    controller = &config->controller_param_init_config;

    if (setting->angle_feedback_source == OTHER_FEED && controller->other_angle_feedback_ptr == NULL)
        while (1)
            LOGERROR("[dm_motor] angle feedback pointer is null.");

    if (setting->speed_feedback_source == OTHER_FEED && controller->other_speed_feedback_ptr == NULL)
        while (1)
            LOGERROR("[dm_motor] speed feedback pointer is null.");

    if ((setting->feedforward_flag & SPEED_FEEDFORWARD) && controller->speed_feedforward_ptr == NULL)
        while (1)
            LOGERROR("[dm_motor] speed feedforward pointer is null.");

    if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) && controller->current_feedforward_ptr == NULL)
        while (1)
            LOGERROR("[dm_motor] current feedforward pointer is null.");
}
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);
    motor->feedback_count++;
    measure->id = rxbuff[0] & 0x0f;
    measure->state = rxbuff[0] >> 4;

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    // [ADDED] 多圈角度累计,假设两次采样间电机转过的角度小于 PI rad (半圈)
    // 利用position在[-12.5,12.5] rad范围内的跳变检测圈数变化
    if (measure->position - measure->last_position > DM_P_MAX)
        measure->total_round--;
    else if (measure->position - measure->last_position < DM_P_MIN)
        measure->total_round++;
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorConfigAssert(config);

    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    if (motor == NULL)
        while (1)
            LOGERROR("[dm_motor] malloc failed.");

    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    // [ORIGINAL] 以下为原有分散PID初始化,已注释
    // PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    // PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    // PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    // motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    // motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    // ===== 原有代码结束 =====

    // [MODIFIED] 使用统一的Motor_Controller_s,与DJI电机保持一致
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);
    if (motor->motor_can_instace == NULL)
        while (1)
            LOGERROR("[dm_motor] CAN register failed.");

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    // [ORIGINAL] motor->pid_ref = ref;
    // [MODIFIED] 统一使用motor_controller中的pid_ref
    motor->motor_controller.pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}
// [ORIGINAL] DMMotorOuterLoop到此结束

// [ADDED] 切换反馈数据来源,与DJI电机接口保持一致
/**
 * @brief 切换DM电机反馈的目标来源,如将角度和速度的来源换为外部传感器
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dm_motor] loop type error, check memory access and func param");
}


//@Todo: 目前只实现了DM MIT模式力矩输出,应用层可通过闭环配置选择角度/速度/力矩级联
void DMMotorControl(void)
{
    float pid_ref, set, pid_measure;
    DMMotorInstance *motor;
    DM_Motor_Measure_s *measure;
    Motor_Control_Setting_s *setting;
    Motor_Controller_s *motor_controller;
    CANInstance *motor_can;
    DMMotor_Send_s motor_send_mailbox;

    for (size_t i = 0; i < idx; i++)
    {
        motor = dm_motor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        motor_can = motor->motor_can_instace;

        motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
        pid_ref = motor_controller->pid_ref;

        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->position;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->velocity;
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (setting->close_loop_type & CURRENT_LOOP)
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);

        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        set = pid_ref;
        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);

        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;

        if (motor->stop_flag == MOTOR_STOP)
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

        motor_can->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor_can->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor_can->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor_can->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor_can->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor_can->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor_can->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor_can->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor_can, 1);
    }
}

void DMMotorControlInit(void)
{
    // Legacy entry kept for compatibility. DM motors are controlled by DMMotorControl()
    // from the shared MotorControlTask(), matching the project motor framework.
}
