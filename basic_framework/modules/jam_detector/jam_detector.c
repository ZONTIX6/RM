#include "jam_detector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h> // memset

#include "bsp_dwt.h"  // DWT_GetTimeline_ms
#include "motor_def.h"// ANGLE_LOOP, SPEED_LOOP, Closeloop_Type_e


#define DEFAULT_CURRENT_THRESHOLD   9830.0f  /**< 默认电流阈值, CAN原始值 (M2006/C610: 6A * 1638.4 ≈ 9830; 注: C610满量程±10A,换算系数1638.4, 而M3508/C620为819.2) */
#define DEFAULT_SUSPECT_TIMEOUT_MS  300.0f   /**< 默认嫌疑超时 300ms */
#define DEFAULT_HANDLING_TIMEOUT_MS 200.0f   /**< 默认处理超时 200ms */
#define DEFAULT_REVERSE_SPEED       1500.0f   /**< 默认反转速度 1500°/s (电机轴), 确保 PID 输出饱和 */

/**
 * @brief 初始化卡弹检测器
 *
 * @param config 配置参数, 传入零值自动填充默认值
 * @param motor  拨盘电机实例
 * @return 检测器实例指针
 */
JamDetectorInstance *JamDetectorInit(JamDetector_Init_Config_s *config, DJIMotorInstance *motor)
{
    JamDetectorInstance *jam = (JamDetectorInstance*)malloc(sizeof(JamDetectorInstance));
    memset(jam,0,sizeof(JamDetectorInstance));

    jam->current_threshold   = (config->current_threshold   > 0.0f ) ? config->current_threshold :   DEFAULT_CURRENT_THRESHOLD;
    jam->suspect_timeout_ms  = (config->suspect_timeout_ms  >0.0f)   ? config->suspect_timeout_ms :  DEFAULT_SUSPECT_TIMEOUT_MS;
    jam->handling_timeout_ms = (config->handling_timeout_ms > 0.0f)  ? config->handling_timeout_ms : DEFAULT_HANDLING_TIMEOUT_MS;
    jam->reverse_speed       = (config->reverse_speed       > 0.0f)  ? config->reverse_speed :       DEFAULT_REVERSE_SPEED;

    jam->motor = motor;
    jam->state = JAM_NORMAL;

    return jam;

}

/**
 * @brief 核心 FSM: 每周期调用, 返回当前状态
 *
 * 调用者需根据返回值决定是否跳过常规 loader 控制:
 *   JAM_CONFIRMED / JAM_HANDLING → 跳过
 *   JAM_NORMAL / JAM_SUSPECT     → 正常运行
 */
JamState_e JamDetectorTask(JamDetectorInstance *jam)
{
    float now = DWT_GetTimeline_ms();
    float abs_current=fabsf((float)jam->motor->measure.real_current);

    switch (jam->state)
    {
    case JAM_NORMAL:
        if(abs_current>jam->current_threshold)
        {
            jam->state = JAM_SUSPECT;
            jam->suspect_entry_time = now;
        }
        break;

    case JAM_SUSPECT:
    
        if(abs_current <= jam->current_threshold)
        {
            jam->state=JAM_NORMAL;
        }
        else if ((now-jam->suspect_entry_time) > jam->suspect_timeout_ms)
        {
            jam->state = JAM_CONFIRMED;
        }
        break;
    
        case JAM_CONFIRMED:

        DJIMotorOuterLoop(jam->motor , SPEED_LOOP);
        DJIMotorSetRef(jam->motor , -jam->reverse_speed); //反向旋转

        jam->handling_entry_time = now;
        jam->state = JAM_HANDLING;
        break;
        
        case JAM_HANDLING:

        if((now-jam->handling_entry_time) > jam->handling_timeout_ms)
        {
            jam->state = JAM_NORMAL;
        }
        break;  
    
    default:
            jam->state = JAM_NORMAL;
        break;
    }

    return jam->state;
}

/**
 * @brief 强制复位到常规状态
 */
void JamDetectorReset(JamDetectorInstance *jam)
{
    jam->state = JAM_NORMAL;
}