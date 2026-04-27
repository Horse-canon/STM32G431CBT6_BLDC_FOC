#ifndef __M_CTRL_H__
#define __M_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "mcu_pwm.h" // 引用定义的 PWM_PERIOD_VALUE (4250)
#include "stdbool.h"

typedef enum
{
    EXECUTE_MOTOR_STOP,                // 电机停止
    EXECUTE_MOTOR_START,               // 电机启动
    EXECUTE_MOTOR_BOOST_CHARGING,      // 等待充电
    //EXECUTE_MOTOR_OPEN_LOOP,		//电机开环执行  test code
    EXECUTE_MOTOR_EXECUTE,             // 电机执行
} motor_execute_state_machine_e;

typedef struct
{
	volatile uint16_t stabilize_cnt;//速度稳定计数值
	volatile bool stabilize_sign;   //速度稳定标记
	volatile uint16_t spd_val;      //速度实时计算值
	volatile uint16_t set_spd_val;  //速度设定值
	volatile bool speed_update_sign;//速度更新标记
}m_spd_unit_t;

typedef struct
{
    motor_execute_state_machine_e state_machine;  // 状态机
    volatile bool offset_current_sign;			//相电流静态误差计算标记
	volatile uint8_t  direction;				//电机旋转方向 CW CCW
    uint16_t q16_spd_val;                         // Q16格式的电位器对应ADC值
    volatile int16_t  q15_start_iq;				//启动运行阶段Iq电流值
    m_spd_unit_t m_spd;                          //速度计算模块
} m_motor_ctrl_t;

extern m_motor_ctrl_t m_motor_ctrl;

void m_motor_phase_current_offset_calculate(void);
void m_motor_init(void);
void m_motor_start(void);
void m_motor_stop(void);
void m_motor_boost_charge(void);
void m_motor_execute_ctrl(void);

#ifdef __cplusplus
}
#endif

#endif