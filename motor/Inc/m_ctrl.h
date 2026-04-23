#ifndef __M_CTRL_H__
#define __M_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "mcu_pwm.h" // 引用定义的 PWM_PERIOD_VALUE (4250)

typedef enum
{
    EXECUTE_MOTOR_STOP,                // 电机停止
    EXECUTE_MOTOR_START,               // 电机启动
    EXECUTE_MOTOR_BOOST_CHARGING,      // 等待充电
    EXECUTE_MOTOR_EXECUTE,             // 电机执行
} motor_execute_state_machine_e;

typedef struct
{
    motor_execute_state_machine_e state_machine;  // 状态机
    uint16_t q16_spd_val;                         // Q16格式的电位器对应ADC值
    uint8_t direction;                            // 电机旋转方向
} m_motor_ctrl_t;

extern m_motor_ctrl_t m_motor_ctrl;

void m_motor_start(void);
void m_motor_stop(void);
void m_motor_boost_charge(void);
void m_motor_execute_ctrl(void);

#ifdef __cplusplus
}
#endif

#endif