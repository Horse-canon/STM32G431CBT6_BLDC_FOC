#include "m_ctrl.h"
#include <stdio.h>
#include "m_parameter.h"

/* 定义 0% 占空比对应的 CCR 值 */
/* 在 PWM Mode 2 下，CCR = ARR(4250) 时，上管全关，下管全开 */
#define MOTOR_DUTY_ZERO_PERCENT    MCU_PWM_TIMER_ARR 

m_motor_ctrl_t m_motor_ctrl = 
{
    .state_machine = EXECUTE_MOTOR_STOP
};

/**
 * @brief  电机预充电逻辑
 * @note   占空比设为 0 (上管关，下管开)
 */
void m_motor_boost_charge(void)
{    
    m_motor_ctrl.state_machine = EXECUTE_MOTOR_BOOST_CHARGING;
    
    /* 物理表现为 0% 占空比 */
    drv_pwm_set_duty(MOTOR_DUTY_ZERO_PERCENT, MOTOR_DUTY_ZERO_PERCENT, MOTOR_DUTY_ZERO_PERCENT);
}

/**
 * @brief  电机启动逻辑
 * @note   启动初始占空比设为 0
 */
void m_motor_start(void)
{
    m_motor_ctrl.state_machine = EXECUTE_MOTOR_START;
    
    /* 物理表现为 0% 占空比 */
    drv_pwm_set_duty(MOTOR_DUTY_ZERO_PERCENT, MOTOR_DUTY_ZERO_PERCENT, MOTOR_DUTY_ZERO_PERCENT);
}

/**
 * @brief  电机停止逻辑
 * @note   停止占空比设为 0
 */
void m_motor_stop(void)
{
    m_motor_ctrl.state_machine = EXECUTE_MOTOR_STOP;
    
    /* 物理表现为 0% 占空比 */
    drv_pwm_set_duty(MOTOR_DUTY_ZERO_PERCENT, MOTOR_DUTY_ZERO_PERCENT, MOTOR_DUTY_ZERO_PERCENT);
}

/**
 * @brief  电机执行控制逻辑
 * @note   保持原瑞萨状态机跳转逻辑不变
 */
void m_motor_execute_ctrl(void)
{
    if(m_motor_ctrl.state_machine == EXECUTE_MOTOR_STOP)
    {
        if(m_motor_ctrl.q16_spd_val > 0)
        {
            m_motor_start();
        }   
    }
    else if(m_motor_ctrl.state_machine != EXECUTE_MOTOR_STOP)
    {
        if(m_motor_ctrl.q16_spd_val == 0)
        {
            m_motor_stop();
        }
    }
}