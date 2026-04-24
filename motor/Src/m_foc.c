/**
  ******************************************************************************
  * @file    m_foc.c
  * @author  jiangyuhao
  * @version V1.1
  * @date    2026-04-24
  * @brief   motor foc 核心算法与状态机
  ******************************************************************************
  */
#include "m_foc.h"
#include <stdio.h>
#include "m_parameter.h"
#include "m_svpwm.h"
#include "m_tick.h"
#include "m_rotor_angle.h"
#include "mcu_adc_cb.h"
#include "m_ctrl.h"

/* 如果你后续启用了测速/霍尔/编码器定时器，取消下面这行的注释并修改为对应的定时器句柄 */
// extern TIM_HandleTypeDef htim3; 

m_foc_unit_t m_foc_unit;

#define M_US_CALCULATE_CYCLE        10   // Us模长计算周期 (例如：50us * 10 = 500us)

#define SLOPE_ADD_1_VALUE           100  // 斜坡加减速增量值1 (快速)
#define SLOPE_ADD_2_VALUE           10   // 斜坡加减速增量值2 (慢速逼近)

/**
  ******************************************************************************
  * @brief  Us模长计算 (包含滤波后ADC的数据处理与斜坡发生器)
  * @param  None.
  * @retval None.
  ******************************************************************************
  */
void m_us_radius_calculate(void)
{
    uint16_t q16_adc_val = 0;
    
    /* ADC滤波值：左移4位转换为 Q16 格式数据 (放大) */
    q16_adc_val = (uint16_t)(adc_unit.spd_voltage.filter_value << 4);
    
    /* Q16格式ADC值限幅 */
    q16_adc_val = (q16_adc_val < SPD_Q16_MIN_VALUE) ? 0 : q16_adc_val;
    q16_adc_val = (q16_adc_val > SPD_Q16_MAX_VALUE) ? SPD_Q16_MAX_VALUE : q16_adc_val;
        
    m_motor_ctrl.q16_spd_val = q16_adc_val;
    
    /* 斜坡加减速发生器，按照设定的 tick 周期执行 */
    if(!m_tick_unit.spd_time)
    {
        m_tick_unit.spd_time = M_US_CALCULATE_CYCLE;   
        
        if(m_motor_ctrl.q16_spd_val > m_us_unit.q16_m_value)
        {
            if((m_motor_ctrl.q16_spd_val - m_us_unit.q16_m_value) > SLOPE_ADD_1_VALUE)
            {
                m_us_unit.q16_m_value += SLOPE_ADD_1_VALUE;
            }
            else if((m_motor_ctrl.q16_spd_val - m_us_unit.q16_m_value) > SLOPE_ADD_2_VALUE)
            {
                m_us_unit.q16_m_value += SLOPE_ADD_2_VALUE;
            }
            else 
            {
                m_us_unit.q16_m_value = m_motor_ctrl.q16_spd_val;
            }
        }
        else
        {
            if((m_us_unit.q16_m_value - m_motor_ctrl.q16_spd_val) > SLOPE_ADD_1_VALUE)
            {
                m_us_unit.q16_m_value -= SLOPE_ADD_1_VALUE;
            }
            else if((m_us_unit.q16_m_value - m_motor_ctrl.q16_spd_val) > SLOPE_ADD_2_VALUE)
            {
                m_us_unit.q16_m_value -= SLOPE_ADD_2_VALUE;
            }
            else 
            {
                m_us_unit.q16_m_value = m_motor_ctrl.q16_spd_val;
            }
        }
    }
    
    /* Us半径最大限幅 */
    if(m_us_unit.q16_m_value > M_MAX_VALUE)
    {
        m_us_unit.q16_m_value = M_MAX_VALUE;
    }
    /* Us半径最小限幅 */
    if(m_us_unit.q16_m_value < M_MIN_VALUE)
    {
        m_us_unit.q16_m_value = M_MIN_VALUE;
    }
}

/**
  ******************************************************************************
  * @brief  电机FOC算法核心状态机执行
  * @param  None.
  * @retval None.
  ******************************************************************************
  */
void m_foc_algorithm_execute(void)
{   
    switch(m_motor_ctrl.state_machine)
    {
        case EXECUTE_MOTOR_STOP:    //电机停止
        {
            m_motor_stop();
        }
        break;
        
        case EXECUTE_MOTOR_START:   //电机启动
        {
            m_tick_unit.boot_charge_time = BOOTSTRAP_BOOST_CHARGING_TIME;
            m_motor_boost_charge();
        }
        break;
        
        case EXECUTE_MOTOR_BOOST_CHARGING: //自举电容充电等待
        {
            if(!m_tick_unit.boot_charge_time)
            {
                m_rotor_angle_init();
                m_motor_ctrl.state_machine = EXECUTE_MOTOR_EXECUTE;
            }
        }
        break;
        
        case EXECUTE_MOTOR_EXECUTE: //电机执行 (进入闭环或开环运行)
        {
            m_foc_unit.rotor_engle = m_rotor_angle_calculate();
            
            /* 设定定子磁场超前转子磁场 90 度，实现最大转矩 (Id=0 控制) */
            switch(m_motor_ctrl.direction)
            {
                case CCW:
                    m_foc_unit.q_engle = m_foc_unit.rotor_engle + EANGLE90;
                break;
                case CW:
                    m_foc_unit.q_engle = m_foc_unit.rotor_engle - EANGLE90;
                break;
            }
            
            /* 执行 SVPWM 空间矢量脉宽调制 */
            m_svpwm_generate(m_us_unit.q16_m_value, m_foc_unit.q_engle);
        }
        break;
    }
}