/**
  ******************************************************************************
  * @file    m_rotor_angle.c
  * @author  jiangyuhao
  * @version V1.1
  * @date    2026-04-24
  * @brief   转子位置角解算
  ******************************************************************************
  */
/** @addtogroup MOTOR
* @{
*/
#include "m_rotor_angle.h"
#include <stdio.h>
#include "m_parameter.h"
#include "mcu_hall_cb.h"
#include "m_foc.h"
#include "m_ctrl.h"
#include "typedef_header.h" // 确保包含 union_u32 和 LPF_CALC 定义



//转子位置角解算表36BL61 3560
static const uint16_t  ROTOR_ANGLE_TABLE_CCW[7]  = {0,EANGLE330,EANGLE210,EANGLE270,EANGLE90,EANGLE30,EANGLE150};
static const uint16_t  ROTOR_ANGLE_TABLE_CW[7]   = {0,EANGLE30,EANGLE270,EANGLE330,EANGLE150,EANGLE90,EANGLE210};
static const uint16_t  ROTOR_ANGLE_INIT_TABLE[7] = {0,EANGLE0,EANGLE240,EANGLE300,EANGLE120,EANGLE60,EANGLE180};



static union_u32 rotor_angle;
static union_u32 rotor_angle_inc;
static union_u32 monitor_rotor_angle;



m_hall_unit_t m_hall_unit;

/**
  ******************************************************************************
  * @brief  霍尔值获取：获取周期为50us
  ******************************************************************************
  */
void m_hall_value_get(void)
{
    uint8_t hall_u;
    uint8_t hall_v;
    uint8_t hall_w;
    
    /* [STM32 移植] 替换为 STM32 HAL 库直接读取对应 GPIO 状态 */
    hall_u = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);  // TIM3_CH1 (HALL U)
    hall_v = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);  // TIM3_CH2 (HALL V)
    hall_w = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);  // TIM3_CH3 (HALL W)
    
    m_hall_unit.u_val = hall_u;
    m_hall_unit.v_val = hall_v;
    m_hall_unit.w_val = hall_w;     
    
    m_hall_unit.value = (uint8_t)((hall_u << 2) | (hall_v << 1) | (hall_w << 0));
    
    if((m_hall_unit.value > 0) && (m_hall_unit.value < 7))
    {
        if(m_hall_unit.value != m_hall_unit.value_last)
        {
            m_hall_unit.value_last = m_hall_unit.value;
            m_hall_unit.update_sign = true;
        }
    }
    else
    {
        printf("hall value error!\\n");
    }   
}

/**
  ******************************************************************************
  * @brief  转子位置角初始化
  ******************************************************************************
  */
void m_rotor_angle_init(void)
{
    m_hall_unit.time = 0;
    m_hall_unit.time_last = 0;
    m_hall_unit.start_sign = true;
    m_hall_unit.angle_60_time = 0;
    m_hall_unit.angle_60_time_filter1 = 0;
    m_hall_unit.angle_60_time_filter2 = 0;
    m_hall_unit.start_cnt = 0;
    
    
    hall_capture_unit.hall_capture_reset_func();
    m_hall_value_get();
    rotor_angle_inc.u32 = 0;
    
    /* 统一使用标准 32 位数据格式，舍弃对底层非原生的增强宏 */
    rotor_angle.u32 = ROTOR_ANGLE_INIT_TABLE[m_hall_unit.value];
    
    m_hall_unit.hall_val_test_index = 0;
}

/**
 ******************************************************************************
 * @brief  转子位置角计算：间隔50us进行一次计算 (完美融合 STM32 硬件霍尔中断)
 * @retval 转子位置角 Q16 (0~65535 对应 0~360度)
 ******************************************************************************
 */
uint16_t m_rotor_angle_calculate(void)
{
    uint32_t delta_time = 0;
    
    /* 1. 获取当前霍尔引脚电平，判断是否发生状态变化 */
    m_hall_value_get();
    
    /* m_hall_unit.update_sign=true: 霍尔状态在此次 50us 周期内发生了跳变 */
    if(m_hall_unit.update_sign)
    {
        m_hall_unit.update_sign = false;
        m_hall_unit.update_cnt = 0;     
        
        switch(m_motor_ctrl.direction)
        {
            case CCW://逆时针
                /* 获取霍尔沿跳变瞬间的绝对基准转子位置角 */
                rotor_angle.u32 = ROTOR_ANGLE_TABLE_CCW[m_hall_unit.value];
                monitor_rotor_angle.u32 = rotor_angle.u32; 
            break;
            case CW: //顺时针
                rotor_angle.u32 = ROTOR_ANGLE_TABLE_CW[m_hall_unit.value];
                monitor_rotor_angle.u32 = rotor_angle.u32; 
            break;
        }   
        
        /* 2. 检查定时器中断是否已经捕获到了时间 */
        /* 在你的 HAL_TIM_IC_CaptureCallback 中，跳变时 hall_sign 被置位了 */
        if(*hall_capture_unit.hall_sign)
        {
            /* * 【完美对接中断】
             * 中断里已经把 60° 电角度的硬件级时间差存在变量里了，
             * 直接取 hall_capture_val_func() 
             */
            delta_time = hall_capture_unit.hall_capture_val_func(); 
            
            /* 清除中断标志位，等待下一次霍尔边沿跳变 */
            hall_capture_unit.hall_capture_sign_clear_func();
            
            /* 直接更新电角度时间 */
            //m_hall_unit.angle_60_time = delta_time;

            /* -------- 新增：丢弃起步不准的时间 -------- */
            if (m_hall_unit.start_cnt < 1) 
            {
                m_hall_unit.start_cnt++;
                delta_time = 0; // 强制抹掉这个不准的垃圾时间
                m_hall_unit.angle_60_time = 0;
                
                /* 可以顺手清空一下滤波器缓存，防止垃圾值污染 */
                m_hall_unit.angle_60_time_filter1 = 0;
                m_hall_unit.angle_60_time_filter2 = 0;
            }
            else
            {
                /* 第 3 次跳变开始，终于跑满了完整的 60°，采用真实时间！ */
                m_hall_unit.angle_60_time = delta_time;

                if (m_hall_unit.angle_60_time_filter1 == 0)
                {
                    m_hall_unit.angle_60_time_filter1 = delta_time;
                    m_hall_unit.angle_60_time_filter2 = delta_time;
                }
            }
        }

        /* 3. 对 60° 电角度时间进行双重低通滤波，消除机械震动与干扰 */
        if (m_hall_unit.angle_60_time != 0)
        {
            m_hall_unit.angle_60_time_filter1 = LPF_CALC(m_hall_unit.angle_60_time, \
                                                         m_hall_unit.angle_60_time_filter1);
            m_hall_unit.angle_60_time_filter2 = LPF_CALC(m_hall_unit.angle_60_time_filter1, \
                                                         m_hall_unit.angle_60_time_filter2);
        }

        //  /* 4. 起步阶段与极限转速限幅保护 */
        // /* 电机运行初始阶段：霍尔捕获电角度值未到稳定状态 */
        // if(m_hall_unit.start_sign == true)
        // {
        //     m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE;//50RPM 最低转速对应60°电角度时间
        //     if(m_hall_unit.start_cnt++ >= 10)
        //     {
        //         m_hall_unit.start_sign = false;
        //     }
        // }
        // /* 霍尔捕获电角度值已到稳定状态 */
        // else
        // {
        //     m_hall_unit.time = m_hall_unit.angle_60_time_filter2;   
        // }

        m_hall_unit.time = m_hall_unit.angle_60_time_filter2;
        
        if (m_hall_unit.time == 0 || m_hall_unit.start_cnt < 1)
        {
            /* 阶段一：静止瞬间，没有时间差，绝对不插值，保持阶梯波输出最大转矩 */
            rotor_angle_inc.u32 = 0;
            m_motor_ctrl.m_spd.spd_val = 0; // 真实转速为 0
        }
        else
        {
            /* 阶段二与阶段三：只要有了真实的时间差，立刻开启插值！无论速度环是否介入 */
            
            /* 极限转速限幅保护：防止分母过小或过大导致计算溢出 */
            if(m_hall_unit.time <= MAX_SPEED_HALL_TIME_VALUE)
                m_hall_unit.time = MAX_SPEED_HALL_TIME_VALUE;
            if(m_hall_unit.time >= MIN_SPEED_HALL_TIME_VALUE)
                m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE;
                
            /* 极速浮点除法：算出完美的平滑步进角 */
            rotor_angle_inc.u32 = (uint32_t)((float)DθR_DIFF_VALUE / (float)m_hall_unit.time);
            
            /* 计算实际绝对转速，供外部观测或速度环使用 */
            m_motor_ctrl.m_spd.spd_val = 60000000 / (m_hall_unit.time * 6 * MOTOR_POLE_PAIRS);
        }

        if(m_motor_ctrl.m_spd.stabilize_cnt++ >= MOTOR_HALL_STABILIZE_NUMBER)
        {
            m_motor_ctrl.m_spd.stabilize_cnt     = MOTOR_HALL_STABILIZE_NUMBER;
            m_motor_ctrl.m_spd.stabilize_sign    = true;    // 速度计算达到稳定标记
            m_motor_ctrl.m_spd.speed_update_sign = true;    // 触发速度环 PID 运算
        }
    }
    else
    {
        /* 7. 没有霍尔跳变的 50us 周期，根据上一次算出的增量进行【角度插值】平滑估算 */
        if (m_hall_unit.update_cnt < HALL_VALUE_TIMEOUT_THRESHOLD_VALUE) 
        {   
            m_hall_unit.update_cnt++;
            switch(m_motor_ctrl.direction)
            {
                case CCW://逆时针
                    rotor_angle.u32 += rotor_angle_inc.u32;
                break;
                case CW: //顺时针
                    rotor_angle.u32 -= rotor_angle_inc.u32;
                break;
            }   
        }
        else
        {
            /* 超时异常处理 (检测到堵转，超过阈值时间没有收到霍尔跳变信号) */
            m_hall_unit.update_cnt = 0;
            //m_monitor_unit.err_type.rotor_stall = 1;  
        }
    }
    
    /* 返回最终计算/插值后的转子位置角 (0~65535) */
    return (uint16_t)rotor_angle.words.low;   
}

/**
  * @}
  */
/******************* (C) COPYRIGHT 2024 PengLi ******END OF FILE******************/