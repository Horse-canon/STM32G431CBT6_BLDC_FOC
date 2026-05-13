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

// //转子位置角解算表
// static const uint16_t  ROTOR_ANGLE_TABLE_CCW[7]  = {0,EANGLE240,EANGLE120,EANGLE180,EANGLE0,EANGLE300,EANGLE60};
// static const uint16_t  ROTOR_ANGLE_TABLE_CW[7]   = {0,EANGLE300,EANGLE180,EANGLE240,EANGLE60,EANGLE0,EANGLE120};
// static const uint16_t  ROTOR_ANGLE_INIT_TABLE[7] = {0,EANGLE270,EANGLE150,EANGLE210,EANGLE30,EANGLE330,EANGLE90};


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

    /* --------强制清除上电首次读取造成的假跳变标志 -------- */
    m_hall_unit.update_sign = false; 
    /* ---------------------------------------------------------------- */

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

           /* -------- 核心修改：第一次丢弃，第二次立刻启用 -------- */
            if (m_hall_unit.start_cnt == 0) 
            {
                /* 发生第 1 次跳变：距离残缺，真实时间作废 */
                m_hall_unit.start_cnt++;
                delta_time = 0; 
                m_hall_unit.angle_60_time = 0;
                
                m_hall_unit.angle_60_time_filter1 = 0;
                m_hall_unit.angle_60_time_filter2 = 0;
            }
            else
            {
                /* 发生第 2 次及以后的跳变：跑满了完整 60°，时间绝对真实！ */
                if(m_hall_unit.start_cnt < 255) m_hall_unit.start_cnt++; // 防止溢出
                
                m_hall_unit.angle_60_time = delta_time;

                /* 滤波器种子初始化：防止初始测速被 0 拖后腿 */
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

        m_hall_unit.time = m_hall_unit.angle_60_time_filter2;
        
/* -------- 核心修改：三段式起步策略 -------- */
        if (m_hall_unit.start_cnt == 0)
        {
            /* 阶段一 (刚通电，未跳变)：
               死锁在扇区中点，提供最大恒定推力，绝不插值 */
            rotor_angle_inc.u32 = 0;
            m_motor_ctrl.m_spd.spd_val = 0; 
        }
        else if (m_hall_unit.start_cnt == 1)
        {
            /* 阶段二 (第 1 次跳变后，等待第 2 次跳变)：
               为了防止磁场死锁导致 Iq 回落，人为给定一个恒定的低速插值步长！ */
            m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE; // 使用设定的最低转速(如50RPM)
            rotor_angle_inc.u32 = (uint32_t)((float)DθR_DIFF_VALUE / (float)m_hall_unit.time);
            
            /* 注意：虽然给定了假插值，但反馈给速度环的真实速度依然保持为 0，防止 PID 干扰起步 */
            m_motor_ctrl.m_spd.spd_val = 0; 
        }
        else
        {
            /* 阶段三 (第 2 次跳变及以后)：
               拥有了真实的 60 度时间，接入真实时间，丝滑闭环插值！ */
            if(m_hall_unit.time <= MAX_SPEED_HALL_TIME_VALUE)
                m_hall_unit.time = MAX_SPEED_HALL_TIME_VALUE;
            if(m_hall_unit.time >= MIN_SPEED_HALL_TIME_VALUE)
                m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE;
                
            rotor_angle_inc.u32 = (uint32_t)((float)DθR_DIFF_VALUE / (float)m_hall_unit.time);
            m_motor_ctrl.m_spd.spd_val = 60000000 / (m_hall_unit.time * 6 * MOTOR_POLE_PAIRS);
        }

        if(m_motor_ctrl.m_spd.stabilize_cnt++ >= MOTOR_HALL_STABILIZE_NUMBER)
        {
            m_motor_ctrl.m_spd.stabilize_cnt     = MOTOR_HALL_STABILIZE_NUMBER;
            m_motor_ctrl.m_spd.stabilize_sign    = true;    // 速度计算达到稳定标记
            //m_motor_ctrl.m_spd.stabilize_sign    = false;    // 速度计算达到稳定标记
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