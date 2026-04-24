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


/* 移除不再使用的宏定义：ANGLE_ACCURACY_ENHANCEMENT_MODE，我们直接用32位自然溢出 */

static const uint16_t  ROTOR_ANGLE_TABLE_CCW[7]  = {0,EANGLE330,EANGLE210,EANGLE270,EANGLE90,EANGLE30,EANGLE150};
static const uint16_t  ROTOR_ANGLE_TABLE_CW[7]   = {0,EANGLE30,EANGLE270,EANGLE330,EANGLE150,EANGLE90,EANGLE210};
static const uint16_t  ROTOR_ANGLE_INIT_TABLE[7] = {0,EANGLE0,EANGLE240,EANGLE300,EANGLE120,EANGLE60,EANGLE180};

static union_u32 rotor_angle;
static union_u32 rotor_angle_inc;
static union_u32 monitor_rotor_angle;

typedef struct
{
    uint8_t u_val;
    uint8_t v_val;
    uint8_t w_val;
    
    uint8_t value;
    uint8_t value_last;
    bool update_sign;
    uint16_t update_cnt;
    uint8_t start_cnt;
    bool start_sign;
    uint32_t time;
    uint32_t time_last;
    
    uint32_t angle_60_time;
    uint32_t angle_60_time_filter1;
    uint32_t angle_60_time_filter2; 
    
    uint8_t hall_val_test_buf[6];
    uint8_t hall_val_test_index;
} m_hall_unit_t;

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

/* * [STM32 移植精简] 
 * 已经彻底删除原版低效的 m_custom_divide 和 m_custom_remainder。
 * STM32G4 拥有 Cortex-M4F 内核，我们将使用硬件 FPU 进行极速浮点除法运算。
 */

/**
  ******************************************************************************
  * @brief  转子位置角计算：间隔50us进行一次计算
  * @retval 转子位置角 Q16
  ******************************************************************************
  */
uint16_t m_rotor_angle_calculate(void)
{
    m_hall_value_get();
    
    /* m_hall_unit.update_sign=true:霍尔值有更新 */
    if(m_hall_unit.update_sign)
    {
        m_hall_unit.update_sign = false;
        m_hall_unit.update_cnt = 0;     
        
        switch(m_motor_ctrl.direction)
        {
            case CCW://逆时针
                /*获取霍尔沿跳变转子位置角*/
                rotor_angle.u32 = ROTOR_ANGLE_TABLE_CCW[m_hall_unit.value];
                monitor_rotor_angle.u32 = rotor_angle.u32; //转子位置角监测
            break;
            case CW: //顺时针
                /*获取霍尔沿跳变转子位置角*/
                rotor_angle.u32 = ROTOR_ANGLE_TABLE_CW[m_hall_unit.value];
                monitor_rotor_angle.u32 = rotor_angle.u32; //转子位置角监测
            break;
        }   
        
        /* 检测到三相中任意一相沿跳变：捕捉到电角度时间 */
        if(*hall_capture_unit.u_sign || *hall_capture_unit.v_sign || *hall_capture_unit.w_sign)
        {
            hall_capture_unit.hall_capture_sign_clear_func();
            
            /* [STM32 移植] 因为底层采集的已经是直接的 60° 电角度时间
             * 且 u, v, w 的值在回调中被统一赋予了相同的值，
             * 因此只需将三者相加后除以 3 取平均即可 (替代原版的 / 9)
             */
            m_hall_unit.time = (hall_capture_unit.hall_u_capture_val_func() + \
                                hall_capture_unit.hall_v_capture_val_func() + \
                                hall_capture_unit.hall_w_capture_val_func()) / 3;
        }
        
        m_hall_unit.angle_60_time = m_hall_unit.time;

        if (m_hall_unit.angle_60_time != 0)
        {
            /* 沿用原有的低通滤波宏 */
            m_hall_unit.angle_60_time_filter1 = LPF_CALC(m_hall_unit.angle_60_time, \
                                                         m_hall_unit.angle_60_time_filter1);
            m_hall_unit.angle_60_time_filter2 = LPF_CALC(m_hall_unit.angle_60_time_filter1, \
                                                         m_hall_unit.angle_60_time_filter2);
        }
        
        /*电机运行初始阶段：霍尔捕获电角度值未到稳定状态*/
        if(m_hall_unit.start_sign == true)
        {
            m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE;//50RPM     最低转速对应60°电角度值
            if(m_hall_unit.start_cnt++ >= 10)
            {
                m_hall_unit.start_sign = false;
            }
        }
        /*霍尔捕获电角度值未到稳定状态*/
        else
        {
            m_hall_unit.time = m_hall_unit.angle_60_time_filter2;   //>50RPM <3000RPM对应60°电角度值
        }
        
        if (m_hall_unit.time <= MAX_SPEED_HALL_TIME_VALUE) //3000RPM 最高转速对应60°电角度值   
        {               
            m_hall_unit.time = MAX_SPEED_HALL_TIME_VALUE;
        }
        
        /* * [STM32 降维打击：FPU极速浮点除法]
         * 直接利用 STM32 硬件 FPU 进行浮点除法，14个时钟周期搞定，
         * 彻底淘汰低效的 while 减法循环。求得每个 50us 控制周期的角度增量。
         */
        rotor_angle_inc.u32 = (uint32_t)((float)DθR_DIFF_VALUE / (float)m_hall_unit.time); 
    }
    else
    {
        if (m_hall_unit.update_cnt < HALL_VALUE_TIMEOUT_THRESHOLD_VALUE) //65535 x 50us = 3.276750s
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
            //异常处理
            m_hall_unit.update_cnt = 0;
        }
    }
    
    /* 统一返回低16位作为最终的 0~65535 的 Q16 转子角度 */
    return (uint16_t)rotor_angle.words.low;   
}

/**
  * @}
  */
/******************* (C) COPYRIGHT 2024 PengLi ******END OF FILE******************/