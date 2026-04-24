/**
  ******************************************************************************
  * @file    m_svpwm.c
  * @author  chengbb (Ported & FPU Optimized for STM32G4)
  * @version V1.1
  * @date    2025-01-10
  * @brief   svpwm
  ******************************************************************************
  */
#include "m_svpwm.h"
#include <stdio.h>
#include <math.h>           // 引入数学库，使用FPU
#include "m_parameter.h"
#include "stm32g431xx.h"
#include "typedef_header.h" // 确保包含联合体定义

/* 引入 TIM1 句柄 */
extern TIM_HandleTypeDef htim1;

m_us_unit_t m_us_unit;
m_svpwm_unit_t m_svpwm_unit;

/* 定义浮点常量以加速计算 */
#define PI_F            3.14159265358979f
#define SQRT3_DIV_2_F   0.86602540378f

/**
  ******************************************************************************
  * @brief  获取Us角度对应的扇区
  ******************************************************************************
  */
void m_us_sector_calculate(uint16_t theta)
{
    if      ((theta >= 0)         && (theta < EANGLE60))    m_svpwm_unit.sector = 1;
    else if ((theta >= EANGLE60)  && (theta < EANGLE120))   m_svpwm_unit.sector = 2;
    else if ((theta >= EANGLE120) && (theta < EANGLE180))   m_svpwm_unit.sector = 3;
    else if ((theta >= EANGLE180) && (theta < EANGLE240))   m_svpwm_unit.sector = 4;
    else if ((theta >= EANGLE240) && (theta < EANGLE300))   m_svpwm_unit.sector = 5;
    else                                                    m_svpwm_unit.sector = 6;
}

/**
  ******************************************************************************
  * @brief  基于硬件 FPU 计算 x y z 的值，并转为 Q15 格式
  * @param  theta: 0-65535 对应的 0-2PI 角度
  ******************************************************************************
  */
void m_ux_uy_uz_calculate(uint16_t theta)
{
    /* 1. 将 0-65535 的定点角度转换为单精度浮点弧度值 */
    float theta_rad = (float)theta * (2.0f * PI_F / 65536.0f);
    
    /* 2. 运用 STM32G4 硬件 FPU 执行极限速度的三角函数运算 */
    float f_sin = sinf(theta_rad);
    float f_cos = cosf(theta_rad);
    
    /* 3. 计算浮点下的 x, y, z
       x = sinθ
       y = 1/2 sinθ + √3/2 cosθ
       z = 1/2 sinθ - √3/2 cosθ
    */
    float f_x = f_sin;
    float f_y = 0.5f * f_sin + SQRT3_DIV_2_F * f_cos;
    float f_z = 0.5f * f_sin - SQRT3_DIV_2_F * f_cos;
    
    /* 4. 乘上 32767.0f 还原回原算法的 Q15 (1.15) 定点格式，无缝衔接底层 */
    m_svpwm_unit.q15_ux = (int16_t)(f_x * 32767.0f);
    m_svpwm_unit.q15_uy = (int16_t)(f_y * 32767.0f);
    m_svpwm_unit.q15_uz = (int16_t)(f_z * 32767.0f);
}

/**
  ******************************************************************************
  * @brief  计算第一矢量作用时间ta和第二矢量作用时间tb的值
  ******************************************************************************
  */
void m_ta_tb_calculate(int16_t first_x_y_z, int16_t second_x_y_z, uint16_t us_m)
{
    union_u32 m_t_value;
    union_s32 ta;
    union_s32 tb;
    
    if(first_x_y_z < 0)     first_x_y_z = 0;
    if(second_x_y_z < 0)    second_x_y_z = 0;
    
    /*计算M*T Q16*Q16=Q32*/
    m_t_value.u32 = us_m * (MCU_PWM_TIMER_ARR<<1);      //Q32
    
    /*计算ta Q16*Q15=Q31*/
    ta.s32 = m_t_value.words.high * first_x_y_z;    //Q31 
    m_svpwm_unit.q15_ta = ta.words.high;            //Q15
    
    /*计算tb Q16*Q15=Q31*/
    tb.s32 = m_t_value.words.high * second_x_y_z;    //Q31 
    m_svpwm_unit.q15_tb = tb.words.high;            //Q15
}

/**
  ******************************************************************************
  * @brief  计算taout tbout tcout
  ******************************************************************************
  */
void m_taout_tbout_tcout_calculate(void)
{
    uint16_t v1t = 0; 
    uint16_t v2t = 0;
    uint16_t ta_q15_to_q16 = 0;
    uint16_t tb_q15_to_q16 = 0;
    
    ta_q15_to_q16 = (uint16_t)(((uint32_t)(m_svpwm_unit.q15_ta << 1)) & 0x0000FFFF); //Q16
    tb_q15_to_q16 = (uint16_t)(((uint32_t)(m_svpwm_unit.q15_tb << 1)) & 0x0000FFFF); //Q16
    
    v1t = ta_q15_to_q16 >> 1;   //ta/2
    v2t = tb_q15_to_q16 >> 1;   //tb/2

    m_svpwm_unit.q16_tc_out = (MCU_PWM_TIMER_ARR - v1t - v2t) >> 1;  //tcout:(T/2 - ta/2 - tb / 2) / 2
    m_svpwm_unit.q16_tb_out = m_svpwm_unit.q16_tc_out + v2t;  //tbout = tcout + tb/2
    m_svpwm_unit.q16_ta_out = m_svpwm_unit.q16_tb_out + v1t;  //taout = tbout + ta/2
}

/**
  ******************************************************************************
  * @brief  svpwm输出到 STM32 TIM1
  ******************************************************************************
  */
void m_svpwm_duty_calculate(uint16_t us_m)
{
    switch (m_svpwm_unit.sector)
    {
    case 1:
        m_ta_tb_calculate(-m_svpwm_unit.q15_uz, m_svpwm_unit.q15_ux, us_m);
        m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tc_out;
        break;
    case 2:
        m_ta_tb_calculate(m_svpwm_unit.q15_uz, m_svpwm_unit.q15_uy, us_m);
        m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tc_out;
        break;
    case 3:
        m_ta_tb_calculate(m_svpwm_unit.q15_ux, -m_svpwm_unit.q15_uy, us_m);
        m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tb_out;
        break;
    case 4:
        m_ta_tb_calculate(-m_svpwm_unit.q15_ux, m_svpwm_unit.q15_uz, us_m);
        m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_ta_out;
        break;
    case 5:
        m_ta_tb_calculate(-m_svpwm_unit.q15_uy, -m_svpwm_unit.q15_uz, us_m);
        m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_ta_out;
        break;
    case 6:
        m_ta_tb_calculate(m_svpwm_unit.q15_uy, -m_svpwm_unit.q15_ux, us_m);
        m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tb_out;
        break;
    default:
        break;
    }

    /*最小占空比限制*/
    m_svpwm_unit.u_duty_value = (m_svpwm_unit.u_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit.u_duty_value;
    m_svpwm_unit.v_duty_value = (m_svpwm_unit.v_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit.v_duty_value;
    m_svpwm_unit.w_duty_value = (m_svpwm_unit.w_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit.w_duty_value;
    
    /*最大占空比限制*/
    m_svpwm_unit.u_duty_value = (m_svpwm_unit.u_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit.u_duty_value;
    m_svpwm_unit.v_duty_value = (m_svpwm_unit.v_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit.v_duty_value;
    m_svpwm_unit.w_duty_value = (m_svpwm_unit.w_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit.w_duty_value;

    /* [STM32 移植] 最终占空比给定  duty = period/2 - u_duty_value 
       计算出实际的CCR比较值，并直接写入硬件寄存器 */
    uint16_t ccr_u = MCU_PWM_TIMER_ARR  - m_svpwm_unit.u_duty_value;
    uint16_t ccr_v = MCU_PWM_TIMER_ARR - m_svpwm_unit.v_duty_value;
    uint16_t ccr_w = MCU_PWM_TIMER_ARR - m_svpwm_unit.w_duty_value;
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_w);
}

/**
  ******************************************************************************
  * @brief  Us输出
  ******************************************************************************
  */
void m_svpwm_generate(uint16_t us_m, uint16_t us_angle)
{
    /* 第1步：计算扇区 */
    m_us_sector_calculate(us_angle);
    
    /* 第2步：将角度传给包含 FPU 单精度浮点计算的函数，求出 Q15 的 x,y,z */
    m_ux_uy_uz_calculate(us_angle);
    
    /* 第3步：生成SVPWM波形并输出到 TIM1 */
    m_svpwm_duty_calculate(us_m);
}