/**
  ******************************************************************************
  * @file    m_svpwm.h
  * @author  chengbb (Ported & FPU Optimized for STM32G4)
  * @version V1.1
  * @date    2026-04-24
  * @brief   svpwm头文件
  ******************************************************************************
  */
#ifndef __M__SVPWM_H__
#define __M__SVPWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 引入 STM32 标准库 */
#include "main.h"
#include <stdint.h>

typedef struct
{
    uint16_t Angle;           // Q16 0.16     Us对应的θ角度    
    uint16_t q16_m_value;     // Q16 0.16     系数M值
} m_us_unit_t;

typedef struct
{
    uint16_t sector;        // 扇区编号
    
    uint16_t u_duty_value;  // 电机U相占空比
    uint16_t v_duty_value;  // 电机V相占空比
    uint16_t w_duty_value;  // 电机W相占空比
    
    int16_t q15_ux;         // Q15 1.15
    int16_t q15_uy;         // Q15 1.15
    int16_t q15_uz;         // Q15 1.15
    
    int16_t q15_ta;         // Q15 1.15
    int16_t q15_tb;         // Q15 1.15
    
    uint16_t q16_ta_out;    // Q16 0.16
    uint16_t q16_tb_out;    // Q16 0.16
    uint16_t q16_tc_out;    // Q16 0.16
} m_svpwm_unit_t;

extern m_us_unit_t m_us_unit;
extern m_svpwm_unit_t m_svpwm_unit;

void m_us_sector_calculate(uint16_t theta);
void m_ux_uy_uz_calculate(uint16_t theta); // 修改：传入角度供FPU计算
void m_ta_tb_calculate(int16_t first_x_y_z, int16_t second_x_y_z, uint16_t us_m);
void m_svpwm_duty_calculate(uint16_t us_m);
void m_svpwm_generate(uint16_t us_m, uint16_t us_angle);
    
#ifdef __cplusplus
}
#endif

#endif