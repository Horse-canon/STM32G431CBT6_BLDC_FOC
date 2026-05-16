/**
 ******************************************************************************
 * @file    m_coordinate.c
 * @author  jiangyuhao (Optimized for STM32G4 FPU)
 * @brief   FOC 坐标变换源文件
 ******************************************************************************
 */
#include "m_coordinate.h"
#include "m_foc.h"
#include <stdio.h>
#include "m_parameter.h"
#include "m_svpwm.h"
#include "m_tick.h"
#include "m_rotor_angle.h"
#include "mcu_adc_cb.h" 
#include "m_ctrl.h"
#include "typedef_header.h"

/* 引入标准C数学库，利用STM32G4硬件FPU加速计算 */
#include <math.h> 

/* 定义一个宏，用于将 0~65535 的角度值转换为 0~2π 弧度 */
/* 2π ≈ 6.28318530718f，转换系数为 6.28318530718 / 65536.0 = 0.00009587379f */
#define ANGLE_TO_RAD(angle)  ((float)(angle) * 0.00009587379f)

/**
 ******************************************************************************
 * @brief  相电流计算
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_phase_current_calculate(void)
{
    int16_t ia_raw = 0;
    int16_t ib_raw = 0;
    int16_t ic_raw = 0;
    int16_t ia = 0;
    int16_t ib = 0;
    int16_t ic = 0;
    
    /*
        初始静态误差值-正常电流采样值=反应相电流方向/大小值
        
        u_current.instant_value = (IU+  -  IBUS+) * 50 + 1.65V +静态误差 * 50 
        v_current.instant_value = (IV+  -  IBUS+) * 50 + 1.65V +静态误差 * 50
        w_current.instant_value = (IW+  -  IBUS+) * 50 + 1.65V +静态误差 * 50
        <<3：将ADC值转换为Q15格式
    */
    ia_raw = (int16_t)(adc_unit.u_current_offset - adc_unit.u_current.instant_value) << 3;
    ib_raw = (int16_t)(adc_unit.v_current_offset - adc_unit.v_current.instant_value) << 3;
    ic_raw = (int16_t)(adc_unit.w_current_offset - adc_unit.w_current.instant_value) << 3;
    
    /*根据扇区编号：只保留采样窗口最大的一相ADC值，其他两相用KCL定律重建
      采样窗口最大的相 = 占空比最大的相 = CCR最小的相 = 下管导通时间最长的相
    */
    switch(m_svpwm_unit.sector)
    {
        case 1: // U相窗口最大，保留U；重建V、W
            ia = ia_raw;
            ib = 0 - ia - ic_raw;
            ic = 0 - ia - ib_raw;
            break;
        case 2: // V相窗口最大，保留V；重建U、W
            ib = ib_raw;
            ia = 0 - ib - ic_raw;
            ic = 0 - ib - ia_raw;
            break;
        case 3: // V相窗口最大，保留V；重建U、W
            ib = ib_raw;
            ia = 0 - ib - ic_raw;
            ic = 0 - ib - ia_raw;
            break;
        case 4: // W相窗口最大，保留W；重建U、V
            ic = ic_raw;
            ia = 0 - ic - ib_raw;
            ib = 0 - ic - ia_raw;
            break;
        case 5: // W相窗口最大，保留W；重建U、V
            ic = ic_raw;
            ia = 0 - ic - ib_raw;
            ib = 0 - ic - ia_raw;
            break;
        case 6: // U相窗口最大，保留U；重建V、W
            ia = ia_raw;
            ib = 0 - ia - ic_raw;
            ic = 0 - ia - ib_raw;
            break;
        default:
            ia = ia_raw;
            ib = ib_raw;
            ic = ic_raw;
            break;
    }
    
    m_foc_unit.coordinate.q15_ia = ia;
    m_foc_unit.coordinate.q15_ib = ib;
    m_foc_unit.coordinate.q15_ic = ic;

    //printf("ia: %d, ib: %d, ic: %d\r\n", ia, ib, ic);
}

/**
 ******************************************************************************
 * @brief  电流Clark变换
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_clark_transform(void)
{
    union_s32 val_a;
    union_s32 val_b;
    union_s32 val_beta;
    /*Iα=Ia*/
    m_foc_unit.coordinate.q15_i_alpha = m_foc_unit.coordinate.q15_ia;
    /*Iβ=1/sqrt(3)*(Ia+2*Ib)*/
    val_a.s32 = ONE_DIV_SQRT3 * m_foc_unit.coordinate.q15_ia; // 1/sqrt(3)*Ia    Q15*Q16=Q31
    val_b.s32 = ONE_DIV_SQRT3 * m_foc_unit.coordinate.q15_ib; // 1/sqrt(3)*Ib    Q15*Q16=Q31
    val_b.s32 = val_b.s32 * 2;                                // 1/sqrt(3)*Ib*2  Q31
    val_beta.s32 = val_a.s32 + val_b.s32;                     // Iβ=1/sqrt(3)*Ia + 1/sqrt(3)*Ib*2 Q31
    m_foc_unit.coordinate.q15_i_beta = val_beta.words.high;   // 取高16位，Q15
}

/**
 ******************************************************************************
 * @brief  电流Clark逆变换
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_inverse_clark_transform(void)
{
    union_s32 val;
    /*Ia=Iα*/
    m_foc_unit.coordinate.q15_iclark_ia = m_foc_unit.coordinate.q15_i_alpha;
    
    /*Ib=sqrt(3)*Iβ/2-*Iα/2*/
    val.s32 = SQRT3DIV2 * m_foc_unit.coordinate.q15_i_beta - \
              (int32_t)((m_foc_unit.coordinate.q15_i_alpha >> 1) << 16);
    m_foc_unit.coordinate.q15_iclark_ib = val.words.high;   // 取高16位，Q15
    
    /*Ic=-*Iα/2-sqrt(3)*Iβ/2*/
    val.s32 = -(int32_t)((m_foc_unit.coordinate.q15_i_alpha >> 1) << 16) - \
              SQRT3DIV2 * m_foc_unit.coordinate.q15_i_beta;
    m_foc_unit.coordinate.q15_iclark_ic = val.words.high;   // 取高16位，Q15
}

/**
 ******************************************************************************
 * @brief  电流Park变换 (使用STM32G4 FPU硬件加速)
 * @param  rotor_engle：转子位置角 (0~65535 对应 0~360度)
 * @retval None.
 ******************************************************************************/
void m_park_transform(uint16_t rotor_engle)
{
    int32_t id = 0;
    int32_t iq = 0;
    
    /* 1. 将 0~65535 的定点角度转换为 0~2π 的浮点弧度 */
    float angle_rad = ANGLE_TO_RAD(rotor_engle);
    
    /* 2. 调用 math.h 的硬件浮点三角函数 */
    float sin_f = sinf(angle_rad);
    float cos_f = cosf(angle_rad);
    
    /* 3. 将计算结果转换为原代码需要的 Q15 格式 (乘 32768) */
    int16_t q15_sin = (int16_t)(sin_f * 32768.0f);
    int16_t q15_cos = (int16_t)(cos_f * 32768.0f);
    
    /*Id=Iα*cosθr+Iβ*sinθr*/
    id = m_foc_unit.coordinate.q15_i_alpha * q15_cos + \
         m_foc_unit.coordinate.q15_i_beta * q15_sin;      //Q15*Q15=Q30
    
    m_foc_unit.coordinate.q15_id = (int16_t)(id >> 15);   //Q30右移15位=Q15
    
    /*Iq=Iβ*cosθr-Iα*sinθr*/
    iq = m_foc_unit.coordinate.q15_i_beta * q15_cos - \
         m_foc_unit.coordinate.q15_i_alpha * q15_sin;     //Q15*Q15=Q30
     
    m_foc_unit.coordinate.q15_iq = (int16_t)(iq >> 15);   //Q30右移15位=Q15
}

/**
 ******************************************************************************
 * @brief  电压park逆变换 (使用STM32G4 FPU硬件加速)
 * @param  rotor_engle：转子位置角 (0~65535 对应 0~360度)
 * @retval None.
 ******************************************************************************/
void m_inverse_park_transform(uint16_t rotor_engle)
{
    int32_t u_alpha = 0;
    int32_t u_beta = 0;
    
    /* 1. 将 0~65535 的定点角度转换为 0~2π 的浮点弧度 */
    float angle_rad = ANGLE_TO_RAD(rotor_engle);
    
    /* 2. 调用 math.h 的硬件浮点三角函数 */
    float sin_f = sinf(angle_rad);
    float cos_f = cosf(angle_rad);
    
    /* 3. 将计算结果转换为原代码需要的 Q15 格式 (乘 32768) */
    int16_t q15_sin = (int16_t)(sin_f * 32768.0f);
    int16_t q15_cos = (int16_t)(cos_f * 32768.0f);
    
    /*Uα=Ud*cosθr-Uq*sinθr*/
    u_alpha = m_foc_unit.coordinate.q15_ud * q15_cos - \
              m_foc_unit.coordinate.q15_uq * q15_sin;     //Q15*Q15=Q30
    
    m_foc_unit.coordinate.q15_u_alpha = (int16_t)(u_alpha >> 15);   //Q30右移15位=Q15
    
    /*Uβ=Ud*sinθr+Uq*cosθr*/
    u_beta  = m_foc_unit.coordinate.q15_ud * q15_sin + \
              m_foc_unit.coordinate.q15_uq * q15_cos;     //Q15*Q15=Q30
    
    m_foc_unit.coordinate.q15_u_beta = (int16_t)(u_beta >> 15);     //Q30右移15位=Q15
}

/**
 ******************************************************************************
 * @brief  Us模长和超前角求取
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_us_theta_c_calculate(void)
{
    uint16_t ud_2;
    uint16_t uq_2;
    uint16_t uq_max_2;
    uint16_t us_2;
    uint16_t us_max_2;
    float    uq_max_f;
    int16_t  uq_max;
    float    us_f;
    
    /*Us模长最大限幅到90%*/
    m_foc_unit.q15_us_max = US_MAX_VALUE;
    
    /*
        Ud^2+Uq^2=Us^2
        Q15*Q15=Q30,Q30右移14位=Q16
    */
    ud_2 = (uint16_t)((uint32_t)(m_foc_unit.coordinate.q15_ud * m_foc_unit.coordinate.q15_ud) >> 14);
    us_max_2 = (uint16_t)((uint32_t)(m_foc_unit.q15_us_max * m_foc_unit.q15_us_max) >> 14);
    
    uq_max_2 = us_max_2 - ud_2;
    
    /* 使用 math.h 的 sqrtf()，硬件FPU会自动加速 */
    uq_max_f = sqrtf((float)uq_max_2 / 65536.0f);
    
    uq_max = ((uint16_t)(uq_max_f * 65536.0f)) >> 1;
    
    /*Uq正负限幅*/
    if(m_foc_unit.coordinate.q15_uq > uq_max)  m_foc_unit.coordinate.q15_uq = uq_max;
    if(m_foc_unit.coordinate.q15_uq < -uq_max) m_foc_unit.coordinate.q15_uq = -uq_max;
    
    uq_2 = (uint16_t)((uint32_t)(m_foc_unit.coordinate.q15_uq * m_foc_unit.coordinate.q15_uq) >> 14);
    us_2 = ud_2 + uq_2;
    
    /* 使用 math.h 的 sqrtf()计算模长 */
    us_f = sqrtf((float)us_2 / 65536.0f);
    m_foc_unit.q16_us = (uint16_t)(us_f * 65536.0f);
    
    /*超前角计算：arctan(Ud/Uq)*/
    if(m_foc_unit.coordinate.q15_uq == 0 || m_foc_unit.coordinate.q15_ud == 0)
    {
        m_foc_unit.advance_angle = 0; // 超前角直接设置为0°
    }
    else
    {
        float x = (float)m_foc_unit.coordinate.q15_uq / 32768.0f;
        float y = (float)m_foc_unit.coordinate.q15_ud / 32768.0f;
        float result_rad_f;
        float result_angle_f;
        
        #define PI_DIV_180          (180.0f / M_PI)
        #define RAD_TO_ANGLE(rad)   (float)(rad * PI_DIV_180)  
        
        switch(m_motor_ctrl.direction)
        {
            case CCW:
                /* 使用 math.h 的 atan2f() 计算 */
                result_rad_f = atan2f(-y, x);
                result_angle_f = RAD_TO_ANGLE(result_rad_f) / 360.0f;
                m_foc_unit.advance_angle = (uint16_t)(result_angle_f * 65536.0f); 
            break;
            case CW:
                /* 使用 math.h 的 atan2f() 计算 */
                result_rad_f = atan2f(-y, -x);
                result_angle_f = RAD_TO_ANGLE(result_rad_f) / 360.0f;
                m_foc_unit.advance_angle = (uint16_t)(result_angle_f * 65536.0f); 
            break;
        }
    }
}