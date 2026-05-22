#include <math.h>
#include "m_observer.h"
#include "m_rotor_angle_sensorless.h"
#include "m_parameter.h"
#include "m_ctrl.h"

/* ========================================================== */
/* ================= 无感相关宏定义与参数 ===================== */
/* ========================================================== */
m_obs_angle_unit_t m_obs_angle_unit;

#define DELTA_T_NUMBER                  20               // δt累加次数：20 * 50us = 1ms 无感转速估算
#define ERPM_SECOND_MUL_ONE_THOUSAND    (uint16_t)60000  // 60s * 1000(1ms的倒数)，完美契合 >> 16 的数学变换

#ifndef M_PI
#define M_PI                            3.14159265358979f
#endif

#define MOTOR_FC_SPEED                  300              // 截止频率设定的最低机械转速 300 RPM
#define DELTA_THETA_FC                  (MOTOR_FC_SPEED * MOTOR_POLE_PAIRS / 60.0f) // 对应电频率 (Hz)

/* δθ对应滤波系数 Q15格式 (K = T * 2π * fc * 32768) */
/* PWM_PERIOD_T = 0.00005f */
#define DELTA_THETA_K                   ((int16_t)((PWM_PERIOD_T * M_PI * 2.0f * (float)DELTA_THETA_FC) * 32768.0f) + 1)

/* 最低转速对应δθ值 Q15格式 */
/* DELTA_ONE_PART_OF_T = 1 / 0.001 = 1000 */
#define MIN_RPM_DELTA_THETA             ((int16_t)((float)DELTA_THETA_FC / 1000.0f * 2.0f * M_PI / M_PI * 32768.0f))

#define KAFC_T                          (float)(PWM_PERIOD_T * 20.0f) // 0.001s
#define Q16_T_PI_DIV_DELTA_T            ((uint32_t)(PWM_PERIOD_T * M_PI / KAFC_T * 65536.0f))


/**
  ******************************************************************************
  * @brief  无感转子位置角与速度提取初始化
  ******************************************************************************
  */
void m_sensorless_rotor_angle_init(void)
{
    m_obs_angle_unit.q15_k_dt = DELTA_THETA_K;   // 初始化 δθ 滤波系数
}

/**
  ******************************************************************************
  * @brief  θe 角度计算 (STM32G4 FPU 硬件加速版) + 90° 相位补偿
  * @retval 补偿后的 θe 角度 (利用 int16_t 溢出特性实现环形 0~65535)
  ******************************************************************************
  */
static int16_t m_sensorless_theta_e_calculate(void)
{
    int16_t q15_theta_e;
    
    /* 1. 利用 FPU 直接求反正切，取代查表，精度极高 */
    float e_alpha = (float)m_obs_unit.q15_e_alpha_final;
    float e_beta  = (float)m_obs_unit.q15_e_beta_final;
    
    /* 理论反电动势：Eα = -ω*Ψ*sinθ, Eβ = ω*Ψ*cosθ  => θ = atan2(-Eα, Eβ) */
    float theta_rad = atan2f(-e_alpha, e_beta); 
    
    if (theta_rad < 0.0f) {
        theta_rad += 2.0f * M_PI;
    }
    
    /* 转换为 Q16/Q15 等效格式 (65536 / 2π ≈ 10430.378f) */
    /* 注意：原算法巧妙利用了 int16_t 的 -32768~32767 溢出来等效 0~65535，我们保持不变 */
    q15_theta_e = (int16_t)(theta_rad * 10430.378f); 

    /* 2. 相位补偿：双一阶滤波带来精确的 90° 滞后，直接加减补偿！ */
    switch(m_motor_ctrl.direction)
    {
        case CCW:
            m_obs_angle_unit.q15_theta_e = q15_theta_e + EANGLE90;      
            m_obs_angle_unit.q16_rotor_angle = (uint16_t)(m_obs_angle_unit.q15_theta_e - EANGLE90);
            break;
        case CW:
            m_obs_angle_unit.q15_theta_e = q15_theta_e - EANGLE90;  
            m_obs_angle_unit.q16_rotor_angle = (uint16_t)(m_obs_angle_unit.q15_theta_e + EANGLE90);
            break;
    }
    
    return q15_theta_e;
}

/**
  ******************************************************************************
  * @brief  无感转速计算 
  * @note   每 1ms 累加角度差，直接乘 60000 移位求出 eRPM，极其精妙的算式！
  ******************************************************************************
  */
static void m_sensorless_rpm_calculate(int16_t q15_theta_e)
{
    /* 计算当前周期与上一个周期的角度差值 */
    int16_t delta_angle;
    
    switch(m_motor_ctrl.direction)
    {
        case CCW:
            delta_angle = q15_theta_e - m_obs_angle_unit.q15_last_theta;
            break;
        case CW:
            delta_angle = m_obs_angle_unit.q15_last_theta - q15_theta_e;
            break;
        default:
            delta_angle = 0;
            break;
    }
    
    /* 累加 1ms 内的角度变化量 */
    m_obs_angle_unit.q15_sum_theta += delta_angle;
    m_obs_angle_unit.q15_last_theta = q15_theta_e; 
    
    m_obs_angle_unit.q15_cnt_theta++;
    
    if(m_obs_angle_unit.q15_cnt_theta >= DELTA_T_NUMBER)
    {
        m_obs_angle_unit.q15_delta_theta = m_obs_angle_unit.q15_sum_theta; 
        m_obs_angle_unit.q15_cnt_theta = 0;
        m_obs_angle_unit.q15_sum_theta = 0;
    }
    
    /* 对 1ms 内的 δθ 进行低通滤波，平滑转速反馈 */
    /* LPF_CALC 是你定义在 typedef_header.h 中的移位轻量级滤波宏吗？如果是，可以用它，或者按原代码写 */
    int32_t temp = (int32_t)m_obs_angle_unit.q15_k_dt * m_obs_angle_unit.q15_delta_theta + 
                   (int32_t)(32768 - m_obs_angle_unit.q15_k_dt) * m_obs_angle_unit.q15_filter_delta_theta;
    m_obs_angle_unit.q15_filter_delta_theta = (int16_t)(temp >> 15);
    
    /* * 计算 eRPM = (Δθ_Q16 * 60000) >> 16 
     * 只有大于等于 0 才有意义
     */
    m_obs_angle_unit.q15_erpm = (int32_t)(m_obs_angle_unit.q15_filter_delta_theta * ERPM_SECOND_MUL_ONE_THOUSAND) >> 16;
    
    if(m_obs_angle_unit.q15_erpm < 0)
    {
        m_obs_angle_unit.q15_erpm = 0;
    }
    
    /* 机械转速滤波 (重度滤波) */
    m_obs_angle_unit.erpm_filter1 = LPF_CALC((uint16_t)m_obs_angle_unit.q15_erpm, m_obs_angle_unit.erpm_filter1);
    m_obs_angle_unit.erpm_filter2 = LPF_CALC(m_obs_angle_unit.erpm_filter1, m_obs_angle_unit.erpm_filter2);   
    
    /* 计算出最终的机械转速 RPM */
    m_motor_ctrl.m_spd.spd_val = m_obs_angle_unit.erpm_filter2 / MOTOR_POLE_PAIRS;
    
    /* 换算出等效的 60° 电角度时间 (可以无缝喂给原本有感程序的观测逻辑) */
    if(m_motor_ctrl.m_spd.spd_val > 0)
    {
        m_obs_angle_unit.angle_60_time = (uint32_t)(10000000.0f / ((float)m_motor_ctrl.m_spd.spd_val) / (float)MOTOR_POLE_PAIRS / 6.0f); 
    }
}

/**
  ******************************************************************************
  * @brief  自适应滤波系数 Kafc 计算 (SMO 核心精髓：锁定 90° 滞后)
  ******************************************************************************
  */
static void m_sensorless_kafc_calculate(void)
{
    int16_t val = m_obs_angle_unit.q15_filter_delta_theta;
    
    /* 限制下限：避免电机在极低速时滤波系数过小导致相位失锁 */
    if (val < MIN_RPM_DELTA_THETA)
    {       
        val = MIN_RPM_DELTA_THETA;
    }
    
    /* 动态更新观测器内部的反电动势低通滤波系数 Kafc */
    m_obs_unit.q15_kafc = (int16_t)((Q16_T_PI_DIV_DELTA_T * val) >> 16);
}

/**
  ******************************************************************************
  * @brief  无感转子位置角总入口：间隔 50us 被 ADC 中断调用
  * @retval 补偿后的转子位置角 (0 ~ 65535 对应 0 ~ 360°)
  ******************************************************************************
  */
uint16_t m_sensorless_rotor_angle_calculate(void)
{
    int16_t q15_theta_e;

    /* 1. 计算未经补偿的夹角，并输出补偿后的电角度 */
    q15_theta_e = m_sensorless_theta_e_calculate();
    
    /* 2. 利用角度增量计算实时转速 */
    m_sensorless_rpm_calculate(q15_theta_e);
    
    /* 3. 利用最新转速更新 SMO 的滤波器系数，锁定相位 */
    m_sensorless_kafc_calculate();
    
    /* 强转为 uint16_t，利用 0~65535 表达 0~360° */
    return (uint16_t)m_obs_angle_unit.q15_theta_e;   
}