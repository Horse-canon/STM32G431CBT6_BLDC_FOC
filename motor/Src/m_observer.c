/**
  ******************************************************************************
  * @file    m_observer.c
  * @author  STM32G4 Ported
  * @brief   无感FOC 滑模观测器 (SMO) 源文件
  ******************************************************************************
  */
#include "m_observer.h"
#include <stdio.h>
#include <math.h>            // 引入数学库，使用 STM32G4 FPU
#include "m_parameter.h"
#include "m_foc.h"
#include "m_ctrl.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

m_obs_unit_t m_obs_unit;

/**
  ******************************************************************************
  * @brief  Q15 数字一阶低通滤波器
  * @note   y(n) = Kafc * x(n) + (1 - Kafc) * y(n-1)
  * Kafc 取值 0~32767，对应 0~1.0
  ******************************************************************************
  */
static inline int16_t math_q15_digital_lpf(int16_t q15_kafc, int16_t xn, int16_t yn_1)
{
    int32_t temp;
    temp = (int32_t)q15_kafc * xn + (int32_t)(32768 - q15_kafc) * yn_1;
    return (int16_t)(temp >> 15);
}

/**
  ******************************************************************************
  * @brief  观测器初始化
  ******************************************************************************
  */
void m_observer_init(void)
{
    /*F = 1 - Ts * R / L*/
    m_obs_unit.q15_f_coefficient = F_COEFF;  
    /*G = Ts / L*/  
    m_obs_unit.q15_g_coefficient = G_COEFF;  
    
    m_obs_unit.q15_z_s_max = OBSERVER_S_MAX_VALUE;
    m_obs_unit.q15_z_k_slide = OBSERVER_K_SLIDE;
}

/**
  ******************************************************************************
  * @brief  观测器电流估算方程电流估算
  ******************************************************************************
  */
int16_t m_observer_estimate_current(int16_t q15_f, int16_t q15_g, int16_t q15_in_1, int16_t q15_un_1, int16_t q15_zn_1, int16_t q15_en_1)
{
    int32_t temp;
    /*Q30*/
    temp = (int32_t)(q15_f * q15_in_1) + \
           (int32_t)(q15_g * q15_un_1) - \
           (int32_t)(q15_g * q15_zn_1) - \
           (int32_t)(q15_g * q15_en_1);
    /*Q15*/
    return (int16_t)(temp >> 15); 
}

/**
  ******************************************************************************
  * @brief  观测器校正值Z计算 (滑模Bang-Bang + 线性区)
  ******************************************************************************
  */
int16_t m_observer_z_calculate(int16_t q15_i_estimate, int16_t q15_i, int16_t q15_z_s_max, int16_t q15_z_k_slide)
{
    int16_t q15_s; //误差值s
    int16_t z_val; //校正值z
    
    q15_s = q15_i_estimate - q15_i;
    
    if(q15_s > q15_z_s_max)
    {
        z_val = q15_z_k_slide;  //+K
    }
    else if(q15_s < -q15_z_s_max)
    {
        z_val = -q15_z_k_slide; //-K
    }
    else
    {
        /*线性区:线性输出*/
        z_val = (int16_t)((int32_t)(q15_z_k_slide * q15_s) / q15_z_s_max);
    }
    
    return z_val;
}

/**
  ******************************************************************************
  * @brief  观测器执行核心逻辑
  ******************************************************************************
  */
void m_observer_execute(void)
{
    /* 获取当前周期的相电流 (Iα, Iβ) */
    m_obs_unit.q15_i_alpha = m_foc_unit.coordinate.q15_i_alpha; //Iα(n)
    m_obs_unit.q15_i_beta  = m_foc_unit.coordinate.q15_i_beta;  //Iβ(n)
    
    /* * 获取上一周期的电压 Uα(n-1), Uβ(n-1)
     * 注意：由于在ADC中断中，观测器应该在最新一轮的电压计算之前执行，
     * 所以此时 m_foc_unit.coordinate.q15_u_alpha 里存的就是上一周期的输出！
     */
    m_obs_unit.q15_u_alpha_1 = m_foc_unit.coordinate.q15_u_alpha; //Uα(n-1)
    m_obs_unit.q15_u_beta_1  = m_foc_unit.coordinate.q15_u_beta;  //Uβ(n-1)
    
    /*电流估算方程电流估算*/
    m_obs_unit.q15_i_alpha_estimate = m_observer_estimate_current(       
                                        m_obs_unit.q15_f_coefficient,    
                                        m_obs_unit.q15_g_coefficient,    
                                        m_obs_unit.q15_i_alpha_estimate_1, 
                                        m_obs_unit.q15_u_alpha_1,          
                                        m_obs_unit.q15_z_alpha_1,          
                                        m_obs_unit.q15_e_alpha_1);         
    
    m_obs_unit.q15_i_beta_estimate  = m_observer_estimate_current(       
                                        m_obs_unit.q15_f_coefficient,    
                                        m_obs_unit.q15_g_coefficient,    
                                        m_obs_unit.q15_i_beta_estimate_1,
                                        m_obs_unit.q15_u_beta_1,         
                                        m_obs_unit.q15_z_beta_1,         
                                        m_obs_unit.q15_e_beta_1);        
    
    /*滑膜校正Zα / Zβ值*/
    m_obs_unit.q15_z_alpha = m_observer_z_calculate(
                                m_obs_unit.q15_i_alpha_estimate,         
                                m_obs_unit.q15_i_alpha,                  
                                m_obs_unit.q15_z_s_max,                  
                                m_obs_unit.q15_z_k_slide);               
                                
    m_obs_unit.q15_z_beta  = m_observer_z_calculate(
                                m_obs_unit.q15_i_beta_estimate,          
                                m_obs_unit.q15_i_beta,                   
                                m_obs_unit.q15_z_s_max,                  
                                m_obs_unit.q15_z_k_slide);               
                                
    /*第1次滤波：反电动势滤波*/
    m_obs_unit.q15_e_alpha = math_q15_digital_lpf(m_obs_unit.q15_kafc, m_obs_unit.q15_z_alpha, m_obs_unit.q15_e_alpha_1);
    m_obs_unit.q15_e_beta  = math_q15_digital_lpf(m_obs_unit.q15_kafc, m_obs_unit.q15_z_beta,  m_obs_unit.q15_e_beta_1);
                                                  
    /*第2次滤波：反电动势滤波*/
    m_obs_unit.q15_e_alpha_final = math_q15_digital_lpf(m_obs_unit.q15_kafc, m_obs_unit.q15_e_alpha, m_obs_unit.q15_e_alpha_final_1);
    m_obs_unit.q15_e_beta_final  = math_q15_digital_lpf(m_obs_unit.q15_kafc, m_obs_unit.q15_e_beta,  m_obs_unit.q15_e_beta_final_1);                                     
    
    /*上一次值(n-1)更新*/
    m_obs_unit.q15_i_alpha_estimate_1 = m_obs_unit.q15_i_alpha_estimate;
    m_obs_unit.q15_i_beta_estimate_1  = m_obs_unit.q15_i_beta_estimate;
    
    m_obs_unit.q15_e_alpha_1 = m_obs_unit.q15_e_alpha;
    m_obs_unit.q15_e_beta_1  = m_obs_unit.q15_e_beta;
    
    m_obs_unit.q15_z_alpha_1 = m_obs_unit.q15_z_alpha;
    m_obs_unit.q15_z_beta_1  = m_obs_unit.q15_z_beta;
    
    m_obs_unit.q15_e_alpha_final_1 = m_obs_unit.q15_e_alpha_final;
    m_obs_unit.q15_e_beta_final_1  = m_obs_unit.q15_e_beta_final;
}