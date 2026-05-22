/**
  ******************************************************************************
  * @file    m_observer.h
  * @author  JYH
  * @brief   无感FOC 滑模观测器 (SMO) 头文件
  ******************************************************************************
  */
#ifndef __M_OBSERVER_H__
#define __M_OBSERVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 替换为 STM32 平台和你的公共类型头文件 */
#include "main.h"
#include "typedef_header.h"

typedef struct
{
    int16_t q15_u_alpha_1;          //真实电机模型Uα(n-1)
    int16_t q15_u_beta_1;           //真实电机模型Uβ(n-1)
    
    int16_t q15_i_alpha;            //真实电机模型Iα，从Clark变换获取
    int16_t q15_i_beta;             //真实电机模型Iβ，从Clark变换获取
    
    int16_t q15_i_alpha_estimate;   //观测器估算Iα`电流值Iα`(n)
    int16_t q15_i_beta_estimate;    //观测器估算Iβ`电流值Iβ`(n)
    
    int16_t q15_i_alpha_estimate_1; //观测器估算Iα`电流值Iα`(n-1)
    int16_t q15_i_beta_estimate_1;  //观测器估算Iβ`电流值Iβ`(n-1)
    
    int16_t q15_z_alpha;            //校正值Zα(n)
    int16_t q15_z_beta;             //校正值Zβ(n)
    int16_t q15_z_alpha_1;          //校正值Zα(n-1)
    int16_t q15_z_beta_1;           //校正值Zβ(n-1)
    
    int16_t q15_z_s_max;            //误差最大值(开关面边界层厚度)
    int16_t q15_z_k_slide;          //滑膜增益K值
    
    int16_t q15_e_alpha;            //第1次低通滤波Eα`(n)
    int16_t q15_e_beta;             //第1次低通滤波Eβ`(n)
    int16_t q15_e_alpha_1;          //第1次低通滤波Eα`(n-1)
    int16_t q15_e_beta_1;           //第1次低通滤波Eβ`(n-1)
    
    int16_t q15_e_alpha_final;      //第2次低通滤波Eα_final(n)
    int16_t q15_e_beta_final;       //第2次低通滤波Eβ_final(n)
    int16_t q15_e_alpha_final_1;    //第2次低通滤波Eα_final(n-1)
    int16_t q15_e_beta_final_1;     //第2次低通滤波Eβ_final(n-1)
    
    int16_t q15_f_coefficient;      //电流估算方程中F值
    int16_t q15_g_coefficient;      //电流估算方程中G值
    
    int16_t q15_kafc;               //自适应滤波系数值 (0~32767)
    
    /* --- 新增：估算角度与转速 --- */
    uint16_t q16_theta_est;         // 观测器解算出的转子电角度 (0~65535)
    int16_t  speed_est_rpm;         // 观测器解算出的转速 (用于闭环切换)

} m_obs_unit_t;

extern m_obs_unit_t m_obs_unit;

void m_observer_init(void);
void m_observer_execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __M_OBSERVER_H__ */