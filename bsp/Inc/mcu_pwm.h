/**
  ******************************************************************************
  * @file    mcu_pwm.h
  * @brief   电机三相 PWM 驱动封装（基于 TIM1）
  * @author  Jiang Yuhao (江雨豪)
  ******************************************************************************
  */

#ifndef __MCU_PWM_H__
#define __MCU_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"  // 包含 HAL 库及句柄定义
#include "m_parameter.h"

/* * 频率计算: 170MHz / (2 * 20KHz) = 4250 
 * 这里定义为宏，方便其他模块（如 FOC 算法）引用进行标幺化计算 
 */
#define PWM_PERIOD_VALUE       MCU_PWM_TIMER_ARR

/**
  * @brief  PWM 初始化与自举电容预充电
  * @note   配置 TIM1 产生中心对齐模式 1 的三相带互补输出 PWM。
  * 初始状态将下管全开，给预驱芯片 FD6288T 的自举电容充电。
  */
void drv_pwm_init(void);

/**
  * @brief  设置三相 PWM 占空比
  * @param  duty_u: U相比较值 (0 ~ 4250)
  * @param  duty_v: V相比较值 (0 ~ 4250)
  * @param  duty_w: W相比较值 (0 ~ 4250)
  * @note   配合 PWM Mode 2 使用：
  * - CCR = 0: 100% 占空比 (上管常开)
  * - CCR = 4250: 0% 占空比 (上管常关)
  */
void drv_pwm_set_duty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w);

#ifdef __cplusplus
}
#endif

#endif /* __MCU_PWM_H__ */