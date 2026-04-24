#include "mcu_pwm.h"
#include "tim.h" 
#include <stdio.h>
#include "m_parameter.h"

/* 引入在 tim.c 中生成的 TIM1 句柄 */
extern TIM_HandleTypeDef htim1;

/* * 频率计算: 170MHz / (2 * 20KHz) = 4250 
 * 这里定义为宏，方便其他模块（如 FOC 算法）引用进行标幺化计算 
 */
#define PWM_PERIOD_VALUE       MCU_PWM_TIMER_ARR

/**
  ******************************************************************************
  * @brief  PWM 初始化与自举电容预充电
  * @note   使用 PWM Mode 2: 计数值 < CCR 为低，计数值 > CCR 为高。
  * 要实现 0% 占空比 (上管关，下管开，给自举电容充电)，需将 CCR 设为最大值。
  ******************************************************************************
  */
void drv_pwm_init(void)
{
    /* 1. 设置初始比较值为 ARR (4250)，在 Mode 2 下等效于占空比 0% */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_PERIOD_VALUE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_PERIOD_VALUE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_PERIOD_VALUE);

    /* 2. 开启三相主输出 (U, V, W 相的上管) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // /* 3. 开启三相互补输出 (U, V, W 相的下管) */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}


/**
  ******************************************************************************
  * @brief  实时设置三相 PWM 占空比 (后续 FOC 控制使用)
  ******************************************************************************
  */
void drv_pwm_set_duty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w)
{
    /* 安全限幅 */
    if (duty_u > PWM_PERIOD_VALUE) duty_u = PWM_PERIOD_VALUE;
    if (duty_v > PWM_PERIOD_VALUE) duty_v = PWM_PERIOD_VALUE;
    if (duty_w > PWM_PERIOD_VALUE) duty_w = PWM_PERIOD_VALUE;

    /* 更新比较寄存器 CCR */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_w);
}