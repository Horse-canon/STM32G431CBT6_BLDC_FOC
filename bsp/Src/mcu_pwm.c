#include "mcu_pwm.h"
#include "tim.h" 
#include <stdio.h>


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

    /* 4. 延时给自举电容充电，FD6288T 通常 5~10ms 即可 */
    HAL_Delay(10); 
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