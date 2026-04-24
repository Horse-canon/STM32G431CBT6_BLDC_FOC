/** @addtogroup DRV
* @{
*/
#include "mcu_adc.h"
#include <stdio.h>

/* 引入在 adc.c 中生成的 ADC 句柄 */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/**
  ******************************************************************************
  * @brief  adc初始化 (STM32版)
  * @param  None.
  * @retval None.
  ******************************************************************************
  */
void drv_adc_init(void)
{
    /* 1. ADC 运行前校准 (STM32G4必备，大幅降低零点漂移) */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 2. 启动 ADC2 规则组 (带中断)，负责缓慢读取调速电位器 */
    HAL_ADC_Start_IT(&hadc2);

    /* 3. 启动 ADC1 和 ADC2 注入组 (带中断)，并在硬件层面挂起等待 TIM1 下溢触发 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
}
/**
  * @}
  */