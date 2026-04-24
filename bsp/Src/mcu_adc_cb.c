/**
  ******************************************************************************
  * @file    drv_adc_cb.c
  * @author  chengbb (Ported & Optimized for STM32G4)
  * @version V1.2
  * @brief   adc回调及数据处理 (精简版)
  ******************************************************************************
  */
#include "mcu_adc_cb.h"
#include <stdio.h>
#include "m_tick.h"
#include "typedef_header.h"
#include "m_foc.h"

/* 引入外部的ADC句柄，用于HAL库读取 */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

adc_unit_t adc_unit;

/**
  ******************************************************************************
  * @brief  注入组数据采样 (替代原 adc0组：相电流 + 母线电压)
  ******************************************************************************
  */
void drv_adc0_sample(void)
{
    /* 1. 读取 ADC1 注入组：U、V、W相电流 */
    adc_unit.u_current.instant_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    adc_unit.v_current.instant_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    adc_unit.w_current.instant_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    
    /* 2. 读取 ADC2 注入组：母线电压 */
    adc_unit.bus_voltage.instant_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    
    adc_unit.adc0_cb_sign = true;
}

/**
  ******************************************************************************
  * @brief  规则组数据采样 (替代原 adc1组：调速电位器)
  ******************************************************************************
  */
void drv_adc1_sample(void)
{
    /* 读取 ADC2 规则组：调速旋钮电压 */
    adc_unit.spd_voltage.instant_value = HAL_ADC_GetValue(&hadc2); 
}

/**
  ******************************************************************************
  * @brief  调速电位器数据滤波 (保持原有的16次均值低通滤波逻辑)
  ******************************************************************************
  */
void drv_adc1_filter(void)
{
    adc_unit.spd_voltage.sum_value += adc_unit.spd_voltage.instant_value;
    adc_unit.spd_voltage.number_value++;
    
    if(adc_unit.spd_voltage.number_value >= 16)
    {
        adc_unit.spd_voltage.average_value = (uint16_t)(adc_unit.spd_voltage.sum_value >> 4); // 除以16
        adc_unit.spd_voltage.sum_value = 0;
        adc_unit.spd_voltage.number_value = 0;
        
        adc_unit.spd_voltage.filter_value = LPF_CALC(adc_unit.spd_voltage.average_value, \
                                                     adc_unit.spd_voltage.filter_value);
    } 
}

/**
  ******************************************************************************
  * @brief  STM32 注入组转换完成中断回调 
  ******************************************************************************
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* ADC1 和 ADC2 的注入组是硬件同步触发的，只需在一个句柄中处理 FOC 即可 */
    if (hadc->Instance == ADC1)
    {     
        m_foc_algorithm_execute();
        drv_adc0_sample();
        m_tick();
    }
}

/**
  ******************************************************************************
  * @brief  STM32 规则组转换完成中断回调 
  ******************************************************************************
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 只有 ADC2 配置了规则组用于旋钮采样 */
    if (hadc->Instance == ADC2)
    {
        drv_adc1_sample();
        drv_adc1_filter();
    }
}