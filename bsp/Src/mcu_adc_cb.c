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
#include "m_ctrl.h"
#include "m_coordinate.h"

/* 引入外部的ADC句柄，用于HAL库读取 */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

volatile uint32_t foc_loop_count = 0;

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
 * @brief  ADC 组0数据滤波 (STM32 移植版)
 * @note   包含启动阶段相电流偏置校准，以及母线电压的滑动平均滤波
 ******************************************************************************/
void drv_adc0_filter(void)
{
    /*初始阶段电流静态误差计算*/
    if(m_motor_ctrl.offset_current_sign == true)
    {
        /*累加和：
            Ia_static=静态误差 * 50 + 1.65V
            Ib_static=静态误差 * 10 + 1.65V
            Ic_static=静态误差 * 10 + 1.65V 
        */
        adc_unit.u_current.sum_value += adc_unit.u_current.instant_value;
        adc_unit.v_current.sum_value += adc_unit.v_current.instant_value;
        adc_unit.w_current.sum_value += adc_unit.w_current.instant_value;
        /*累加计数*/
        adc_unit.u_current.number_value++;
        adc_unit.v_current.number_value++;
        adc_unit.w_current.number_value++;
        /*静态误差校正时间到*/
        if(m_tick_unit.phase_current_offset_time == 0)
        {
            /*静态误差平均值*/
            adc_unit.u_current.average_value = adc_unit.u_current.sum_value / \
                                               adc_unit.u_current.number_value;
            adc_unit.v_current.average_value = adc_unit.v_current.sum_value / \
                                               adc_unit.v_current.number_value;
            adc_unit.w_current.average_value = adc_unit.w_current.sum_value / \
                                               adc_unit.w_current.number_value;
            
            adc_unit.u_current_offset = adc_unit.u_current.average_value;
            adc_unit.v_current_offset = adc_unit.v_current.average_value;
            adc_unit.w_current_offset = adc_unit.w_current.average_value;
            /*静态误差计算结束*/
            m_motor_ctrl.offset_current_sign = false;
        }
    }
    
    //母线电压 128次平均滤波处理
    adc_unit.bus_voltage.sum_value += adc_unit.bus_voltage.instant_value;
    adc_unit.bus_voltage.number_value++;
    if(adc_unit.bus_voltage.number_value >= 128)
    {
        adc_unit.bus_voltage.average_value = (uint16_t)(adc_unit.bus_voltage.sum_value >> 7);
        adc_unit.bus_voltage.sum_value = 0;
        adc_unit.bus_voltage.number_value = 0;
    } 
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
 * @brief  STM32 注入组转换完成中断回调 (高频 FOC 控制核心)
 ******************************************************************************
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* ADC1 和 ADC2 的注入组是硬件同步触发的，只需在一个句柄中处理 FOC 即可 */
    if (hadc->Instance == ADC1)
    {     
        /* 第1步：获取最新鲜的 ADC 采样数据 */
        drv_adc0_sample();
        
        /* 第2步：执行滤波与静态偏置误差校准 */
        drv_adc0_filter();  

        /* 第3步：Ia Ib Ic三相相电流计算 */
        m_phase_current_calculate();		       
        
        /* 第4步：电流Clark变换 */
        m_clark_transform();		

        /* 第5步：执行 FOC 核心控制算法（角度更新→速度环→Park→电流环→SVPWM） */
        m_foc_algorithm_execute();
        
        /* 第6步：系统时基滴答更新 */
        m_tick();

        /* 增加这行计数 */
        foc_loop_count++;
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