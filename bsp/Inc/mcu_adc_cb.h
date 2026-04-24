/**
  ******************************************************************************
  * @file    mcu_adc.h
  * @author  jiangyuhao
  * @version V1.2
  * @brief   adc回调头文件 (精简版：仅包含实际使用的5个参数)
  ******************************************************************************
  */
#ifndef __MCU_ADC_H__
#define __MCU_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 单个通道的数据结构保持不变 */
typedef struct
{
    uint16_t instant_value;     //瞬时值
    uint16_t average_value;     //平均值
    uint32_t sum_value;         //累加和值
    uint16_t number_value;      //计数值
    uint16_t filter_value;      //滤波值
} adcx_ch_type;

/* 核心 ADC 结构体 */
typedef struct
{
    adcx_ch_type u_current;     // U相相电流 (源自 ADC1 JRank1)
    adcx_ch_type v_current;     // V相相电流 (源自 ADC1 JRank2)
    adcx_ch_type w_current;     // W相相电流 (源自 ADC1 JRank3)
    adcx_ch_type bus_voltage;   // 母线电压   (源自 ADC2 JRank1)
    adcx_ch_type spd_voltage;   // 调速电位器 (源自 ADC2 Rank1)
    
    // 如果你在其他地方用到了下面这几个偏置量，可以保留，否则也可以删掉
    uint16_t u_current_offset;
    uint16_t v_current_offset;
    uint16_t w_current_offset;
    
    bool adc0_cb_sign;          // 对应原ADC0完成标志(现为注入组完成标志)
} adc_unit_t;

extern adc_unit_t adc_unit;

/* 暴露给外部的回调处理函数 */
void drv_adc0_sample(void);
void drv_adc1_sample(void);
void drv_adc1_filter(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ADC_CB_H__ */