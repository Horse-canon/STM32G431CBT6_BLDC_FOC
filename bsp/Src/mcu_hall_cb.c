/** @addtogroup DRV
* @{
*/
#include "mcu_hall_cb.h"
#include <stdio.h>
#include <string.h>

/* 移除不需要的 th1, th2, trg_sign 变量，因为STM32硬件自动处理了 */
typedef struct
{
    uint32_t u_capture_val;
    uint32_t v_capture_val;
    uint32_t w_capture_val;
    
    bool  u_sign;
    bool  v_sign;
    bool  w_sign;
} drv_hall_capture_t;

drv_hall_capture_t drv_hall_capture;

static void drv_hall_capture_sign_clear(void);
static void drv_hall_capture_reset(void);
static uint32_t drv_hall_u_capture_value(void);
static uint32_t drv_hall_v_capture_value(void);
static uint32_t drv_hall_w_capture_value(void);

hall_capture_unit_t hall_capture_unit = 
{
    .hall_capture_sign_clear_func = drv_hall_capture_sign_clear,
    .hall_capture_reset_func = drv_hall_capture_reset,
    .hall_u_capture_val_func = drv_hall_u_capture_value,
    .hall_v_capture_val_func = drv_hall_v_capture_value,
    .hall_w_capture_val_func = drv_hall_w_capture_value,
    .u_sign = &drv_hall_capture.u_sign,
    .v_sign = &drv_hall_capture.v_sign,
    .w_sign = &drv_hall_capture.w_sign,
};

/**
  ******************************************************************************
  * @brief  霍尔捕获标记清除
  ******************************************************************************
  */
static void drv_hall_capture_sign_clear(void)
{
    drv_hall_capture.u_sign = false;
    drv_hall_capture.v_sign = false;
    drv_hall_capture.w_sign = false;
}

/**
  ******************************************************************************
  * @brief  霍尔捕获参数复位
  ******************************************************************************
  */
static void drv_hall_capture_reset(void)
{
    memset(&drv_hall_capture, 0, sizeof(drv_hall_capture));
}

static uint32_t drv_hall_u_capture_value(void) { return drv_hall_capture.u_capture_val; }
static uint32_t drv_hall_v_capture_value(void) { return drv_hall_capture.v_capture_val; }
static uint32_t drv_hall_w_capture_value(void) { return drv_hall_capture.w_capture_val; }

/**
  ******************************************************************************
  * @brief  STM32 统一输入捕获中断回调 (替代原来的 g_timer4 和 g_timer5)
  * @param  htim: 定时器句柄
  * @retval None
  ******************************************************************************
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* 检测是否为 TIM3 (霍尔定时器) 的通道1 (硬件异或综合通道) 触发 */
    if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        /* 1. 直接读取硬件算好的时间差 (单位: us)。
           注意：这是转过 60° 电角度的时间！
        */
        uint32_t capture_60_deg = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        drv_hall_capture.u_capture_val = capture_60_deg;
        drv_hall_capture.v_capture_val = capture_60_deg;
        drv_hall_capture.w_capture_val = capture_60_deg;
        
        drv_hall_capture.u_sign = true;
        drv_hall_capture.v_sign = true;
        drv_hall_capture.w_sign = true;
    }
}