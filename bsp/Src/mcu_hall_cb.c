/** @addtogroup DRV
* @{
*/
#include "mcu_hall_cb.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
    uint32_t hall_capture_val; // 记录捕获到的定时器计数值（即 60° 电角度的时间差 Δt）
    bool     hall_sign;        // 捕获更新标志位：true 表示发生了新的霍尔跳变
} drv_hall_capture_t;

/* 实例化内部捕获变量 */
drv_hall_capture_t drv_hall_capture;

static void drv_hall_capture_sign_clear(void);
static void drv_hall_capture_reset(void);
static uint32_t drv_hall_capture_value(void);

/* 接口函数实例化 */
hall_capture_unit_t hall_capture_unit = 
{
    .hall_capture_sign_clear_func = drv_hall_capture_sign_clear,
    .hall_capture_reset_func      = drv_hall_capture_reset,
    .hall_capture_val_func        = drv_hall_capture_value,
    .hall_sign                    = &drv_hall_capture.hall_sign,
};

/**
 * @brief 清除霍尔跳变标志位
 * @note  上层算法读取完跳变时间后，调用此函数清除标记，等待下一次跳变
 */
static void drv_hall_capture_sign_clear(void)
{
    drv_hall_capture.hall_sign = false;
}

/**
 * @brief 复位霍尔捕获参数
 * @note  通常在电机停机、初始化或发生堵转故障恢复时调用，清空历史数据
 */
static void drv_hall_capture_reset(void)
{
    memset(&drv_hall_capture, 0, sizeof(drv_hall_capture));
}

/**
 * @brief 获取缓存的霍尔捕获时间
 * @retval 60°电角度对应的定时器计数值 (Δt)
 */
static uint32_t drv_hall_capture_value(void) 
{ 
    return drv_hall_capture.hall_capture_val; 
}

/**
 ******************************************************************************
 * @brief  STM32 统一输入捕获中断回调
 * @param  htim: 定时器句柄
 ******************************************************************************
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* 检测是否为 TIM3 (霍尔定时器) 的通道1 (硬件异或综合通道) 触发 */
    if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        /* 直接读取硬件算好的时间差 (转过 60° 电角度的时间，单位: 1/f_timer) */
        drv_hall_capture.hall_capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        drv_hall_capture.hall_sign = true;
    }
}
/** @} */