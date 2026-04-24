/** @addtogroup DRV
* @{
*/
#include "mcu_hall.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim3;

/**
  ******************************************************************************
  * @brief  hall初始化 (STM32 专属霍尔模式)
  * @param  None.
  * @retval None.
  ******************************************************************************
  */
void drv_hall_init(void)
{
    /* 启动 TIM3 霍尔传感器接口，并开启输入捕获中断 */
    /* 硬件会自动处理三相异或、锁存时间差、并自动复位计数器 */
    HAL_TIMEx_HallSensor_Start_IT(&htim3);
}