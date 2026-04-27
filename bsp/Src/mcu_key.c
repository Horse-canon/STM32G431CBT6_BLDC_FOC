/** @addtogroup DRV
* @{
*/
#include "mcu_key.h"
#include <stdio.h>

#define FILTER_KEEP_TIME   (10)  // 10ms 消抖时间

/* * 硬件引脚映射 (根据你之前提供的原理图) 
 * SW2: M_PB11 -> CW/CCW
 * SW3: M_PB12 -> START/STOP
 */
#define CW_CCW_PORT        GPIOB
#define CW_CCW_PIN         GPIO_PIN_11

#define ST_SP_PORT         GPIOB
#define ST_SP_PIN          GPIO_PIN_12

key_parameter_t key_st_sp_parameter;
key_parameter_t key_cw_ccw_parameter;

/**
 ******************************************************************************
 * @brief  key初始化: 空函数 (因为引脚初始化已在 CubeMX 的 MX_GPIO_Init 中完成)
 * @param  None.
 * @retval None.
 ******************************************************************************/
void drv_key_init(void)
{
    /* 确保初始状态为清零状态 */
    key_st_sp_parameter.trigger = 0;
    key_st_sp_parameter.down_sign = 0;
    
    key_cw_ccw_parameter.trigger = 0;
    key_cw_ccw_parameter.down_sign = 0;
}

/**
 ******************************************************************************
 * @brief  按键扫描 (非阻塞状态机，需周期性调用)
 * @param  None.
 * @retval None.
 ******************************************************************************/
void drv_key_scan(void)
{
    uint8_t key_val = 0;
    
    /* =========================================================================
       第 1 部分：START/STOP 按键检测 (PB12)
       ========================================================================= */
    /*第1步：start/stop按键检测，判断是否按下*/
    key_val = (uint8_t)HAL_GPIO_ReadPin(ST_SP_PORT, ST_SP_PIN);
    
    if((key_val == KEY_DOWN_VALUE ) && (key_st_sp_parameter.trigger == 0))
    {
        key_st_sp_parameter.trigger = 0x01;                 //触发器状态更新为0x01
        key_st_sp_parameter.time = HAL_GetTick();           //获取当前毫秒级时间戳
    }
    
    /*第2步：进行按键软件消抖滤波*/
    if(key_st_sp_parameter.trigger == 0x01)
    {
        /*从按键按下开始计时 10ms*/
        if(HAL_GetTick() - key_st_sp_parameter.time > FILTER_KEEP_TIME)
        {
            /*10ms时间到：再次读取按键值是否是按下状态*/
            key_val = (uint8_t)HAL_GPIO_ReadPin(ST_SP_PORT, ST_SP_PIN);
            if(key_val == KEY_DOWN_VALUE)
            {
                key_st_sp_parameter.down_sign = 1;      //按下状态：将按键按下标记设置为1
                printf("Start/Stop Key Down\r\n");      //信息打印 (需重定向 printf)
            }
            key_st_sp_parameter.trigger = 0x02;         //触发器状态更新为0x02 (等待抬起)
        }
    }
    
    /*第3步：按键是抬起状态：将触发器值更新为0，为下一次按键动作做准备*/
    /* 注意这里需实时读取引脚状态来判断是否抬起 */
    key_val = (uint8_t)HAL_GPIO_ReadPin(ST_SP_PORT, ST_SP_PIN);
    if((key_st_sp_parameter.trigger == 0x02) && (key_val == KEY_UP_VALUE))
    {
        key_st_sp_parameter.trigger = 0;
    }
    
    
    /* =========================================================================
       第 2 部分：CW/CCW 按键检测 (PB11)
       ========================================================================= */
    /*第1步：cw/ccw按键检测，判断是否按下*/
    key_val = (uint8_t)HAL_GPIO_ReadPin(CW_CCW_PORT, CW_CCW_PIN);
    
    if((key_val == KEY_DOWN_VALUE ) && (key_cw_ccw_parameter.trigger == 0))
    {
        key_cw_ccw_parameter.trigger = 0x01;                 //触发器状态更新为0x01
        key_cw_ccw_parameter.time = HAL_GetTick();           //时间记录
    }
    
    /*第2步：进行按键软件消抖滤波*/
    if(key_cw_ccw_parameter.trigger == 0x01)
    {
        /*从按键按下开始计时 10ms*/
        if(HAL_GetTick() - key_cw_ccw_parameter.time > FILTER_KEEP_TIME)
        {
            /*10ms时间到：再次读取按键值是否是按下状态*/
            key_val = (uint8_t)HAL_GPIO_ReadPin(CW_CCW_PORT, CW_CCW_PIN);
            if(key_val == KEY_DOWN_VALUE)
            {
                key_cw_ccw_parameter.down_sign = 1; //按下状态：将按键按下标记设置为1
                printf("CW/CCW Key Down\r\n");      //信息打印
            }
            key_cw_ccw_parameter.trigger = 0x02;    //触发器状态更新为0x02
        }
    }
    
    /*第3步：按键是抬起状态：将触发器值更新为0，为下一次按键动作做准备*/
    key_val = (uint8_t)HAL_GPIO_ReadPin(CW_CCW_PORT, CW_CCW_PIN);
    if((key_cw_ccw_parameter.trigger == 0x02) && (key_val == KEY_UP_VALUE))
    {
        key_cw_ccw_parameter.trigger = 0;
    }
}
/** @} */