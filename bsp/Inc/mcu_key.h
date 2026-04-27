/** @addtogroup DRV
* @{
*/
#ifndef __DRV_KEY_H__
#define __DRV_KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 引入 STM32 HAL 库核心头文件 */
#include "main.h"

#define KEY_DOWN_VALUE    0x00     //按键按下键值 (低电平有效)
#define KEY_UP_VALUE      0x01     //按键抬起键值 (高电平)

typedef enum
{
    START_STOP_KEY = 0,
    CW_CCW_KEY,
    KEY_MAX
} key_num_e;

typedef struct
{
    uint32_t time;        //时间记录
    uint8_t  trigger;     //触发标记 (状态机)
    uint8_t  down_sign;   //按下标记 (供外部调用的标志位)
    uint32_t key_down_cnt;//按键按下计数
} key_parameter_t;

extern key_parameter_t key_st_sp_parameter;
extern key_parameter_t key_cw_ccw_parameter;

void drv_key_init(void);
void drv_key_scan(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_KEY_H__ */
/** @} */