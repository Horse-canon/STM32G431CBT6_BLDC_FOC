/** @addtogroup DRV
* @{
*/
#ifndef __DRV_HALL_CB_H__
#define __DRV_HALL_CB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 极简版霍尔捕获接口结构体 */
typedef struct
{
    void (*hall_capture_sign_clear_func)(void);
    void (*hall_capture_reset_func)(void);
    
    /* 只需要这一个统一的获取时间函数即可 */
    uint32_t (*hall_capture_val_func)(void); 
    
    /* 只需要这一个统一的跳变标记 */
    const bool *hall_sign; 
} hall_capture_unit_t;

extern hall_capture_unit_t hall_capture_unit;

#ifdef __cplusplus
}
#endif

#endif