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

typedef struct
{
    void (*hall_capture_sign_clear_func)(void);
    void (*hall_capture_reset_func)(void);
    uint32_t (*hall_u_capture_val_func)(void);
    uint32_t (*hall_v_capture_val_func)(void);
    uint32_t (*hall_w_capture_val_func)(void);
    
    const bool  *u_sign;
    const bool  *v_sign;
    const bool  *w_sign;
} hall_capture_unit_t;

extern hall_capture_unit_t hall_capture_unit;

#ifdef __cplusplus
}
#endif

#endif