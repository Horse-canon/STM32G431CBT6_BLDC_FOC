/** @addtogroup MOTOR
* @{
*/
#ifndef __M__ROTOR_ANGLE_H__
#define __M__ROTOR_ANGLE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 移除瑞萨的 hal_data.h，替换为 STM32 的 main.h */
#include "main.h"
#include "typedef_header.h"

typedef struct
{
    uint8_t u_val;
    uint8_t v_val;
    uint8_t w_val;
    
    uint8_t value;
    uint8_t value_last;
    bool update_sign;
    uint16_t update_cnt;
    uint8_t start_cnt;
    bool start_sign;
    uint32_t time;
    uint32_t time_last;
    
    uint32_t angle_60_time;
    uint32_t angle_60_time_filter1;
    uint32_t angle_60_time_filter2; 
    
    uint8_t hall_val_test_buf[6];
    uint8_t hall_val_test_index;
} m_hall_unit_t;

extern m_hall_unit_t m_hall_unit;

void m_rotor_angle_init(void);
uint16_t m_rotor_angle_calculate(void);

#ifdef __cplusplus
}
#endif

#endif
/**
  * @}
  */