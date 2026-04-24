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

void m_rotor_angle_init(void);
uint16_t m_rotor_angle_calculate(void);

#ifdef __cplusplus
}
#endif

#endif
/**
  * @}
  */