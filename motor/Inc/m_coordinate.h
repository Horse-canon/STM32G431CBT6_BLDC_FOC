/**
 ******************************************************************************
 * @file    m_coordinate.h
 * @author  chengbb (Ported to STM32G4)
 * @brief   FOC 坐标变换头文件
 ******************************************************************************
 */
#ifndef __M_COORDINATE_H__
#define __M_COORDINATE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 引入 STM32 HAL 库核心头文件 */
#include "main.h"

void m_phase_current_calculate(void);
void m_clark_transform(void);
void m_park_transform(uint16_t rotor_engle);
void m_inverse_clark_transform(void);
void m_inverse_park_transform(uint16_t rotor_engle);
void m_us_theta_c_calculate(void);

#ifdef __cplusplus
}
#endif

#endif /* __M_COORDINATE_H__ */