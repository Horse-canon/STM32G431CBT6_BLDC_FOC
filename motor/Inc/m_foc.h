#ifndef __M__FOC_H__
#define __M__FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{   
    uint16_t rotor_engle;
    uint16_t q_engle;   
} m_foc_unit_t;

extern m_foc_unit_t m_foc_unit;

void m_us_radius_calculate(void);
void m_foc_algorithm_execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __M__FOC_H__ */