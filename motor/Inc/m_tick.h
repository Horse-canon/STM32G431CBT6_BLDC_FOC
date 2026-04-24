#ifndef __M__TICK_H__
#define __M__TICK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h" 

typedef struct
{
    uint16_t boot_charge_time;
    uint16_t spd_time;
} m_tick_unit_t;

extern m_tick_unit_t m_tick_unit;

void m_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __M__TICK_H__ */