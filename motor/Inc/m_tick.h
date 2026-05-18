#ifndef __M__TICK_H__
#define __M__TICK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h" 

typedef struct
{
    uint16_t boot_charge_time; //自举电容充电时间
    uint16_t spd_time;         //调速电位器逻辑执行时间
    uint16_t phase_current_offset_time;	//相电流偏移采样时间
	uint16_t spd_pid_cycle_time;		//速度环执行周期时间
} m_tick_unit_t;
extern m_tick_unit_t m_tick_unit;

void m_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __M__TICK_H__ */