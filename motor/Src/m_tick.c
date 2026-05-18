#include "m_tick.h"

m_tick_unit_t m_tick_unit;

/**
  ******************************************************************************
  * @brief  motor ctrl tick
  * @param  None.
  * @retval None.
  ******************************************************************************/
void m_tick(void)
{
    if(m_tick_unit.boot_charge_time != 0) {
        m_tick_unit.boot_charge_time--;
    }
    if(m_tick_unit.spd_time != 0) {
        m_tick_unit.spd_time--;
    }
	if(m_tick_unit.phase_current_offset_time != 0) {
        m_tick_unit.phase_current_offset_time--;
    }
	if(m_tick_unit.spd_pid_cycle_time != 0)	{
        m_tick_unit.spd_pid_cycle_time--;
    }
	if(m_tick_unit.current_loop_test_time != 0)	{
        m_tick_unit.current_loop_test_time--;
    }
}