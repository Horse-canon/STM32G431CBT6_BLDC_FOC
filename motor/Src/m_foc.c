/**
 ******************************************************************************
 * @file    m_foc.c
 * @author  chengbb (Ported to STM32G4)
 * @version V1.0
 * @date    2025-01-10
 * @brief   motor foc
 ******************************************************************************
 */
/** @addtogroup MOTOR
* @{
*/
#include "m_foc.h"
#include <stdio.h>
#include "m_parameter.h"
#include "m_svpwm.h"
#include "m_tick.h"
#include "m_rotor_angle.h"
/* [STM32 移植] 替换为适配后的 STM32 ADC 头文件 */
#include "mcu_adc_cb.h" 
#include "m_ctrl.h"
#include "m_pid.h"
#include "m_coordinate.h"
//#include "m_monitor.h"
#include "mcu_key.h"

m_foc_unit_t m_foc_unit;

/**
 ******************************************************************************
 * @brief  Us模长计算
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_us_radius_calculate(void)
{
	static uint8_t dir_change_sign = false;
	
	/*检测到按键按下*/
	if(key_cw_ccw_parameter.down_sign)
	{
		key_cw_ccw_parameter.down_sign = 0;
		/*停机状态下直接进行电机方向切换*/
		if(m_motor_ctrl.state_machine <= EXECUTE_MOTOR_STOP)
		{
			if(m_motor_ctrl.direction == CCW)
			{
				m_motor_ctrl.direction = CW;
			}
			else if(m_motor_ctrl.direction == CW)
			{
				m_motor_ctrl.direction = CCW;
			}
		}
		else
		{
			dir_change_sign = true;
		}
	}
	/*电机切换旋转方向中*/
	if(dir_change_sign == true)
	{
		/*直接将spd转速强制更新为0*/
		m_motor_ctrl.q16_spd_val = 0;
		
		/*转速低于400rpm，直接停机，然后进行方向切换*/
		if(m_motor_ctrl.m_spd.spd_val < 400)
		{
			m_motor_stop();
			if(m_motor_ctrl.direction == CCW)
			{
				m_motor_ctrl.direction = CW;
			}
			else if(m_motor_ctrl.direction == CW)
			{
				m_motor_ctrl.direction = CCW;
			}
			dir_change_sign = false;  //切换完成，进行正常速度调节逻辑
		}
	}

	if(dir_change_sign == false)
	{
		uint16_t q16_adc_val = 0;
		uint16_t q16_spd_val = 0;
		
		/*ADC滤波值：左移4位转换为0.16格式数据*/
		q16_adc_val = (uint16_t)(adc_unit.spd_voltage.filter_value << 4);
		/*
			q16_adc_val:Q16
			MOTOR_MAX_SPEED:Q16
			(q16_adc_val * MOTOR_MAX_SPEED):Q32
			(q16_adc_val * MOTOR_MAX_SPEED) >> 16:Q16
		*/
		q16_spd_val = (q16_adc_val * MOTOR_MAX_SPEED) >> 16;  //Q16格式 0-2300rpm
		
		/*转速限幅*/
		q16_spd_val = (q16_spd_val < MOTOR_MIN_SPEED)?0:q16_spd_val;
		q16_spd_val = (q16_spd_val > MOTOR_MAX_SPEED)?MOTOR_MAX_SPEED:q16_spd_val;
			
		m_motor_ctrl.q16_spd_val = q16_spd_val;
	}
}

int16_t Iq_target = 512;     
/**
 ******************************************************************************
 * @brief  电流环PID执行
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_current_pid_execute(void)
{
	/*Id电流环PID*/
	m_id_pid_unit.q15_target_value = 0;								//Id目标值固定为0
	m_id_pid_unit.q15_actual_value = m_foc_unit.coordinate.q15_id;	//更新实时Id
	/*Id电流环PID计算结果Ud：串联型PID*/
	m_foc_unit.coordinate.q15_ud = m_series_pid_algorithm(&m_id_pid_unit);
	//m_foc_unit.coordinate.q15_ud = m_parallel_incremental_pid_algorithm(&m_id_pid_unit);

	m_iq_pid_unit.q15_actual_value = m_foc_unit.coordinate.q15_iq;//更新实时Iq
	/*Iq电流环PID计算结果Uq：串联型PID*/
	m_foc_unit.coordinate.q15_uq = m_series_pid_algorithm(&m_iq_pid_unit);
	//m_foc_unit.coordinate.q15_uq = m_parallel_incremental_pid_algorithm(&m_iq_pid_unit);
}

uint32_t loop_cnt;
/**
 ******************************************************************************
 * @brief  速度环PID执行
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_spd_pid_execute(void)
{
	/*获取到稳定实时转速*/
	if(m_motor_ctrl.m_spd.stabilize_sign == true)
	{
		/*速度环执行周期判断*/
		if(m_tick_unit.spd_pid_cycle_time == 0)
		{
			uint16_t val;
			
			m_tick_unit.spd_pid_cycle_time = SPD_PID_CYCLE_TIME;  //200ms
			
			/*目标设定值 > 实时设定值*/
			if(m_motor_ctrl.q16_spd_val > m_motor_ctrl.m_spd.set_spd_val)
			{
				val = m_motor_ctrl.q16_spd_val - m_motor_ctrl.m_spd.set_spd_val;
				/*转速斜坡递增*/
				if(val > INC_SPD_RPM)
				{
					m_motor_ctrl.m_spd.set_spd_val += INC_SPD_RPM;
				}
				else
				{
					m_motor_ctrl.m_spd.set_spd_val = m_motor_ctrl.q16_spd_val;
				}
			}
			/*目标设定值 < 实时设定值*/
			if(m_motor_ctrl.q16_spd_val < m_motor_ctrl.m_spd.set_spd_val)
			{
				val = m_motor_ctrl.m_spd.set_spd_val - m_motor_ctrl.q16_spd_val;
				/*转速斜坡递减*/
				if(val > DEC_SPD_RPM)
				{
					m_motor_ctrl.m_spd.set_spd_val -= DEC_SPD_RPM;
				}
				else
				{
					m_motor_ctrl.m_spd.set_spd_val = m_motor_ctrl.q16_spd_val;
				}
			}
			
		}
	}
	/*速度PID闭环控制*/
	if(m_motor_ctrl.m_spd.stabilize_sign == true)
	{
		/*转速有更新*/
		if(m_motor_ctrl.m_spd.speed_update_sign)
		{
			loop_cnt++;
			m_motor_ctrl.m_spd.speed_update_sign = false;
			/*更新速度环设置目标值*/
			m_spd_pid_unit.q15_target_value = m_motor_ctrl.m_spd.set_spd_val;
			/*更新速度环实时值*/
			m_spd_pid_unit.q15_actual_value = m_motor_ctrl.m_spd.spd_val;
			m_spd_pid_unit.q15_out_val = m_series_pid_algorithm(&m_spd_pid_unit);
			//m_spd_pid_unit.q15_out_val = m_parallel_incremental_pid_algorithm(&m_spd_pid_unit);
			switch(m_motor_ctrl.direction)
			{
				case CCW:
					/*Iq最小值限定*/
					if(m_spd_pid_unit.q15_out_val < SET_IQ_MIN)
					{
						m_iq_pid_unit.q15_target_value = SET_IQ_MIN;
					}
					/*正常范围内*/
					else
					{
						m_iq_pid_unit.q15_target_value = m_spd_pid_unit.q15_out_val;
					}
				break;
				case CW:
					/*Iq最小值限定*/
					if(m_spd_pid_unit.q15_out_val < SET_IQ_MIN)
					{
						m_iq_pid_unit.q15_target_value = -SET_IQ_MIN;
					}
					else
					{
						m_iq_pid_unit.q15_target_value = -m_spd_pid_unit.q15_out_val;
					}
				break;
			}
		}
	}
	/*速度闭环未开启*/
	else
	{
		/*速度闭环未开启：电流环先执行，Iq目标电流设置为启动电流*/
		m_iq_pid_unit.q15_target_value = m_motor_ctrl.q15_start_iq;
		/*误差累加和设置为0*/
//		m_iq_pid_unit.i_err_sum.s32 = 0;
	}
}

/**
 ******************************************************************************
 * @brief  电机FOC算法执行
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_foc_algorithm_execute(void)
{   
	switch(m_motor_ctrl.state_machine)
	{
		case EXECUTE_MOTOR_STOP:	//电机停止
		{
			m_motor_stop();
		}
		break;
		case EXECUTE_MOTOR_START:	//电机启动
		{
			// m_motor_monitor_init(); //清除霍尔异常状态
			// /*状态监测异常*/
			// if(m_monitor_unit.error != 0)
			// {
			// 	m_motor_ctrl.state_machine = EXECUTE_MOTOR_STOP;
			// }
			// /*状态监测正常，执行电机启动步骤*/
			// else
			// {
			// 	m_tick_unit.boot_charge_time = BOOTSTRAP_BOOST_CHARGING_TIME;
			// 	m_motor_boost_charge();
			// }
            m_tick_unit.boot_charge_time = BOOTSTRAP_BOOST_CHARGING_TIME;
			m_motor_boost_charge();
		}
		break;
		case EXECUTE_MOTOR_BOOST_CHARGING://等待充电
		{
			if(!m_tick_unit.boot_charge_time)
			{
				m_motor_init();			//电机参数初始化
				m_rotor_angle_init();	//转子位置角初始化
				m_current_pid_init(); 	//电流环PID初始化
				m_spd_pid_init(); 		//速度环PID初始化	
				m_motor_ctrl.state_machine = EXECUTE_MOTOR_EXECUTE;
			}
		}
		break;
		case EXECUTE_MOTOR_EXECUTE:	//电机执行
		{
			/*转子位置角计算*/
			m_foc_unit.rotor_engle = m_rotor_angle_calculate();
			/*通过Ud Uq进行Us模长计算以及超前角θc计算*/
			m_us_theta_c_calculate();
			m_spd_pid_execute();
			switch(m_motor_ctrl.direction)
			{
				case CCW:
					m_foc_unit.q_engle = m_foc_unit.rotor_engle + EANGLE90 + m_foc_unit.advance_angle;
				break;
				case CW:
					m_foc_unit.q_engle = m_foc_unit.rotor_engle - EANGLE90 - m_foc_unit.advance_angle;
				break;
			}
			/*将电流闭环计算的Us模长幅值到M值，也就是矢量圆的半径*/
			m_us_unit.q16_m_value = m_foc_unit.q16_us;
			m_svpwm_generate(m_us_unit.q16_m_value, m_foc_unit.q_engle);
		}
		break;
	}
}

/**
 * @}
 */
/******************* (C) COPYRIGHT 2024 PengLi ******END OF FILE******************/