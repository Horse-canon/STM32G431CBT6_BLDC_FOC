#ifndef __M__FOC_H__
#define __M__FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
	int16_t q15_ia; 		//a相相电流
	int16_t q15_ib; 		//b相相电流
	int16_t q15_ic; 		//c相相电流
	
	int16_t q15_i_alpha;	//clark变换Iα
	int16_t q15_i_beta;		//clark变换Iβ
	
	int16_t q15_iclark_ia;	//clark逆变换a相相电流
	int16_t q15_iclark_ib;	//clark逆变换b相相电流
	int16_t q15_iclark_ic;  //clark逆变换c相相电流
	
	int16_t q15_id;			//park变换Id
	int16_t q15_iq;			//park变换Iq
	int16_t q15_id_filter;	//park变换Id滤波值
	int16_t q15_iq_filter;	//park变换Iq滤波值
	
	int16_t q15_ud;			//PI控制器d轴Ud输出
	int16_t q15_uq;			//PI控制器q轴Uq输出
	
	int16_t q15_u_alpha;	//park逆变换Uα
	int16_t q15_u_beta;		//park逆变换Uβ
	
}m_coordinate_t;

typedef struct
{   
    uint16_t rotor_engle;		//转子位置角
	uint16_t q_engle;   		//Us与0°位置夹角
	uint16_t advance_angle;		//超前角
	
	uint16_t q16_us;			//Q15 pid闭环计算的Ud Uq值计算的Us模长
	int16_t  q15_us_max;		//Us模长最大值
	
	m_coordinate_t coordinate;	//坐标变换参数
} m_foc_unit_t;

extern m_foc_unit_t m_foc_unit;

void m_us_radius_calculate(void);
void m_current_pid_execute(void);
void m_spd_pid_execute(void);
void m_foc_algorithm_execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __M__FOC_H__ */