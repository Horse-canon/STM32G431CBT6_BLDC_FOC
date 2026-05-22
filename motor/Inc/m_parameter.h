/**
  ******************************************************************************
  * @file    m_parameter.h
  * @author  Jiang Yuhao (江雨豪)适配
  * @version V1.1
  * @date    2026-04-23
  * @brief   电机参数头文件 (STM32G431 适配版)
  ******************************************************************************
  */

#ifndef __M_PARAMETER_H__
#define __M_PARAMETER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" 

/* --- 滤波算法宏定义 --- */
/* 功能：一阶低通滤波 Yout = Yout * 0.75 + Xin * 0.25 */
#define LPF_Calc(Xin, Yout)        ((Yout>>1)+(Yout>>2)+(Xin>>2))

/* --- 数学常量 --- */
/* sqrt(3)/2 的 Q16 格式: 0.866025 * 65536 = 56756 */
/* 1/sqrt3 的 Q16 格式: 37837 */
#define SQRT3DIV2                  56756  
#define ONE_DIV_SQRT3              37837  

/* --- PWM 周期参数 (核心适配点) --- */
/* * STM32G431 主频 170MHz
 * 中心对齐模式下：ARR = 170MHz / (2 * 20KHz) = 4250
 */
#define MCU_PWM_TIMER_ARR         (4250)  
#define PWM_FREQUENCY							(20000)   //PWM频率 20KHZ
#define PWM_PERIOD_T							((float)(1.0f / (float)PWM_FREQUENCY))//PWM周期

/* --- 占空比限制与死区补偿 --- */
/* * 死区时间计算：170MHz 时钟，1us 约对应 170 个计数位
 * 注意：如果您依靠预驱 FD6288T 硬件死区，这里可以设小或设为 0
 */
#define DEAD_TIME                  34         // 170MHz 下 1us 对应 170
#define MIN_DUTY_VALUE             (DEAD_TIME )        // 最小有效脉宽限制（防止波形太窄）
#define MAX_DUTY_VALUE             (MCU_PWM_TIMER_ARR  - (DEAD_TIME + (DEAD_TIME >> 1)))

/* --- 电角度常量 (Q16 格式，0~65535 对应 0~360度) --- */
#define      EANGLE0       0
#define      EANGLE30      5461
#define      EANGLE60      10922
#define      EANGLE90      16384
#define      EANGLE120     21845
#define      EANGLE150     27306
#define      EANGLE180     32768
#define      EANGLE210     38229
#define      EANGLE240     43690
#define      EANGLE270     49151
#define      EANGLE300     54613
#define      EANGLE330     60074
#define      EANGLE360     0

/* --- 速度计算相关 --- */
/* 计算公式保持不变，注意 us 参数传入 --- */
#define SPEED_HALL_TIME_CALCULATE(speed, p, us) (uint32_t)(60000000.0f / ((float)speed * (float)us * 6.0f * (float)p))

/* 电机极对数及范围设定 */
#define MOTOR_POLE_PAIRS           2
#define MOTOR_MIN_SPEED            1000     //电机最小转速
#define MOTOR_MAX_SPEED            2300    //电机最大转速 36BL61:2300  3650:6000
#define MIN_SPEED_HALL_TIME_VALUE  SPEED_HALL_TIME_CALCULATE(MOTOR_MIN_SPEED, MOTOR_POLE_PAIRS, 1)
#define MAX_SPEED_HALL_TIME_VALUE  SPEED_HALL_TIME_CALCULATE(MOTOR_MAX_SPEED, MOTOR_POLE_PAIRS, 1)

/* 角度步进计算 */
#define DθR_DIFF_CACULATE(us)      ((uint32_t)(50.0f * 60.0f * 65536.0f / ((float)us * 360.0f)))
#define DθR_DIFF_VALUE             DθR_DIFF_CACULATE(1)    

/* 方向定义 */
#define CW                         0
#define CCW                        1

/* --- 状态机时间参数 --- */
#define BOOTSTRAP_BOOST_CHARGING_TIME      (200)   // 200ms 充电时间
#define HALL_VALUE_TIMEOUT_THRESHOLD_VALUE (20 * 1000) // 超时阈值

/* --- 速度指令 (Q16) 范围 --- */
#define SPD_Q16_MIN_VALUE          512
#define SPD_Q16_MAX_VALUE          65535

/* --- 调制比 M 限幅 (Q16) --- */
/* 0.9 * 65536 ≈ 58982 (最大调制比) */
#define M_MAX_VALUE                58982
#define M_MIN_VALUE                655
#define M_OPEN_LOOP_VALUE                       9830

#define US_MAX_VALUE                            (int16_t)29491   //Us模长最大值 Q15格式 90%
#define PHASE_CURRENT_OFFSET_TIME               (20 * 1000)      //1s

#define SPD_PID_CYCLE_TIME                      (20 * 30)        //200ms
#define INC_SPD_RPM                             100              //斜坡转速递增步进        
#define DEC_SPD_RPM                             100              //斜坡减速递减步进
#define SET_IQ_MIN                              20               //最小Iq设定值 20
#define START_IQ                                1000              //启动运行阶段Iq 800

#define MOTOR_HALL_STABILIZE_NUMBER             12               //电机霍尔传感器稳定检测阈值         // Iq过渡总步数（50步 × 50us = 2.5ms完成过渡）

/*线电感*/
#define LINE_INDUCTANCE_U_V_MEASUREMENT			(12.011f * 1000.0f)		 //UV线电感 mH*1000=uH	
#define LINE_INDUCTANCE_U_W_MEASUREMENT			(13.352f * 1000.0f)		 //UW线电感	mH*1000=uH 	
#define LINE_INDUCTANCE_V_W_MEASUREMENT			(12.850f * 1000.0f)		 //VW线电感	mH*1000=uH
/*相电感=三相线电感之和/3/2*/
#define PHASE_INDUCTANCE_MEASUREMENT			((LINE_INDUCTANCE_U_V_MEASUREMENT+ LINE_INDUCTANCE_U_W_MEASUREMENT+ LINE_INDUCTANCE_V_W_MEASUREMENT) / 3.0f / 2.0f)

/*线电阻*/
#define LINE_RESISTANCE_U_V_MEASUREMENT			(13.2f)		 //UV线电阻 Ohm	
#define LINE_RESISTANCE_U_W_MEASUREMENT			(13.2f)		 //UW线电阻	Ohm	
#define LINE_RESISTANCE_V_W_MEASUREMENT			(13.3f)		 //VW线电阻	Ohm
/*相电阻=三相线电阻之和/3/2*/
#define PHASE_RESISTANCE_MEASUREMENT			((LINE_RESISTANCE_U_V_MEASUREMENT+ LINE_RESISTANCE_U_W_MEASUREMENT+ LINE_RESISTANCE_V_W_MEASUREMENT) / 3.0f / 2.0f)

/*F = 1 - Ts * R / L   Q15格式*/												
#define F_COEFF									((int16_t)((1.0f - PWM_PERIOD_T * 1000000.0f * PHASE_RESISTANCE_MEASUREMENT / PHASE_INDUCTANCE_MEASUREMENT)*32768.0f))
/*G = Ts / L   Q15格式*/																								
#define G_COEFF									((int16_t)(PWM_PERIOD_T * 1000000.0f/ PHASE_INDUCTANCE_MEASUREMENT*32768.0f))

/*观测器最大S值 Q15格式 0-32767范围*/ 
#define OBSERVER_S_MAX_VALUE					170//150  256
/*观测器K滑膜增益系数  Q15格式 0-32767范围*/
#define OBSERVER_K_SLIDE						30000//30000  32767

#define ROTOR_POSITION_STEP_1_US_VALUE			(20000)			      //转子预定位第1步:Us模长 10%	
#define ROTOR_POSITION_STEP_1_TIME				(20 * 500)	 	      //转子预定位第1步持续时间:20K->50us  500ms
#define ROTOR_POSITION_STEP_1_THETA_ANGLE       (EANGLE90)		//转子预定位第1步预定位角度:Us与0°矢量夹角

#define ROTOR_POSITION_STEP_2_US_VALUE			(20000)			//转子预定位第1步:Us模长
#define ROTOR_POSITION_STEP_2_TIME				(20 * 500)		//转子预定位第2步持续时间:20K->50us 500ms
#define ROTOR_POSITION_STEP_2_THETA_ANGLE		(EANGLE0)		//转子预定位第2步预定位角度:Us与0°矢量夹角

#define MOTOR_DRAG_TICK_TIME					(10)			    //强拖tick时间 500us
#define MOTOR_DRAG_ACC_MAX_VALUE				(65000)			//强拖加速度最大值
#define MOTOR_DRAG_ACC_STEP_VALUE				(5)				  //加速度步进值
#define MOTOR_DRAG_CLOSE_LOOP_TICK				(100)			//强拖切换闭环tick时间电流校正周期
#define MOTOR_DRAG_CLOSE_LOOP_THRESHOLD_VALUE	(3)		//切换到闭环阈值
#define MOTOR_DRAG_ACC_MAX_OMIGA_VALUE			(546)		//500RPM最小转速对应的角速度Q15格式值

#define Q16_TO_Q15(q16_x) 						(int16_t)(q16_x >> 1) //Q16数据转Q15数据格式
#define Q15_TO_Q16(q15_x) 						(uint16_t)(q15_x)	  //Q15数据转Q16数据格式

#define SPD_PID_EXECUTE_CYCLE					(20 * 10)		//10ms执行一次

#ifdef __cplusplus
#define OPEN_LOOP_UQ                    20000                          // 固定Uq电压 Q15格式 (3000 ≈ 9.2% of 32767)
#endif

#endif /* __M_PARAMETER_H__ */