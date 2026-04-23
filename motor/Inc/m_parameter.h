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
#define SQRT3DIV2                  56756  

/* --- PWM 周期参数 (核心适配点) --- */
/* * STM32G431 主频 170MHz
 * 中心对齐模式下：ARR = 170MHz / (2 * 20KHz) = 4250
 */
#define PWM_PERIOD_T_VALUE         (4250)  

/* --- 占空比限制与死区补偿 --- */
/* * 死区时间计算：170MHz 时钟，2us 约对应 340 个计数位
 * 注意：如果您依靠预驱 FD6288T 硬件死区，这里可以设小或设为 0
 */
#define DEAD_TIME                  340         // 170MHz 下 2us 对应 340
#define MIN_DUTY_VALUE             (20)        // 最小有效脉宽限制（防止波形太窄）

/* * 最大占空比限幅：
 * 在 PWM Mode 2 下，CCR 越小占空比越大。
 * 为了给自举电容留出充电时间，不能让上管 100% 开启，
 * 因此 CCR 不能真的到 0。这里留出 2us 的余量（340）。
 */
#define MAX_DUTY_VALUE             (PWM_PERIOD_T_VALUE - DEAD_TIME)

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
#define MIN_SPEED_HALL_TIME_VALUE  SPEED_HALL_TIME_CALCULATE(50, MOTOR_POLE_PAIRS, 1)
#define MAX_SPEED_HALL_TIME_VALUE  SPEED_HALL_TIME_CALCULATE(3000, MOTOR_POLE_PAIRS, 1)

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

#ifdef __cplusplus
}
#endif

#endif /* __M_PARAMETER_H__ */