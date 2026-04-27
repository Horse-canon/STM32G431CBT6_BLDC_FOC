/**
 ******************************************************************************
 * @file    m_pid.h
 * @author  chengbb (Ported to STM32G4)
 * @brief   motor pid 头文件
 ******************************************************************************
 */
#ifndef __M_PID_H__
#define __M_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 引入 STM32 HAL 库核心头文件，替换原有的 hal_data.h */
#include "main.h"

/* 确保这个路径正确指向了你定义 union_s32 的头文件 */
#include "typedef_header.h" 

typedef struct
{
    uint16_t  q16_kp;           //比例项系数
    uint16_t  q16_ki;           //积分项系数
    uint16_t  q16_kd;           //微分项系数
    
    int16_t   q15_kp_value;     //比例项计算值
    int16_t   q15_ki_value;     //积分项计算值
    int16_t   q15_kd_value;     //微分项计算值
    
    int16_t   q15_out_val;      //计算结果输出值
    
    int16_t   q15_last_err;     //上次误差值
    int16_t   q15_err;          //当前误差值
    
    int16_t   q15_out_val_limit;//pid计算结果输出最大值
    
    union_s32 s_i_limit;        //积分项限幅值
    union_s32 i_sum;            //积分项累加和 
    
    int16_t   q15_target_value; //PID实时目标值
    int16_t   q15_actual_value; //PID实时反馈值
} m_pid_unit_t;

extern m_pid_unit_t m_id_pid_unit;      //电流环id pid
extern m_pid_unit_t m_iq_pid_unit;      //电流环iq pid
extern m_pid_unit_t m_spd_pid_unit;     //速度环pid

void m_current_pid_init(void);
void m_spd_pid_init(void);
int16_t m_parallel_incremental_pid_algorithm(m_pid_unit_t *pid);
int16_t m_parallel_position_pid_algorithm(m_pid_unit_t *pid);
int16_t m_series_pid_algorithm(m_pid_unit_t *pid); 

#ifdef __cplusplus
}
#endif

#endif /* __M_PID_H__ */