/**
 ******************************************************************************
 * @file    m_pid.c
 * @author  chengbb (Ported to STM32G4)
 * @brief   motor pid 源文件
 ******************************************************************************
 */
#include "m_pid.h"

m_pid_unit_t m_id_pid_unit;     //电流环id pid
m_pid_unit_t m_iq_pid_unit;     //电流环iq pid
m_pid_unit_t m_spd_pid_unit;    //速度环pid

/**
 ******************************************************************************
 * @brief  电流环PI初始化
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_current_pid_init(void)
{
    m_id_pid_unit.q16_kp = 32768;               //比例项系数 32767
    m_id_pid_unit.q16_ki = 2048;                //积分项系数 2048
    m_id_pid_unit.q16_kd = 0;                   //微分项系数
    m_id_pid_unit.q15_out_val = 0;              //计算结果输出值
    m_id_pid_unit.q15_last_err = 0;             //上次误差值
    m_id_pid_unit.q15_err = 0;                  //当前误差值
    m_id_pid_unit.s_i_limit.s32 = 0;            //积分项限幅值
    m_id_pid_unit.i_sum.s32 = 0;                //积分项累加和 
    m_id_pid_unit.q15_out_val_limit = 32767;    //pid计算结果输出最大值
    m_id_pid_unit.q15_target_value = 0;         //PID实时目标值
    m_id_pid_unit.q15_actual_value = 0;         //PID实时反馈值
    
    m_iq_pid_unit.q16_kp = 32768;               //比例项系数 32767
    m_iq_pid_unit.q16_ki = 2048;                //积分项系数 2048
    m_iq_pid_unit.q16_kd = 0;                   //微分项系数
    m_iq_pid_unit.q15_out_val = 0;              //计算结果输出值
    m_iq_pid_unit.q15_last_err = 0;             //上次误差值
    m_iq_pid_unit.q15_err = 0;                  //当前误差值
    m_iq_pid_unit.s_i_limit.s32 = 0;            //积分项限幅值
    m_iq_pid_unit.i_sum.s32 = 0;                //积分项累加和 
    m_iq_pid_unit.q15_out_val_limit = 32767;    //pid计算结果输出最大值
    m_iq_pid_unit.q15_target_value = 0;         //PID实时目标值
    m_iq_pid_unit.q15_actual_value = 0;         //PID实时反馈值
}

/**
 ******************************************************************************
 * @brief  速度环PI初始化
 * @param  None.
 * @retval None.
 ******************************************************************************/
void m_spd_pid_init(void)
{
    m_spd_pid_unit.q16_kp = 25000;              //比例项系数 16384
    m_spd_pid_unit.q16_ki = 10;                 //积分项系数 10
    m_spd_pid_unit.q16_kd = 0;                  //微分项系数
    m_spd_pid_unit.q15_out_val = 0;             //计算结果输出值
    m_spd_pid_unit.q15_last_err = 0;            //上次误差值
    m_spd_pid_unit.q15_err = 0;                 //当前误差值
    m_spd_pid_unit.s_i_limit.s32 = 0;           //积分项限幅值
    m_spd_pid_unit.i_sum.s32 = 0;               //积分项累加和 
    m_spd_pid_unit.q15_out_val_limit = 32767;   //pid计算结果输出最大值
    m_spd_pid_unit.q15_target_value = 0;        //PID实时目标值
    m_spd_pid_unit.q15_actual_value = 0;        //PID实时反馈值
}

/**
 ******************************************************************************
 * @brief  并联增量式PID算法实现
 * @param  pid:PID数据指针
 * @retval 计算结果输出值
 ******************************************************************************/
int16_t m_parallel_incremental_pid_algorithm(m_pid_unit_t *pid) 
{
    union_s32 val;
    
    /*计算当前误差*/
    pid->q15_err = pid->q15_target_value - pid->q15_actual_value;
    
    /*比例项计算*/
    val.s32 = pid->q16_kp * (pid->q15_err - pid->q15_last_err);     //Q16*Q15=Q31
    pid->q15_kp_value = val.words.high;                             //Q15
    
    /*积分项计算*/
    val.s32 = pid->q16_ki * pid->q15_err;
    pid->q15_ki_value = val.words.high;
    
    val.s32 = pid->q15_out_val + pid->q15_kp_value + pid->q15_ki_value;
    
    /*输出最大值限幅*/
    if(val.s32 > pid->q15_out_val_limit)
    {
        pid->q15_out_val = pid->q15_out_val_limit;
    }
    /*输出最小值限幅*/
    else if(val.s32 < -pid->q15_out_val_limit)
    {
        pid->q15_out_val = -pid->q15_out_val_limit;
    }
    /*正常输出范围内*/
    else
    {
        pid->q15_out_val += pid->q15_kp_value + pid->q15_ki_value;
    }
    pid->q15_last_err = pid->q15_err;
    
    return pid->q15_out_val;
}

/**
 ******************************************************************************
 * @brief  并联型位置式PID算法实现   抗积分饱和
 * @param  pid:PID数据指针
 * @retval 计算结果输出值
 ******************************************************************************/
int16_t m_parallel_position_pid_algorithm(m_pid_unit_t *pid) 
{
    union_s32 val;
    int16_t   d_diff_val;
    
    /*计算当前误差*/
    pid->q15_err = pid->q15_target_value - pid->q15_actual_value;
    
    /*比例项计算*/
    val.s32 = pid->q16_kp * pid->q15_err;   //Q16*Q15=Q31
    pid->q15_kp_value = val.words.high;     //比例项计算结果值 Q15
    
    /*积分项值最大限幅动态计算 (Anti-Windup) */
    if(pid->q15_kp_value >= 0)
    {
        val.words.high = pid->q15_out_val_limit - pid->q15_kp_value;
    }
    else
    {
        val.words.high = pid->q15_out_val_limit + pid->q15_kp_value;
    }
    
    val.words.low = 0;
    pid->s_i_limit.s32 = val.s32;           //积分项限幅值
    
    /*积分项计算*/
    val.s32 = pid->q16_ki * pid->q15_err; //Q16*Q15=Q31
    pid->i_sum.s32 += val.s32;            //Q31
    
    /*积分项抗饱和最大值限幅*/
    if(pid->i_sum.s32 > pid->s_i_limit.s32)
    {
        pid->i_sum.s32 = pid->s_i_limit.s32;
    }
    /*积分项抗饱和最小值限幅*/
    else if(pid->i_sum.s32 < -pid->s_i_limit.s32)
    {
        pid->i_sum.s32 = -pid->s_i_limit.s32;
    }
    pid->q15_ki_value = pid->i_sum.words.high; //积分项计算结果值
    
    /*微分项计算*/
    d_diff_val = pid->q15_err - pid->q15_last_err; //微分项差值
    pid->q15_last_err = pid->q15_err;              //上次误差值更新
    val.s32 = pid->q16_kd * d_diff_val;            //Q16*Q15=Q31
    pid->q15_kd_value = val.words.high;            //微分项计算结果值
    
    /*Kp Ki Kd输出和*/
    val.s32 = (int32_t)pid->q15_kp_value + (int32_t)pid->q15_ki_value + (int32_t)pid->q15_kd_value;
    
    /*输出最大值限幅*/
    if(val.s32 > pid->q15_out_val_limit)
    {
        pid->q15_out_val = pid->q15_out_val_limit;
    }
    /*输出最小值限幅*/
    else if(val.s32 < -pid->q15_out_val_limit)
    {
        pid->q15_out_val = -pid->q15_out_val_limit;
    }
    /*正常输出范围内*/
    else
    {
        pid->q15_out_val = pid->q15_kp_value + pid->q15_ki_value + pid->q15_kd_value;
    }
    
    return pid->q15_out_val;
}

/**
 ******************************************************************************
 * @brief  串联型PID算法实现
 * @param  pid:PID数据指针
 * @retval 计算结果输出值
 ******************************************************************************/
int16_t m_series_pid_algorithm(m_pid_unit_t *pid) 
{   
    union_s32 val;
    
    /*计算当前误差*/
    pid->q15_err = pid->q15_target_value - pid->q15_actual_value;
    
    /*比例项计算*/
    val.s32 = pid->q16_kp * pid->q15_err;   //Q16*Q15=Q31
    pid->q15_kp_value = val.words.high;     //比例项计算结果值
    
    /*积分项值最大限幅 (Anti-Windup)*/
    if(pid->q15_kp_value >= 0)
    {
        val.words.high = pid->q15_out_val_limit - pid->q15_kp_value;
    }
    else
    {
        val.words.high = pid->q15_out_val_limit + pid->q15_kp_value;
    }
    
    val.words.low = 0;
    pid->s_i_limit.s32 = val.s32;               //积分项限幅值
    
    /*积分项计算 (基于 Kp 的串联积分)*/
    val.s32 = pid->q16_ki * pid->q15_kp_value;  //Q16*Q15=Q31
    pid->i_sum.s32 += val.s32;                  //Q31
    
    /*积分项抗饱和最大值限幅*/
    if(pid->i_sum.s32 > pid->s_i_limit.s32)
    {
        pid->i_sum.s32 = pid->s_i_limit.s32;
    }
    /*积分项抗饱和最小值限幅*/
    else if(pid->i_sum.s32 < -pid->s_i_limit.s32)
    {
        pid->i_sum.s32 = -pid->s_i_limit.s32;
    }
    pid->q15_ki_value = pid->i_sum.words.high; //积分项计算结果值
    
    /*Kp Ki输出和*/
    val.s32 = (int32_t)pid->q15_kp_value + (int32_t)pid->q15_ki_value;
    
    /*输出最大值限幅*/
    if(val.s32 > pid->q15_out_val_limit)
    {
        pid->q15_out_val = pid->q15_out_val_limit;
    }
    /*输出最小值限幅*/
    else if(val.s32 < -pid->q15_out_val_limit)
    {
        pid->q15_out_val = -pid->q15_out_val_limit;
    }
    /*正常输出范围内*/
    else
    {
        pid->q15_out_val = pid->q15_kp_value + pid->q15_ki_value;
    }
    
    return pid->q15_out_val;
}