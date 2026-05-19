/**
  ******************************************************************************
  * @file    m_rotor_angle.c
  * @author  jiangyuhao
  * @version V1.1
  * @date    2026-04-24
  * @brief   转子位置角解算
  ******************************************************************************
  */
/** @addtogroup MOTOR
* @{
*/
#include "m_rotor_angle.h"
#include <stdio.h>
#include "m_parameter.h"
#include "mcu_hall_cb.h"
#include "m_foc.h"
#include "m_ctrl.h"
#include "typedef_header.h" // 确保包含 union_u32 和 LPF_CALC 定义
#include "m_pid.h"



// //转子位置角解算表36BL61 3560
// static const uint16_t  ROTOR_ANGLE_TABLE_CCW[7]  = {0,EANGLE330,EANGLE210,EANGLE270,EANGLE90,EANGLE30,EANGLE150};
// static const uint16_t  ROTOR_ANGLE_TABLE_CW[7]   = {0,EANGLE30,EANGLE270,EANGLE330,EANGLE150,EANGLE90,EANGLE210};
// static const uint16_t  ROTOR_ANGLE_INIT_TABLE[7] = {0,EANGLE0,EANGLE240,EANGLE300,EANGLE120,EANGLE60,EANGLE180};


// CCW 边界: 1(348°), 5(44.376°), 4(98.508°), 6(166.68°), 2(215.454°), 3(265.236°)
static const uint32_t ROTOR_ANGLE_TABLE_CCW[7] = {
    0, 
    63351, // H1: 348.000 * 182.044
    39222, // H2: 215.454 * 182.044
    48285, // H3: 265.236 * 182.044
    17933, // H4: 98.508  * 182.044
    8078,  // H5: 44.376  * 182.044
    30343  // H6: 166.680 * 182.044
};

// CW 边界: 1(8.232°), 5(66.408°), 4(127.218°), 6(187.446°), 2(245.82°), 3(312.03°)
static const uint32_t ROTOR_ANGLE_TABLE_CW[7] = {
    0, 
    1498,  // H1: 8.232   * 182.044
    44749, // H2: 245.820 * 182.044
    56801, // H3: 312.030 * 182.044
    23163, // H4: 127.218 * 182.044
    12089, // H5: 66.408  * 182.044
    34123  // H6: 187.446 * 182.044
};


/* 5. 初始绝对中心表 (用于静止刚上电的第一脚启动，取双向平均抵消磁滞) */
/* ========================================================================= */
static const uint32_t ROTOR_ANGLE_INIT_TABLE[7] = {
    0, 
    65201, // H1: 真实中心 358.16°
    41595, // H2: 真实中心 228.49°
    53298, // H3: 真实中心 292.77°
    20881, // H4: 真实中心 114.70°
    9899,  // H5: 真实中心 54.38°
    31712  // H6: 真实中心 174.20°
};

/* ------------------------------------------------------------------------- */
/* 4. 扇区最大宽度钳位表 (防止角度越界跑到下一个扇区): 扇区宽度 * 65536 / 360 */
static const uint32_t SECTOR_MAX_WIDTH_TABLE_CCW[7] = {
    0, 10263, 9063, 15067, 12410, 9854, 8879
};

static const uint32_t SECTOR_MAX_WIDTH_TABLE_CW[7] = {
    0, 10231, 10627, 12053, 11070, 10591, 10964
};

/* 测速专用：360度电周期环形缓冲区 (完美消除霍尔不对称引起的测速波动) */
static uint32_t hall_time_buf[6] = {0};
static uint8_t  hall_time_idx = 0;
static uint32_t hall_time_sum = 0;
static uint32_t interpolated_angle_sum = 0; // 记录当前扇区已走过的总角度


static union_u32 rotor_angle;
static union_u32 rotor_angle_inc;
static union_u32 monitor_rotor_angle;



m_hall_unit_t m_hall_unit;

/**
  ******************************************************************************
  * @brief  霍尔值获取：获取周期为50us
  ******************************************************************************
  */
void m_hall_value_get(void)
{
    uint8_t hall_u;
    uint8_t hall_v;
    uint8_t hall_w;
    
    /* [STM32 移植] 替换为 STM32 HAL 库直接读取对应 GPIO 状态 */
    hall_u = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);  // TIM3_CH1 (HALL U)
    hall_v = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);  // TIM3_CH2 (HALL V)
    hall_w = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);  // TIM3_CH3 (HALL W)
    
    m_hall_unit.u_val = hall_u;
    m_hall_unit.v_val = hall_v;
    m_hall_unit.w_val = hall_w;     
    
    m_hall_unit.value = (uint8_t)((hall_u << 2) | (hall_v << 1) | (hall_w << 0));
    
    if((m_hall_unit.value > 0) && (m_hall_unit.value < 7))
    {
        if(m_hall_unit.value != m_hall_unit.value_last)
        {
            m_hall_unit.value_last = m_hall_unit.value;
            m_hall_unit.update_sign = true;
        }
    }
    else
    {
        printf("hall value error!\\n");
    }   
}

/**
  ******************************************************************************
  * @brief  转子位置角初始化
  ******************************************************************************
  */
void m_rotor_angle_init(void)
{
    m_hall_unit.time = 0;
    m_hall_unit.time_last = 0;
    m_hall_unit.start_sign = true;
    m_hall_unit.angle_60_time = 0;
    m_hall_unit.angle_60_time_filter1 = 0;
    m_hall_unit.angle_60_time_filter2 = 0;
    m_hall_unit.start_cnt = 0;
    
    
    hall_capture_unit.hall_capture_reset_func();
    m_hall_value_get();

    /* --------强制清除上电首次读取造成的假跳变标志 -------- */
    m_hall_unit.update_sign = false; 
    /* ---------------------------------------------------------------- */

    rotor_angle_inc.u32 = 0;
    
    /* 统一使用标准 32 位数据格式，舍弃对底层非原生的增强宏 */
    rotor_angle.u32 = ROTOR_ANGLE_INIT_TABLE[m_hall_unit.value];
    
    m_hall_unit.hall_val_test_index = 0;
}

/**
 ******************************************************************************
 * @brief  转子位置角计算：间隔50us进行一次计算 (完美融合 STM32 硬件霍尔中断)
 * @retval 转子位置角 Q16 (0~65535 对应 0~360度)
 ******************************************************************************
 */
uint16_t m_rotor_angle_calculate(void)
{
    uint32_t delta_time = 0;
    
    /* 1. 获取当前霍尔引脚电平，判断是否发生状态变化 */
    m_hall_value_get();
    
    /* m_hall_unit.update_sign=true: 霍尔状态在此次 50us 周期内发生了跳变 */
    if(m_hall_unit.update_sign)
    {
        m_hall_unit.update_sign = false;
        m_hall_unit.update_cnt = 0;     

        /* 发生真实的物理跳变，清空插值累加器，重新开始计步 */
        interpolated_angle_sum = 0;
        
        switch(m_motor_ctrl.direction)
        {
            case CCW://逆时针
                /* 获取霍尔沿跳变瞬间的绝对基准转子位置角 */
                rotor_angle.u32 = ROTOR_ANGLE_TABLE_CCW[m_hall_unit.value];
                monitor_rotor_angle.u32 = rotor_angle.u32; 
            break;
            case CW: //顺时针
                rotor_angle.u32 = ROTOR_ANGLE_TABLE_CW[m_hall_unit.value];
                monitor_rotor_angle.u32 = rotor_angle.u32; 
            break;
        }   
        
        /* 2. 检查定时器中断是否已经捕获到了时间 */
        /* 在你的 HAL_TIM_IC_CaptureCallback 中，跳变时 hall_sign 被置位了 */
        if(*hall_capture_unit.hall_sign)
        {
            /* * 【完美对接中断】
             * 中断里已经把 60° 电角度的硬件级时间差存在变量里了，
             * 直接取 hall_capture_val_func() 
             */
            delta_time = hall_capture_unit.hall_capture_val_func(); 
            
            /* 清除中断标志位，等待下一次霍尔边沿跳变 */
            hall_capture_unit.hall_capture_sign_clear_func();
            
            /* 直接更新电角度时间 */
            //m_hall_unit.angle_60_time = delta_time;

           /* -------- 核心修改：第一次丢弃，第二次立刻启用 -------- */
            if (m_hall_unit.start_cnt == 0) 
            {
                /* 发生第 1 次跳变：距离残缺，真实时间作废 */
                m_hall_unit.start_cnt++;
                delta_time = 0; 
                m_hall_unit.angle_60_time = 0;
                
                m_hall_unit.angle_60_time_filter1 = 0;
                m_hall_unit.angle_60_time_filter2 = 0;
            }
            else
            {
                /* 发生第 2 次及以后的跳变：跑满了完整 60°，时间绝对真实！ */
                if(m_hall_unit.start_cnt < 255) m_hall_unit.start_cnt++; // 防止溢出
                
                m_hall_unit.angle_60_time = delta_time;

                /* 滤波器种子初始化：防止初始测速被 0 拖后腿 */
                if (m_hall_unit.angle_60_time_filter1 == 0)
                {
                    m_hall_unit.angle_60_time_filter1 = delta_time;
                    m_hall_unit.angle_60_time_filter2 = delta_time;
                }
            }
        }

        /* 3. 对 60° 电角度时间进行双重低通滤波，消除机械震动与干扰 */
        if (m_hall_unit.angle_60_time != 0)
        {
            m_hall_unit.angle_60_time_filter1 = LPF_CALC(m_hall_unit.angle_60_time, \
                                                         m_hall_unit.angle_60_time_filter1);
            m_hall_unit.angle_60_time_filter2 = LPF_CALC(m_hall_unit.angle_60_time_filter1, \
                                                         m_hall_unit.angle_60_time_filter2);
        }

        m_hall_unit.time = m_hall_unit.angle_60_time_filter2;
        
/* -------- 核心修改：三段式起步策略 -------- */
        if (m_hall_unit.start_cnt == 0)
        {
            /* 阶段一 (刚通电，未跳变)：
               死锁在扇区中点，提供最大恒定推力，绝不插值 */
            rotor_angle_inc.u32 = 0;
            m_motor_ctrl.m_spd.spd_val = 0; 
            hall_time_sum = 0;
            for(int i=0; i<6; i++) hall_time_buf[i] = 0;
        }
        else if (m_hall_unit.start_cnt == 1)
        {
            /* 阶段二 (第 1 次跳变后，等待第 2 次跳变)：
               为了防止磁场死锁导致 Iq 回落，人为给定一个恒定的低速插值步长！ */
            m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE; // 使用设定的最低转速(如50RPM)
            rotor_angle_inc.u32 = (uint32_t)((float)546133.0f / (float)m_hall_unit.time);

            for(int i=0; i<6; i++) hall_time_buf[i] = m_hall_unit.angle_60_time; 
            hall_time_sum = m_hall_unit.angle_60_time * 6;
            m_motor_ctrl.m_spd.spd_val = 50;
        }
        else
        {
            /* 阶段三 (第 2 次跳变及以后)：
               拥有了真实的 60 度时间，接入真实时间，丝滑闭环插值！ */
            if(m_hall_unit.time <= MAX_SPEED_HALL_TIME_VALUE)
                m_hall_unit.time = MAX_SPEED_HALL_TIME_VALUE;
            if(m_hall_unit.time >= MIN_SPEED_HALL_TIME_VALUE)
                m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE;
                

            /* 使用恒定角速度插值 */
            /* 50us * (65536 / 6) = 50 * 10922.66 = 546133 */
            /* 物理意义：无论当前扇区多宽，我们都以平均 60° 的时间基准匀速推进电角度 */
            rotor_angle_inc.u32 = (uint32_t)((float)546133.0f / (float)m_hall_unit.time);

            // 减去最旧的那个扇区时间
            hall_time_sum -= hall_time_buf[hall_time_idx];
            // 存入刚刚走完的这个扇区的原始时间
            // hall_time_buf[hall_time_idx] = m_hall_unit.angle_60_time;
            hall_time_buf[hall_time_idx] = m_hall_unit.time;
            // 加上最新的这个扇区时间
            hall_time_sum += hall_time_buf[hall_time_idx];
            // 游标推进
            hall_time_idx = (hall_time_idx + 1) % 6;
            /* 3. 【真实转速计算】：基于走完完整 6 个状态的总时间
               公式: RPM = 60,000,000 / (总时间 * 极对数) 
               (注意这里不需要再乘 6 了，因为 hall_time_sum 已经是 6 个状态的总和) */
            if (hall_time_sum > 0) {
                m_motor_ctrl.m_spd.spd_val = 60000000 / (hall_time_sum * MOTOR_POLE_PAIRS);
            }
        }

        if(m_motor_ctrl.m_spd.stabilize_cnt++ >= MOTOR_HALL_STABILIZE_NUMBER)
        {
            m_motor_ctrl.m_spd.stabilize_cnt     = MOTOR_HALL_STABILIZE_NUMBER;
            if(m_motor_ctrl.m_spd.stabilize_sign == false)
            {
                m_motor_ctrl.m_spd.set_spd_val = m_motor_ctrl.m_spd.spd_val;
                /*根据旋转方向设置速度环积分项初始值*/
                if(m_motor_ctrl.direction == CCW)
                {
                    m_spd_pid_unit.i_sum.s32 = (int32_t)m_motor_ctrl.q15_start_iq << 16;
                }
                else
                {
                    m_spd_pid_unit.i_sum.s32 = -(int32_t)m_motor_ctrl.q15_start_iq << 16;
                }
            }
            m_motor_ctrl.m_spd.stabilize_sign    = true;
            m_motor_ctrl.m_spd.speed_update_sign = true;
        }
    }
    else
    {
        /* 7. 没有霍尔跳变的 50us 周期，根据上一次算出的增量进行【角度插值】平滑估算 */
        if (m_hall_unit.update_cnt < HALL_VALUE_TIMEOUT_THRESHOLD_VALUE) 
        {   
            m_hall_unit.update_cnt++;

            uint32_t max_width = (m_motor_ctrl.direction == CCW) ? \
                                  SECTOR_MAX_WIDTH_TABLE_CCW[m_hall_unit.value] : \
                                  SECTOR_MAX_WIDTH_TABLE_CW[m_hall_unit.value];
            /* 重点修改：限定插值累计值不能超过当前扇区的物理宽度 
               预留 100 个单位 (约 0.5度) 的安全余量，防止浮点计算误差导致压线越界 */
            if ( (interpolated_angle_sum + rotor_angle_inc.u32) < (max_width) )
            {
                interpolated_angle_sum += rotor_angle_inc.u32;
                switch(m_motor_ctrl.direction)
                {
                    case CCW://逆时针
                        rotor_angle.u32 += rotor_angle_inc.u32;
                    break;
                    case CW: //顺时针
                        rotor_angle.u32 -= rotor_angle_inc.u32;
                    break;
                }   
            }
        }
        else
        {
            /* 超时异常处理 (检测到堵转，超过阈值时间没有收到霍尔跳变信号) */
            m_hall_unit.update_cnt = 0;  

            /* -------- 防死锁逻辑 -------- */
            m_motor_ctrl.m_spd.spd_val = 0;        // 1. 真实速度强制清零，打破 239 幻影
            hall_time_sum = 0;                     // 2. 清除历史测速缓存
            m_hall_unit.start_cnt = 0;             // 3. 重置起步状态机，允许重新插值

            /* 强制触发一次速度环更新！让 PI 控制器看到速度变成 0 了，从而重新输出大电流把电机“踹”醒 */
            m_motor_ctrl.m_spd.speed_update_sign = true;
        }
    }
    
    /* 返回最终计算/插值后的转子位置角 (0~65535) */
    return (uint16_t)rotor_angle.words.low;   
}

/**
  * @}
  */
/******************* (C) COPYRIGHT 2024 PengLi ******END OF FILE******************/