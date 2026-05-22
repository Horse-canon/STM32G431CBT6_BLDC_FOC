/* 引入必要的头文件 */
#include "main.h"

typedef struct
{
    int16_t  q15_theta_e;            //θe角度->相位90°补偿    
    
    int16_t  q15_last_theta;         //转速计算参数：上一次角度值记录    
    int16_t  q15_sum_theta;          //角度累加和
    int16_t  q15_cnt_theta;          //角度累加记录
    
    int16_t  q15_delta_theta;        //δθ角度
    int16_t  q15_filter_delta_theta; //δθ滤波处理角度
    int16_t  q15_k_dt;               //δθ LPF低通滤波系数值
    
    int16_t  q15_erpm;               //电转速 eRPM 单位：min
    uint16_t erpm_filter1;           //机械转速滤波值1
    uint16_t erpm_filter2;           //机械转速滤波值2
    
    uint32_t angle_60_time;          //60度电角度时间 (等效霍尔时间，供外部使用)
    
    uint16_t q16_rotor_angle;        //最终估算出的转子位置角 (0~65535)
} m_obs_angle_unit_t;

extern m_obs_angle_unit_t m_obs_angle_unit;

void m_sensorless_rotor_angle_init(void);
uint16_t m_sensorless_rotor_angle_calculate(void);