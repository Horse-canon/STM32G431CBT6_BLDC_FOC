/**
  ******************************************************************************
  * @file    typedef_header.h
  * @author  chengbb (Ported to STM32)
  * @version V1.1
  * @date    2025-01-10
  * @brief   公共类型定义头文件
  ******************************************************************************
  */
#ifndef __TYPEDEF_HEADER_H__
#define __TYPEDEF_HEADER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 移除瑞萨的 hal_data.h，替换为标准库和 STM32 的主头文件 */
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* * 这是一个非常轻量且高效的一阶低通滤波宏 (1/4 Xin + 3/4 Yout) 
 * 运算全部使用移位操作，非常适合电机控制这种对速度要求极高的场景
 */
#define LPF_CALC(Xin, Yout)     ((Yout>>1) + (Yout>>2) + (Xin>>2))  

/* 32位无符号整数与16位高低字的联合体转换 */
typedef union
{
    uint32_t u32;
    struct
    {
        uint16_t low;
        uint16_t high;
    } words;
} union_u32;

/* 32位有符号整数与16位高低字的联合体转换 */
typedef union
{
    int32_t s32;
    struct
    {
        int16_t low;
        int16_t high;
    } words;
} union_s32;

#ifdef __cplusplus
}
#endif

#endif /* __TYPEDEF_HEADER_H__ */