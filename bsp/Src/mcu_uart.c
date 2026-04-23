#include "mcu_uart.h"
#include "usart.h"  // STM32CubeMX 生成的头文件，包含外设句柄 extern UART_HandleTypeDef huart2;
#include <stdio.h>

/**
  ******************************************************************************
  * @brief  UART阻塞方式发送不定长数据
  * @param  com :端口号
  * @param  data:发送数据指针
  * @param  len :发送数据长度
  * @retval None.
  ******************************************************************************
  */
void drv_uart_send_data(uart_com_e com, uint8_t *data, uint32_t len)
{    
    switch(com)
    {
        case DEBUG_COM:
            /* HAL_UART_Transmit 内部已包含等待发送完成的逻辑，HAL_MAX_DELAY 表示一直阻塞直到发送完毕 */
            HAL_UART_Transmit(&huart2, data, (uint16_t)len, HAL_MAX_DELAY);
            break;
            
        default:
            break;
    }
}

/**
  ******************************************************************************
  * @brief  printf 等标准 I/O 库输出重定向
  * @note   兼容 VS Code 的 GCC 编译器和 MDK 的 ARMCC 编译器
  ******************************************************************************
  */
#ifdef __GNUC__
/* GCC 环境下重定向标准输出流，供 printf 使用 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    /* 通过 USART2 发送数据，作为 DEBUG 输出端口 */
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
#else
/* ARMCC 环境下 (如 Keil MDK) 重定向标准输出流 */
int fputc(int ch, FILE *f)
{
    (void)f;
    /* 等同于 R_SCI_B_UART_Write + 等待 TEND 标志位 */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif