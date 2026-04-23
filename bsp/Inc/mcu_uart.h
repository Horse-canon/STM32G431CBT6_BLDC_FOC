#ifndef __MCU_UART_H
#define __MCU_UART_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 枚举串口通信端口 */
typedef enum {
    DEBUG_COM = 0,
} uart_com_e;

/* 函数声明 */
void drv_uart_send_data(uart_com_e com, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MCU_UART_H */