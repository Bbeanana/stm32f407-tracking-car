#include "bsp_uart.h"

#include <stddef.h>

static UART_HandleTypeDef *s_huart;

/**
 * @brief 绑定调试串口（本工程为 USART2）
 */
void BSP_UART_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
}

/**
 * @brief 阻塞发送（仅用于调试遥测，不要在控制中断里调用）
 */
void BSP_UART_Send(const uint8_t *data, uint16_t len)
{
    if (s_huart == NULL || data == NULL || len == 0U) {
        return;
    }

    (void)HAL_UART_Transmit(s_huart, (uint8_t *)data, len, 100U);
}

/**
 * @brief newlib printf 重定向到调试串口
 */
int __io_putchar(int ch)
{
    uint8_t byte = (uint8_t)ch;

    if (s_huart != NULL) {
        (void)HAL_UART_Transmit(s_huart, &byte, 1U, 10U);
    }

    return ch;
}
