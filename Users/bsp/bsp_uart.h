#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

#include "usart.h"

void BSP_UART_Init(UART_HandleTypeDef *huart);
void BSP_UART_Send(const uint8_t *data, uint16_t len);

#endif /* BSP_UART_H */
