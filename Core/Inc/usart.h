#ifndef __USART_H__
#define __USART_H__

#include "main.h"

/* 调试串口：USART2，PA2=TX，PA3=RX，115200 8N1 */
extern UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void);

#endif /* __USART_H__ */
