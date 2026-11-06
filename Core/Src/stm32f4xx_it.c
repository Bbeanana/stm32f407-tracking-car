#include "main.h"
#include "stm32f4xx_it.h"

#include "tim.h"
#include "camera.h"

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers                    */
/******************************************************************************/

void NMI_Handler(void)
{
    while (1) {
    }
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief TIM5 输入捕获中断（超声波 ECHO）
  */
void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim5);
}

/**
  * @brief 摄像头 UART 接收中断
  */
void USART1_IRQHandler(void)
{
    Camera_UartIrqHandler();
}
