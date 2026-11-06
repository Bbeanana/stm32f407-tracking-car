/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 两路无刷电机 + 八路循迹 + 超声波避障小车主控
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "board_config.h"
#include "bsp_delay.h"
#include "bsp_uart.h"
#include "buzzer.h"
#include "car_control.h"
#include "current_sense.h"
#include "lap_timer.h"
#include "oled_display.h"
#include "telemetry.h"

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* DWT 微秒延时必须先初始化（ADC 初始化里会用到） */
    BSP_Delay_Init();

    /* 外设初始化 */
    MX_GPIO_Init();
    MX_TIM1_Init();     /* 两路 20kHz PWM */
    MX_TIM3_Init();     /* 左编码器 */
    MX_TIM4_Init();     /* 右编码器 */
    MX_TIM5_Init();     /* 超声波 ECHO 输入捕获 */
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    Buzzer_Init();      /* 蜂鸣器 PB0，低电平触发，默认关闭 */

    BSP_UART_Init(DEBUG_UART_HANDLE);
    CurrentSense_Init();

    /* 应用初始化：电机、编码器、循迹、超声波、圈计时、OLED */
    Car_Init();
    LapTimer_Init();
    OledDisplay_Init();
    Telemetry_Init();

    /* 上电短鸣一声，用来确认蜂鸣器接线/极性正确 */
    Buzzer_Beep(120U);

    uint32_t last_tick = HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = (uint32_t)(now - last_tick);

        if (elapsed >= 5U) {
            last_tick = now;

            /* 用实际周期，避免串口打印等阻塞导致编码器测速偏大 */
            float dt = (float)elapsed * 0.001f;
            if (dt > 0.020f) {
                dt = 0.020f;
            }

            Car_Loop(dt);
            LapTimer_Update();      /* 检测发车、直角弯完成、障碍物事件 */
            OledDisplay_Update();   /* 数据变化时重绘，并分块刷新显存 */
            Buzzer_Update();        /* 处理 Buzzer_Beep() 的定时关闭 */
            Telemetry_Update();
        }
    }
}

/**
  * @brief 系统时钟：HSE 8MHz → PLL → 168MHz
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
