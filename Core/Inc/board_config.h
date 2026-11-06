#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"
#include "tim.h"
#include "usart.h"

/* =========================================================================
 * 新小车硬件引脚表（STM32F407IGHx，可按实际接线修改本文件）
 *
 * 说明：
 *   - 电机驱动板每路需要 PWM + INA + INB，两路共用一个 STBY。
 *   - EA1/EA2、EB1/EB2 按两路编码器 A/B 使用。
 *   - 八路循迹使用 PD2~PD9，低 8 位连续排列，便于一次读取。
 *   - 超声波 HC-SR04：PA1=TRIG，PA0=TIM5_CH1=ECHO。
 *   - 驱动板 ADC 输出接 PC0（ADC1_IN10），仅作遥测，不参与控制。
 *   - 调试串口 USART2：PA2=TX，PA3=RX，115200。
 *   - 蜂鸣器接 PB0，低电平触发（当前工程未占用 PB0）。
 * ========================================================================= */

/* ---------------------------- 左电机（A 路） ---------------------------- */
#define MOTOR_L_PWM_TIM          (&htim1)
#define MOTOR_L_PWM_CH           TIM_CHANNEL_1        /* PE9  TIM1_CH1 */
#define MOTOR_L_INA_PORT         GPIOE
#define MOTOR_L_INA_PIN          GPIO_PIN_7
#define MOTOR_L_INB_PORT         GPIOE
#define MOTOR_L_INB_PIN          GPIO_PIN_8

/* ---------------------------- 右电机（B 路） ---------------------------- */
#define MOTOR_R_PWM_TIM          (&htim1)
#define MOTOR_R_PWM_CH           TIM_CHANNEL_2        /* PE11 TIM1_CH2 */
#define MOTOR_R_INA_PORT         GPIOE
#define MOTOR_R_INA_PIN          GPIO_PIN_12
#define MOTOR_R_INB_PORT         GPIOE
#define MOTOR_R_INB_PIN          GPIO_PIN_13

/* ------------------------------ 驱动板 STBY ----------------------------- */
#define MOTOR_STBY_PORT          GPIOE
#define MOTOR_STBY_PIN           GPIO_PIN_14
#define MOTOR_STBY_ACTIVE_LEVEL  1U                    /* 1=高电平使能 */

/* ------------------------------- 编码器 -------------------------------- */
#define ENC_L_TIM                (&htim3)             /* PC6/PC7  TIM3_CH1/CH2 */
#define ENC_R_TIM                (&htim4)             /* PD12/PD13 TIM4_CH1/CH2 */

/* ---------------------------- 八路循迹输入 ------------------------------ */
/* 通道 0 = PD2（车头最左），通道 7 = PD9（车头最右） */
#define TRACK_PORT               GPIOD
#define TRACK_FIRST_PIN          2U
#define TRACK_PIN_COUNT          8U

/* ------------------------------- 超声波 -------------------------------- */
#define ULTRA_TRIG_PORT          GPIOA
#define ULTRA_TRIG_PIN           GPIO_PIN_1
#define ULTRA_ECHO_TIM           (&htim5)
#define ULTRA_ECHO_CH            TIM_CHANNEL_1         /* PA0 TIM5_CH1 */

/* --------------------------- 驱动板 ADC 输出 ---------------------------- */
#define CURRENT_ADC_CHANNEL      10U                   /* PC0 ADC1_IN10 */
#define CURRENT_ADC_PORT         GPIOC
#define CURRENT_ADC_PIN          GPIO_PIN_0

/* ------------------------------- 蜂鸣器 -------------------------------- */
/* 当前固件未使用 PB0；蜂鸣器模块 I/O 接 PB0。
 * BUZZER_ACTIVE_LEVEL = 0：低电平触发（本车用）
 *                       1：高电平触发（改为 1 即可，不用改驱动代码） */
#define BUZZER_PORT              GPIOB
#define BUZZER_PIN               GPIO_PIN_0
#define BUZZER_ACTIVE_LEVEL      0U

/* --------------------------- MAX7219 8x8 点阵 ---------------------------- */
/* 单个 8x8 模块，软件模拟 SPI。PB10/PB12/PB13 当前工程未占用。 */
#define MAX7219_DIN_PORT         GPIOB
#define MAX7219_DIN_PIN          GPIO_PIN_10
#define MAX7219_CS_PORT          GPIOB
#define MAX7219_CS_PIN           GPIO_PIN_12
#define MAX7219_CLK_PORT         GPIOB
#define MAX7219_CLK_PIN          GPIO_PIN_13
#define MAX7219_INTENSITY        6U              /* 0~15 */
/* 如果显示上下/左右反了，改这两个开关，不用拆模块 */
#define MAX7219_FLIP_X           0U
#define MAX7219_FLIP_Y           0U

/* ------------------------------- 调试串口 ------------------------------- */
#define DEBUG_UART_HANDLE        (&huart2)

#endif /* BOARD_CONFIG_H */
