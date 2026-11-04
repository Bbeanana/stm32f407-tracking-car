#ifndef __TIM_H__
#define __TIM_H__

#include "main.h"

/* =========================================================================
 * 定时器配置：
 *   TIM1：2 路 PWM，20kHz
 *         CH1=PE9（左电机 PWM）  CH2=PE11（右电机 PWM）
 *   TIM3：编码器模式（左电机）  CH1=PC6  CH2=PC7
 *   TIM4：编码器模式（右电机）  CH1=PD12 CH2=PD13
 *   TIM5：1MHz 输入捕获（超声波 ECHO） CH1=PA0
 * ========================================================================= */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);

#endif /* __TIM_H__ */
