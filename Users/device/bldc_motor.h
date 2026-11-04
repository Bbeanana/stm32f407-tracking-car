#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include <stdint.h>

#include "main.h"

/**
 * @brief 无刷电机驱动对象（PWM + INA/INB 型双路无刷驱动板）
 *
 * 控制真值表（按常见驱动板约定，实测不对时修改 invert）：
 *   正转：INA=1, INB=0, PWM=占空比
 *   反转：INA=0, INB=1, PWM=占空比
 *   停止：INA=0, INB=0, PWM=0
 *
 * STBY：高电平使能（默认），低电平整板待机。
 */

typedef struct {
    TIM_HandleTypeDef *htim;   /* PWM 定时器句柄 */
    uint32_t pwm_ch;           /* PWM 通道 */
    GPIO_TypeDef *ina_port;    /* INA 端口 */
    uint16_t ina_pin;          /* INA 引脚 */
    GPIO_TypeDef *inb_port;    /* INB 端口 */
    uint16_t inb_pin;          /* INB 引脚 */
    uint8_t invert;            /* 方向取反（左右镜像安装时使用） */
    float min_duty;            /* 最小有效占空比 */
    float max_duty;            /* 最大占空比限幅 */
} BldcMotor_Config_t;

typedef struct {
    BldcMotor_Config_t cfg;
    float output;              /* 最近一次逻辑输出（-1.0 ~ +1.0） */
    uint8_t inited;
} BldcMotor_t;

/* 整板 STBY 控制 */
void BldcDriver_Init(GPIO_TypeDef *stby_port, uint16_t stby_pin,
                     uint8_t active_level);
void BldcDriver_Enable(uint8_t enable);
uint8_t BldcDriver_IsEnabled(void);

/* 单路电机控制 */
void BldcMotor_Init(BldcMotor_t *motor, const BldcMotor_Config_t *config);
void BldcMotor_SetOutput(BldcMotor_t *motor, float output);
void BldcMotor_Stop(BldcMotor_t *motor);
float BldcMotor_GetOutput(const BldcMotor_t *motor);

#endif /* BLDC_MOTOR_H */
