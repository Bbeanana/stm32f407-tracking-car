#include "bldc_motor.h"

#include <math.h>
#include <stddef.h>

static GPIO_TypeDef *s_stby_port;
static uint16_t s_stby_pin;
static uint8_t s_stby_active_level = 1U;
static uint8_t s_stby_enabled;

/**
 * @brief 初始化整板 STBY 引脚（默认待机）
 */
void BldcDriver_Init(GPIO_TypeDef *stby_port, uint16_t stby_pin,
                     uint8_t active_level)
{
    s_stby_port = stby_port;
    s_stby_pin = stby_pin;
    s_stby_active_level = active_level ? 1U : 0U;
    s_stby_enabled = 0U;

    if (s_stby_port != NULL) {
        GPIO_PinState state = s_stby_active_level ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(s_stby_port, s_stby_pin, state);
    }
}

/**
 * @brief 使能/待机整板
 */
void BldcDriver_Enable(uint8_t enable)
{
    s_stby_enabled = enable ? 1U : 0U;
    if (s_stby_port == NULL) {
        return;
    }

    GPIO_PinState active = s_stby_active_level ? GPIO_PIN_SET : GPIO_PIN_RESET;
    GPIO_PinState inactive = s_stby_active_level ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(s_stby_port, s_stby_pin,
                      s_stby_enabled ? active : inactive);
}

uint8_t BldcDriver_IsEnabled(void)
{
    return s_stby_enabled;
}

/**
 * @brief 初始化单路电机
 */
void BldcMotor_Init(BldcMotor_t *motor, const BldcMotor_Config_t *config)
{
    if (motor == NULL || config == NULL || config->htim == NULL) {
        return;
    }

    motor->cfg = *config;
    motor->output = 0.0f;
    motor->inited = 1U;

    HAL_TIM_PWM_Start(motor->cfg.htim, motor->cfg.pwm_ch);
    BldcMotor_Stop(motor);
}

/**
 * @brief 设置输出（-1.0 ~ +1.0），正值为前进（已通过 invert 统一方向）
 */
void BldcMotor_SetOutput(BldcMotor_t *motor, float output)
{
    if (motor == NULL || !motor->inited) {
        return;
    }

    if (output > 1.0f) {
        output = 1.0f;
    }
    if (output < -1.0f) {
        output = -1.0f;
    }

    float logical = output;

    if (motor->cfg.invert) {
        output = -output;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->cfg.htim) + 1U;
    float duty = fabsf(output);
    uint32_t compare = 0U;

    if (duty < 0.01f) {
        HAL_GPIO_WritePin(motor->cfg.ina_port, motor->cfg.ina_pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->cfg.inb_port, motor->cfg.inb_pin,
                          GPIO_PIN_RESET);
    } else {
        if (duty < motor->cfg.min_duty) {
            duty = motor->cfg.min_duty;
        }
        if (duty > motor->cfg.max_duty) {
            duty = motor->cfg.max_duty;
        }

        compare = (uint32_t)(duty * (float)arr);
        if (compare >= arr) {
            compare = arr - 1U;
        }

        if (output > 0.0f) {
            HAL_GPIO_WritePin(motor->cfg.ina_port, motor->cfg.ina_pin,
                              GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->cfg.inb_port, motor->cfg.inb_pin,
                              GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(motor->cfg.ina_port, motor->cfg.ina_pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->cfg.inb_port, motor->cfg.inb_pin,
                              GPIO_PIN_SET);
        }
    }

    __HAL_TIM_SET_COMPARE(motor->cfg.htim, motor->cfg.pwm_ch, compare);
    motor->output = logical;
}

/**
 * @brief 停止（INA=INB=0，PWM=0）
 */
void BldcMotor_Stop(BldcMotor_t *motor)
{
    if (motor == NULL || !motor->inited) {
        return;
    }

    HAL_GPIO_WritePin(motor->cfg.ina_port, motor->cfg.ina_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->cfg.inb_port, motor->cfg.inb_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->cfg.htim, motor->cfg.pwm_ch, 0U);
    motor->output = 0.0f;
}

float BldcMotor_GetOutput(const BldcMotor_t *motor)
{
    return (motor != NULL) ? motor->output : 0.0f;
}
