#include "gpio.h"

#include "board_config.h"

/**
 * @brief GPIO 初始化
 *
 * 电机方向：PE7/PE8（左）、PE12/PE13（右），默认低电平停止。
 * STBY：PE14，默认待机。
 * 八路循迹：幻尔 I2C 模块，PB8/PB9 在 track_sensor.c 中单独初始化。
 * 超声波 TRIG：PA1，默认低。
 * 蜂鸣器：PB0 在 buzzer.c 中单独初始化，低电平触发。
 *
 * PWM（PE9/PE11）、编码器（PC6/7、PD12/13）、ECHO（PA0）、
 * 调试串口（PA2/3）、ADC（PC0）在各自外设 MSP 中配置。
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* 电机方向脚默认低（停止） */
    const uint16_t motor_dir_pins = MOTOR_L_INA_PIN | MOTOR_L_INB_PIN |
                                    MOTOR_R_INA_PIN | MOTOR_R_INB_PIN;
    HAL_GPIO_WritePin(MOTOR_L_INA_PORT, motor_dir_pins, GPIO_PIN_RESET);

    /* STBY 默认待机 */
#if MOTOR_STBY_ACTIVE_LEVEL
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
#endif

    GPIO_InitStruct.Pin = motor_dir_pins | MOTOR_STBY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* 超声波 TRIG */
    HAL_GPIO_WritePin(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = ULTRA_TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ULTRA_TRIG_PORT, &GPIO_InitStruct);
}
