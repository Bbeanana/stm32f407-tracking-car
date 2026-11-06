#include "max7219.h"

#include "board_config.h"

/* MAX7219 寄存器地址 */
#define MAX7219_REG_DECODE_MODE   0x09U
#define MAX7219_REG_INTENSITY     0x0AU
#define MAX7219_REG_SCAN_LIMIT    0x0BU
#define MAX7219_REG_SHUTDOWN      0x0CU
#define MAX7219_REG_DISPLAY_TEST  0x0FU

static void Max7219_Write16(uint16_t value)
{
    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET);

    for (uint8_t i = 0U; i < 16U; i++) {
        HAL_GPIO_WritePin(MAX7219_CLK_PORT, MAX7219_CLK_PIN, GPIO_PIN_RESET);

        if (value & (uint16_t)(0x8000U >> i)) {
            HAL_GPIO_WritePin(MAX7219_DIN_PORT, MAX7219_DIN_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(MAX7219_DIN_PORT, MAX7219_DIN_PIN, GPIO_PIN_RESET);
        }

        HAL_GPIO_WritePin(MAX7219_CLK_PORT, MAX7219_CLK_PIN, GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);
}

static void Max7219_Write(uint8_t reg, uint8_t data)
{
    Max7219_Write16((uint16_t)(((uint16_t)reg << 8) | data));
}

#if MAX7219_FLIP_Y
static uint8_t Max7219_Reverse8(uint8_t value)
{
    uint8_t out = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if (value & (uint8_t)(1U << i)) {
            out |= (uint8_t)(1U << (7U - i));
        }
    }

    return out;
}
#endif

void Max7219_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 先保持 CS 高、CLK/DIN 低，再切为输出，避免初始化毛刺 */
    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MAX7219_CLK_PORT, MAX7219_CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MAX7219_DIN_PORT, MAX7219_DIN_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = MAX7219_DIN_PIN | MAX7219_CS_PIN | MAX7219_CLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 关闭测试模式，使用 8x8 全扫描 */
    Max7219_Write(MAX7219_REG_DISPLAY_TEST, 0x00U);
    Max7219_Write(MAX7219_REG_DECODE_MODE, 0x00U);
    Max7219_Write(MAX7219_REG_SCAN_LIMIT, 0x07U);
    Max7219_Write(MAX7219_REG_INTENSITY, (uint8_t)(MAX7219_INTENSITY & 0x0FU));

    /* 关显示状态下清屏，再打开显示 */
    Max7219_Write(MAX7219_REG_SHUTDOWN, 0x00U);
    Max7219_Clear();
    Max7219_Write(MAX7219_REG_SHUTDOWN, 0x01U);
}

void Max7219_SetIntensity(uint8_t intensity)
{
    if (intensity > 15U) {
        intensity = 15U;
    }
    Max7219_Write(MAX7219_REG_INTENSITY, intensity);
}

void Max7219_Clear(void)
{
    for (uint8_t i = 1U; i <= 8U; i++) {
        Max7219_Write(i, 0x00U);
    }
}

void Max7219_Display(const uint8_t columns[8])
{
    for (uint8_t col = 0U; col < 8U; col++) {
        uint8_t bits = columns[col];
        uint8_t digit = (uint8_t)(col + 1U);

#if MAX7219_FLIP_Y
        bits = Max7219_Reverse8(bits);
#endif

#if MAX7219_FLIP_X
        digit = (uint8_t)(8U - col);
#endif

        Max7219_Write(digit, bits);
    }
}
