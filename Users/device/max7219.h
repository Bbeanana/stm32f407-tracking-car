#ifndef MAX7219_H
#define MAX7219_H

#include <stdint.h>

/**
 * @brief MAX7219 8x8 LED 点阵（软件模拟 SPI）
 *
 * 默认接线见 board_config.h：
 *   DIN=PB10，CS/LOAD=PB12，CLK=PB13。
 * 一个模块只需要 3 根信号线；VCC/GND 按模块要求接 5V 和 GND。
 */
void Max7219_Init(void);

/**
 * @brief 设置亮度，0~15
 */
void Max7219_SetIntensity(uint8_t intensity);

/**
 * @brief 清空显示
 */
void Max7219_Clear(void);

/**
 * @brief 显示 8 列数据
 * @param columns columns[0] 是最左列，每一位对应一行（bit0=最上行）
 */
void Max7219_Display(const uint8_t columns[8]);

#endif /* MAX7219_H */
