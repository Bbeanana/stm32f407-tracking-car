#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

/**
 * @brief 128x64 I2C OLED（SSD1306）
 *
 * 默认接线见 board_config.h：PB10=SCL，PB11=SDA，地址 0x3C。
 * 驱动使用软件 I2C，先写显存，再由 Ssd1306_Update() 分块刷新，
 * 避免一次刷新 1KB 显存长时间阻塞 5ms 电机控制循环。
 */
void Ssd1306_Init(void);

/**
 * @brief 只清空显存，不立即刷新
 */
void Ssd1306_ClearBuffer(void);

/**
 * @brief 在显存中画字符串（5x7 字库，scale=1/2）
 */
void Ssd1306_DrawString(uint8_t x, uint8_t y, const char *text, uint8_t scale);

/**
 * @brief 请求把显存刷新到屏幕（非阻塞，分块进行）
 */
void Ssd1306_RequestUpdate(void);

/**
 * @brief 周期调用，每次发送一小块数据；建议 5ms 调用一次
 */
void Ssd1306_Update(void);

#endif /* SSD1306_H */
