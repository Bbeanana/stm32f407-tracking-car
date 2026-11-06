#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/**
 * @brief 蜂鸣器（低电平触发）
 *
 * 硬件由 board_config.h 的 BUZZER_PORT / BUZZER_PIN / BUZZER_ACTIVE_LEVEL 决定。
 * 当前默认：PB0，低电平响、高电平停。
 *
 * 用法：
 *   Buzzer_Init();              // 上电初始化，默认关闭
 *   Buzzer_Beep(100U);          // 非阻塞短鸣 100ms
 *   Buzzer_Update();            // 主循环周期调用（用于自动关闭）
 *
 * 注意：Buzzer_Beep() 不会阻塞，但必须周期性调用 Buzzer_Update()，
 *       否则蜂鸣器会一直响。
 */
void Buzzer_Init(void);

void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);

/**
 * @brief 非阻塞蜂鸣：打开蜂鸣器，duration_ms 后由 Buzzer_Update() 自动关闭
 * @param duration_ms 持续时间（ms），0 表示立即关闭
 */
void Buzzer_Beep(uint32_t duration_ms);

/**
 * @brief 周期任务：处理 Buzzer_Beep() 的自动关闭，建议 5~10ms 调用一次
 */
void Buzzer_Update(void);

uint8_t Buzzer_IsOn(void);

#endif /* BUZZER_H */
