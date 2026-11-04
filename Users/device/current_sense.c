#include "current_sense.h"

#include "board_config.h"
#include "adc.h"

/**
 * @brief 初始化（ADC 已在 MX_ADC1_Init 中完成）
 */
void CurrentSense_Init(void)
{
    /* 预留：若以后需要零点校准/电流换算，在这里做 */
}

/**
 * @brief 读取一次 ADC 原始值（0~4095）
 */
uint16_t CurrentSense_ReadRaw(void)
{
    return ADC1_ReadChannel10();
}

/**
 * @brief 读取驱动板 ADC 输出电压（V）
 */
float CurrentSense_ReadVolts(void)
{
    uint16_t raw = ADC1_ReadChannel10();
    return (float)raw * 3.3f / 4095.0f;
}
