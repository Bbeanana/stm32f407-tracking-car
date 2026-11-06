#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H

#include <stdint.h>

#include "main.h"

/**
 * @brief 八路数字循迹传感器
 *
 * 通道顺序：通道 0 = 车头最左，通道 7 = 车头最右。
 * 输出极性由 active_level 配置：
 *   active_level = 0：检测到黑线输出低电平（常见）
 *   active_level = 1：检测到黑线输出高电平
 *
 * error 定义：
 *   -1.0 = 线在最左，0 = 线在中间，+1.0 = 线在最右。
 *   全部探头压线（十字/宽黑线）时 all_active=1，error 置 0（直行）。
 *   全部探头丢线时 lost=1，error 保持上一次有效值，便于短暂惯性通过。
 */
typedef struct {
    GPIO_TypeDef *port;
    uint8_t first_pin;
    uint8_t pin_count;
    uint8_t active_level;
    uint8_t reverse_order;      /* 1=物理顺序与通道顺序相反 */
} TrackSensor_Config_t;

typedef struct {
    TrackSensor_Config_t cfg;
    uint8_t raw_mask;           /* 原始电平（未做极性转换） */
    uint8_t active_mask;        /* 已转换极性：1=压线 */
    uint8_t stable_mask;        /* 三取二滤波后的结果 */
    uint8_t history[2];
    float channel[8];           /* 每路一阶低通后的 0~1 权重 */
    float error;
    uint8_t lost;
    uint8_t all_active;
    uint32_t last_seen_tick;
    uint8_t inited;
} TrackSensor_t;

void TrackSensor_Init(TrackSensor_t *sensor, const TrackSensor_Config_t *cfg);
void TrackSensor_Update(TrackSensor_t *sensor);

uint8_t TrackSensor_GetMask(const TrackSensor_t *sensor);
uint8_t TrackSensor_GetRawMask(const TrackSensor_t *sensor);
uint8_t TrackSensor_IsLost(const TrackSensor_t *sensor);
uint8_t TrackSensor_IsAllActive(const TrackSensor_t *sensor);
float TrackSensor_GetError(const TrackSensor_t *sensor);
float TrackSensor_GetChannel(const TrackSensor_t *sensor, uint8_t index);
uint32_t TrackSensor_GetLastSeenTick(const TrackSensor_t *sensor);

#endif /* TRACK_SENSOR_H */
