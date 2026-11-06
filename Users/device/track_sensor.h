#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H

#include <stdint.h>

/**
 * @brief 幻尔八路循迹传感器（UART 模拟量）
 *
 * USART1：PA9=TX，PA10=RX，115200 8N1。
 * 传感器 TX 接 PA10，传感器 RX 接 PA9。
 *
 * 使用手动模式：上电后发送 0 进入手动模式，之后每次发送 2
 * 读取 8 路模拟量。返回帧：
 *   0x55 0xAA 0x02 0x10 + 8×uint16(低字节在前) + 校验
 *
 * 偏差定义：
 *   -1.0 = 线在最左，0 = 线在中间，+1.0 = 线在最右。
 */
typedef struct {
    uint8_t raw_mask;       /* 预留：数字量掩码 */
    uint8_t active_mask;    /* 按模拟值/阈值判断的压线掩码 */
    uint8_t stable_mask;    /* 与 active_mask 相同，保留兼容接口 */
    uint16_t analog[8];     /* 8 路模拟值 */
    uint16_t threshold[8];  /* 8 路阈值 */
    float error;
    uint8_t lost;
    uint8_t all_active;
    uint8_t comm_ok;
    uint8_t threshold_ok;
    uint32_t last_seen_tick;
    uint8_t inited;
} TrackSensor_t;

void TrackSensor_Init(TrackSensor_t *sensor);
void TrackSensor_Update(TrackSensor_t *sensor);

uint8_t TrackSensor_GetMask(const TrackSensor_t *sensor);
uint8_t TrackSensor_GetRawMask(const TrackSensor_t *sensor);
uint8_t TrackSensor_IsLost(const TrackSensor_t *sensor);
uint8_t TrackSensor_IsAllActive(const TrackSensor_t *sensor);
float TrackSensor_GetError(const TrackSensor_t *sensor);
uint16_t TrackSensor_GetAnalog(const TrackSensor_t *sensor, uint8_t index);
uint8_t TrackSensor_GetCommOk(const TrackSensor_t *sensor);
uint8_t TrackSensor_GetThresholdOk(const TrackSensor_t *sensor);
uint32_t TrackSensor_GetLastSeenTick(const TrackSensor_t *sensor);

#endif /* TRACK_SENSOR_H */
