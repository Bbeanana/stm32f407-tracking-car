#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H

#include <stdint.h>

/**
 * @brief 幻尔八路循迹传感器（I2C 模拟量）
 *
 * 独立 I2C：PB8=SCL，PB9=SDA，地址 0x5D。
 * 读取 8 路模拟值和阈值，按暗线权重计算连续偏差：
 *   -1.0 = 线在最左，0 = 线在中间，+1.0 = 线在最右。
 *
 * I2C 寄存器：
 *   0x05        8 路数字结果（1 字节）
 *   0x06..0x14  8 路模拟值（每路 uint16，低字节在前）
 *   0x16..0x24  8 路阈值（每路 uint16，低字节在前）
 */
typedef struct {
    uint8_t raw_mask;       /* I2C 数字结果，bit0=通道 1 */
    uint8_t active_mask;    /* 按模拟值/阈值判断的压线掩码 */
    uint8_t stable_mask;    /* 与 active_mask 相同，保留兼容接口 */
    uint16_t analog[8];     /* 8 路模拟值 */
    uint16_t threshold[8];  /* 8 路阈值 */
    float error;
    uint8_t lost;
    uint8_t all_active;
    uint8_t i2c_ok;
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
uint32_t TrackSensor_GetLastSeenTick(const TrackSensor_t *sensor);

#endif /* TRACK_SENSOR_H */
