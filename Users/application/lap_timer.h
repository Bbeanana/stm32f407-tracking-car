#ifndef LAP_TIMER_H
#define LAP_TIMER_H

#include <stdint.h>

/**
 * @brief 圈计时
 *
 * 起点在第二个右直角弯末端。上电后第一次进入 LINE 状态开始计时；
 * 每完成 CAR_LAP_CORNERS 个右直角弯（SEARCH -> LINE）算一圈，
 * 记录前 CAR_LAP_COUNT 圈。障碍物按进入避障停车的次数计数。
 */
void LapTimer_Init(void);
void LapTimer_Update(void);

/**
 * @brief 读取第 index 圈耗时（index 从 0 开始），单位 ms
 */
uint32_t LapTimer_GetLapMs(uint8_t index);

/**
 * @brief 第 index 圈是否已经完成
 */
uint8_t LapTimer_HasLap(uint8_t index);

/**
 * @brief 障碍物触发次数
 */
uint32_t LapTimer_GetObstacleCount(void);

#endif /* LAP_TIMER_H */
