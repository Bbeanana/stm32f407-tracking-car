#ifndef LAP_TIMER_H
#define LAP_TIMER_H

#include <stdint.h>

/**
 * @brief 圈计时
 *
 * 起点在第二个右直角弯末端。上电后车轮真正开始转动时开始第一圈计时；
 * 摄像头 CURV 检测到 CAR_LAP_CORNERS 个右直角弯算一圈，
 * 记录前 CAR_LAP_COUNT 圈；跑完后锁存停车。障碍物按进入避障停车的次数计数。
 */
void LapTimer_Init(void);
void LapTimer_Update(void);

/**
 * @brief 外部检测到一个右直角弯完成（摄像头 CURV 检测）
 */
void LapTimer_OnRightAngle(void);

/**
 * @brief 是否已经跑完 CAR_LAP_COUNT 圈
 */
uint8_t LapTimer_IsFinished(void);

/**
 * @brief 读取第 index 圈耗时（index 从 0 开始），单位 ms
 */
uint32_t LapTimer_GetLapMs(uint8_t index);

/**
 * @brief 当前圈已计时时间，单位 ms；未发车或已跑完返回 0
 */
uint32_t LapTimer_GetCurrentMs(void);

/**
 * @brief 当前圈号（1 开始）；未发车返回 0，跑完返回 CAR_LAP_COUNT
 */
uint8_t LapTimer_GetCurrentLap(void);

/**
 * @brief 是否正在计时
 */
uint8_t LapTimer_IsRunning(void);

/**
 * @brief 第 index 圈是否已经完成
 */
uint8_t LapTimer_HasLap(uint8_t index);

/**
 * @brief 障碍物触发次数
 */
uint32_t LapTimer_GetObstacleCount(void);

#endif /* LAP_TIMER_H */
