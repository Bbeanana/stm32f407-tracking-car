#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "main.h"

/**
 * @brief 正交编码器对象（定时器编码器模式，AB 相）
 *
 * 前置条件：CubeMX/手写初始化中把定时器配置为 Encoder Mode（TI1+TI2，4 倍频）。
 *   本工程使用 16 位定时器 TIM3（左）、TIM4（右）。
 *
 * counts_per_rev：输出轴（车轮轴）转一圈的计数
 *   = 编码器线数 × 4（正交四倍频）× 减速比
 *   建议实测标定：手转车轮一圈，看 total 增量。
 */
typedef struct {
    TIM_HandleTypeDef *htim;    /* 编码器模式定时器 */
    int32_t counts_per_rev;     /* 输出轴一圈计数 */
    float   radius_m;           /* 车轮半径，用于换算线速度 */
    uint8_t invert;             /* 计数方向取反 */

    int16_t last_count;         /* 上次原始计数 */
    int32_t total_count;        /* 累计计数（含方向取反） */
    int32_t delta;              /* 本周期增量 */
    float   speed_radps;        /* 车轮角速度 (rad/s) */
    float   speed_mps;          /* 车轮线速度 (m/s) */
    uint8_t inited;
} Encoder_t;

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim,
                  int32_t counts_per_rev, float radius_m, uint8_t invert);
void Encoder_Update(Encoder_t *enc, float dt_s);   /* 固定周期调用 */
void Encoder_Reset(Encoder_t *enc);
int32_t Encoder_GetDelta(const Encoder_t *enc);
int32_t Encoder_GetTotal(const Encoder_t *enc);
float Encoder_GetSpeedRadps(const Encoder_t *enc);
float Encoder_GetSpeedMps(const Encoder_t *enc);

#endif /* ENCODER_H */
