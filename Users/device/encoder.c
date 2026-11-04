#include "encoder.h"

#define ENCODER_TWO_PI 6.2831853f

/**
 * @brief 初始化编码器（启动编码器模式定时器并清零计数）
 */
void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim,
                  int32_t counts_per_rev, float radius_m, uint8_t invert)
{
    if (enc == NULL || htim == NULL || counts_per_rev <= 0) {
        return;
    }

    enc->htim = htim;
    enc->counts_per_rev = counts_per_rev;
    enc->radius_m = radius_m;
    enc->invert = invert;

    enc->last_count = 0;
    enc->total_count = 0;
    enc->delta = 0;
    enc->speed_radps = 0.0f;
    enc->speed_mps = 0.0f;
    enc->inited = 1U;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim, 0);
}

/**
 * @brief 周期更新：读取计数增量并计算角速度/线速度
 *
 * @param dt_s 调用周期（秒），需与速度环调用周期一致
 */
void Encoder_Update(Encoder_t *enc, float dt_s)
{
    if (enc == NULL || !enc->inited || dt_s <= 0.0f) {
        return;
    }

    /* 16 位定时器计数，减法天然支持回绕 */
    int16_t raw = (int16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    int16_t step = (int16_t)(raw - enc->last_count);
    enc->last_count = raw;

    int32_t delta = (int32_t)step;
    if (enc->invert) {
        delta = -delta;
    }

    enc->delta = delta;
    enc->total_count += delta;

    enc->speed_radps =
        (float)delta / (float)enc->counts_per_rev * ENCODER_TWO_PI / dt_s;
    enc->speed_mps = enc->speed_radps * enc->radius_m;
}

void Encoder_Reset(Encoder_t *enc)
{
    if (enc == NULL || !enc->inited) {
        return;
    }

    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->last_count = 0;
    enc->total_count = 0;
    enc->delta = 0;
    enc->speed_radps = 0.0f;
    enc->speed_mps = 0.0f;
}

int32_t Encoder_GetDelta(const Encoder_t *enc)
{
    return (enc != NULL) ? enc->delta : 0;
}

int32_t Encoder_GetTotal(const Encoder_t *enc)
{
    return (enc != NULL) ? enc->total_count : 0;
}

float Encoder_GetSpeedRadps(const Encoder_t *enc)
{
    return (enc != NULL) ? enc->speed_radps : 0.0f;
}

float Encoder_GetSpeedMps(const Encoder_t *enc)
{
    return (enc != NULL) ? enc->speed_mps : 0.0f;
}
