#include "ultrasonic.h"

#include <stddef.h>

#include "bsp_delay.h"

#define ULTRA_TRIG_PULSE_US       12U
#define ULTRA_PERIOD_MS           60U   /* HC-SR04 建议测量间隔 ≥60ms */
#define ULTRA_TIMEOUT_MS          60U   /* 无回波/回波过长判定 */

static Ultrasonic_t *s_ultra;

/**
 * @brief 初始化超声波对象并启动输入捕获
 */
void Ultrasonic_Init(Ultrasonic_t *ultra, const Ultrasonic_Config_t *cfg)
{
    if (ultra == NULL || cfg == NULL || cfg->echo_htim == NULL) {
        return;
    }

    ultra->cfg = *cfg;
    ultra->rise_cnt = 0U;
    ultra->width_us = 0U;
    ultra->state = 0U;
    ultra->new_data = 0U;
    ultra->distance_cm = 999.0f;
    ultra->valid = 0U;
    ultra->last_trigger_tick = HAL_GetTick() - ULTRA_PERIOD_MS;
    ultra->inited = 1U;

    s_ultra = ultra;

    HAL_GPIO_WritePin(ultra->cfg.trig_port, ultra->cfg.trig_pin,
                      GPIO_PIN_RESET);
    HAL_TIM_IC_Start_IT(ultra->cfg.echo_htim, ultra->cfg.echo_channel);
}

/**
 * @brief 发送一次 12µs 触发脉冲
 */
static void Ultrasonic_Trigger(Ultrasonic_t *ultra)
{
    if (ultra->state != 0U) {
        return;
    }

    ultra->state = 1U;
    ultra->rise_cnt = 0U;

    HAL_GPIO_WritePin(ultra->cfg.trig_port, ultra->cfg.trig_pin,
                      GPIO_PIN_SET);
    BSP_DelayUs(ULTRA_TRIG_PULSE_US);
    HAL_GPIO_WritePin(ultra->cfg.trig_port, ultra->cfg.trig_pin,
                      GPIO_PIN_RESET);

    ultra->last_trigger_tick = HAL_GetTick();
}

/**
 * @brief 周期任务（建议 5ms）：触发测量 + 超时保护
 */
void Ultrasonic_Update(Ultrasonic_t *ultra)
{
    if (ultra == NULL || !ultra->inited) {
        return;
    }

    uint32_t now = HAL_GetTick();

    if (ultra->state != 0U) {
        if ((uint32_t)(now - ultra->last_trigger_tick) > ULTRA_TIMEOUT_MS) {
            /* 没有回波：恢复到等上升沿，标记本次无效。
             * HC-SR04 在量程内没有障碍物时也会没有回波，因此把距离
             * 置为 999cm（视为“前方空旷”），避免障碍物移开后一直停车。 */
            __HAL_TIM_SET_CAPTUREPOLARITY(ultra->cfg.echo_htim,
                                          ultra->cfg.echo_channel,
                                          TIM_ICPOLARITY_RISING);
            ultra->state = 0U;
            ultra->valid = 0U;
            ultra->distance_cm = 999.0f;
            ultra->new_data = 1U;
        }
        return;
    }

    if ((uint32_t)(now - ultra->last_trigger_tick) >= ULTRA_PERIOD_MS) {
        Ultrasonic_Trigger(ultra);
    }
}

/**
 * @brief TIM 输入捕获回调（由 HAL 在 TIM5 中断中调用）
 *
 * 上升沿记录起始计数，下降沿计算脉宽；TIM5 为 32 位、1MHz，
 * 单次回波远小于计数器回绕时间。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Ultrasonic_t *ultra = s_ultra;

    if (ultra == NULL || htim != ultra->cfg.echo_htim) {
        return;
    }

    uint32_t capture = HAL_TIM_ReadCapturedValue(htim, ultra->cfg.echo_channel);

    if (ultra->state == 1U) {
        ultra->rise_cnt = capture;
        ultra->state = 2U;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ultra->cfg.echo_channel,
                                      TIM_ICPOLARITY_FALLING);
    } else if (ultra->state == 2U) {
        uint32_t width = capture - ultra->rise_cnt;
        float distance = (float)width / 58.0f;

        ultra->width_us = width;
        ultra->distance_cm = distance;
        ultra->valid = (distance >= 2.0f && distance <= 400.0f) ? 1U : 0U;
        ultra->new_data = 1U;
        ultra->state = 0U;

        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ultra->cfg.echo_channel,
                                      TIM_ICPOLARITY_RISING);
    }
}

uint8_t Ultrasonic_HasNewData(Ultrasonic_t *ultra)
{
    if (ultra == NULL || !ultra->new_data) {
        return 0U;
    }

    ultra->new_data = 0U;
    return 1U;
}

float Ultrasonic_GetDistanceCm(const Ultrasonic_t *ultra)
{
    if (ultra == NULL || !ultra->inited) {
        return 999.0f;
    }
    return ultra->distance_cm;
}

uint8_t Ultrasonic_IsValid(const Ultrasonic_t *ultra)
{
    return (ultra != NULL) ? ultra->valid : 0U;
}
