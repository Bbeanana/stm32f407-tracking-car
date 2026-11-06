#include "track_sensor.h"

#include <stddef.h>

#include "board_config.h"

static uint8_t TrackSensor_ReverseBits(uint8_t value, uint8_t count)
{
    uint8_t out = 0U;

    for (uint8_t i = 0U; i < count; i++) {
        if (value & (1U << i)) {
            out |= (uint8_t)(1U << ((count - 1U) - i));
        }
    }

    return out;
}

static uint8_t TrackSensor_CountBits(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count += (uint8_t)(value & 1U);
        value >>= 1U;
    }

    return count;
}

/**
 * @brief 初始化循迹传感器
 */
void TrackSensor_Init(TrackSensor_t *sensor, const TrackSensor_Config_t *cfg)
{
    if (sensor == NULL || cfg == NULL || cfg->port == NULL ||
        cfg->pin_count < 2U || cfg->pin_count > 8U) {
        return;
    }

    sensor->cfg = *cfg;
    sensor->raw_mask = 0U;
    sensor->active_mask = 0U;
    sensor->stable_mask = 0U;
    sensor->history[0] = 0U;
    sensor->history[1] = 0U;
    sensor->error = 0.0f;
    sensor->lost = 1U;
    sensor->all_active = 0U;
    sensor->last_seen_tick = HAL_GetTick();
    sensor->inited = 1U;

    /* 先读一次，避免上电后三取二滤波器需要三个周期才稳定 */
    uint8_t mask = (uint8_t)((sensor->cfg.port->IDR >> sensor->cfg.first_pin) &
                             ((1U << sensor->cfg.pin_count) - 1U));
    if (sensor->cfg.reverse_order) {
        mask = TrackSensor_ReverseBits(mask, sensor->cfg.pin_count);
    }
    sensor->raw_mask = mask;
    if (sensor->cfg.active_level == 0U) {
        mask = (uint8_t)(~mask & ((1U << sensor->cfg.pin_count) - 1U));
    }
    sensor->active_mask = mask;
    sensor->history[0] = mask;
    sensor->history[1] = mask;
    sensor->stable_mask = mask;

    /* 用初始状态填满低通值，避免上电第一拍从 0 慢慢爬升 */
    for (uint8_t i = 0U; i < 8U; i++) {
        sensor->channel[i] = (mask & (uint8_t)(1U << i)) ? 1.0f : 0.0f;
    }
}

/**
 * @brief 周期更新（建议 5ms 调用一次）
 */
void TrackSensor_Update(TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return;
    }

    const uint8_t full_mask =
        (uint8_t)((1U << sensor->cfg.pin_count) - 1U);

    uint8_t raw = (uint8_t)((sensor->cfg.port->IDR >> sensor->cfg.first_pin) &
                            full_mask);
    if (sensor->cfg.reverse_order) {
        raw = TrackSensor_ReverseBits(raw, sensor->cfg.pin_count);
    }
    sensor->raw_mask = raw;

    uint8_t active = raw;
    if (sensor->cfg.active_level == 0U) {
        active = (uint8_t)(~active & full_mask);
    }
    sensor->active_mask = active;

    /* 三取二滤波：抑制单次毛刺 */
    uint8_t h0 = sensor->history[0];
    uint8_t h1 = sensor->history[1];
    uint8_t stable = (uint8_t)((h0 & h1) | (h0 & active) | (h1 & active));
    sensor->history[0] = h1;
    sensor->history[1] = active;
    sensor->stable_mask = stable;

    /* 每路一阶低通：把数字 0/1 变成连续的 0~1 权重 */
    for (uint8_t i = 0U; i < 8U; i++) {
        float target = (stable & (uint8_t)(1U << i)) ? 1.0f : 0.0f;
        sensor->channel[i] +=
            TRACK_LPF_ALPHA * (target - sensor->channel[i]);
    }

    uint8_t count = TrackSensor_CountBits(stable);
    if (count == 0U) {
        sensor->lost = 1U;
        sensor->all_active = 0U;
        return;
    }

    sensor->lost = 0U;
    sensor->last_seen_tick = HAL_GetTick();

    if (count >= sensor->cfg.pin_count) {
        sensor->all_active = 1U;
        sensor->error = 0.0f;
        return;
    }

    sensor->all_active = 0U;

    const float center = ((float)sensor->cfg.pin_count - 1.0f) * 0.5f;
    float sum = 0.0f;
    float weighted = 0.0f;

    for (uint8_t i = 0U; i < sensor->cfg.pin_count; i++) {
        float value = sensor->channel[i];
        sum += value;
        weighted += value * (((float)i - center) / center);
    }

    if (sum > 0.0001f) {
        sensor->error = weighted / sum;
    }
}

uint8_t TrackSensor_GetMask(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->stable_mask;
}

uint8_t TrackSensor_GetRawMask(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->raw_mask;
}

uint8_t TrackSensor_IsLost(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 1U;
    }
    return sensor->lost;
}

uint8_t TrackSensor_IsAllActive(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->all_active;
}

float TrackSensor_GetError(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0.0f;
    }
    return sensor->error;
}

float TrackSensor_GetChannel(const TrackSensor_t *sensor, uint8_t index)
{
    if (sensor == NULL || !sensor->inited || index >= 8U) {
        return 0.0f;
    }
    return sensor->channel[index];
}

uint32_t TrackSensor_GetLastSeenTick(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->last_seen_tick;
}
