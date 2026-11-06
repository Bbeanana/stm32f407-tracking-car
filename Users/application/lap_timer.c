#include "lap_timer.h"

#include "main.h"
#include "car_config.h"
#include "car_control.h"

static CarState_t s_prev_state;

static uint32_t s_lap_start_ms;
static uint32_t s_lap_ms[CAR_LAP_COUNT];
static uint8_t s_lap_valid[CAR_LAP_COUNT];

static uint8_t s_lap_index;
static uint8_t s_corner_count;
static uint8_t s_corner_seen;
static uint32_t s_last_corner_ms;
static uint8_t s_running;
static uint8_t s_started;

static uint32_t s_obstacle_count;

void LapTimer_Init(void)
{
    s_prev_state = Car_GetState();

    s_lap_start_ms = 0U;
    s_lap_index = 0U;
    s_corner_count = 0U;
    s_corner_seen = 0U;
    s_last_corner_ms = 0U;
    s_running = 0U;
    s_started = 0U;
    s_obstacle_count = 0U;

    for (uint8_t i = 0U; i < CAR_LAP_COUNT; i++) {
        s_lap_ms[i] = 0U;
        s_lap_valid[i] = 0U;
    }
}

void LapTimer_Update(void)
{
    CarState_t state = Car_GetState();

    /* 上电后第一次进入 LINE 状态视为发车，开始第一圈计时 */
    if (state == CAR_STATE_LINE && s_prev_state == CAR_STATE_INIT) {
        if (!s_started) {
            s_started = 1U;
            s_running = 1U;
            s_lap_index = 0U;
            s_corner_count = 0U;
            s_corner_seen = 0U;
            s_lap_start_ms = HAL_GetTick();
        }
    }

    /* 进入避障 STOP 时计数一次；同一个障碍物不会重复累加 */
    if (state == CAR_STATE_STOP && s_prev_state != CAR_STATE_STOP &&
        Car_IsObstacle()) {
        s_obstacle_count++;
    }

    s_prev_state = state;
}

/**
 * @brief 外部（摄像头 CURV）判定一个直角弯完成
 */
void LapTimer_OnRightAngle(void)
{
    if (!s_running) {
        return;
    }

    uint32_t now = HAL_GetTick();

    /* 同一个弯道短时间内不重复计数 */
    if (s_corner_seen &&
        (uint32_t)(now - s_last_corner_ms) < CAR_LAP_CORNER_DEBOUNCE_MS) {
        return;
    }

    s_corner_seen = 1U;
    s_last_corner_ms = now;
    s_corner_count++;

    if (s_corner_count >= CAR_LAP_CORNERS &&
        s_lap_index < CAR_LAP_COUNT) {
        s_lap_ms[s_lap_index] = now - s_lap_start_ms;
        s_lap_valid[s_lap_index] = 1U;

        s_lap_start_ms = now;
        s_corner_count = 0U;
        s_lap_index++;

        if (s_lap_index >= CAR_LAP_COUNT) {
            s_running = 0U;
        }
    }
}

uint8_t LapTimer_IsFinished(void)
{
    return (s_started && !s_running && s_lap_index >= CAR_LAP_COUNT) ? 1U : 0U;
}

uint32_t LapTimer_GetLapMs(uint8_t index)
{
    if (index >= CAR_LAP_COUNT) {
        return 0U;
    }
    return s_lap_ms[index];
}

uint32_t LapTimer_GetCurrentMs(void)
{
    if (!s_started || !s_running) {
        return 0U;
    }
    return (uint32_t)(HAL_GetTick() - s_lap_start_ms);
}

uint8_t LapTimer_GetCurrentLap(void)
{
    if (!s_started) {
        return 0U;
    }
    if (s_lap_index >= CAR_LAP_COUNT) {
        return CAR_LAP_COUNT;
    }
    return (uint8_t)(s_lap_index + 1U);
}

uint8_t LapTimer_IsRunning(void)
{
    return s_running;
}

uint8_t LapTimer_HasLap(uint8_t index)
{
    if (index >= CAR_LAP_COUNT) {
        return 0U;
    }
    return s_lap_valid[index];
}

uint32_t LapTimer_GetObstacleCount(void)
{
    return s_obstacle_count;
}
