#include "track_sensor.h"

#include <stddef.h>

#include "board_config.h"
#include "bsp_delay.h"
#include "main.h"

#define TRACK_UART_FRAME_LEN    21U
#define TRACK_UART_PERIOD_MS    20U
#define TRACK_UART_TIMEOUT_MS   10U
#define TRACK_CMD_MANUAL        0U
#define TRACK_CMD_ANALOG        2U
#define TRACK_CMD_THRESHOLD     3U

static UART_HandleTypeDef s_uart;
static uint8_t s_uart_ready;

static uint8_t s_frame[TRACK_UART_FRAME_LEN];
static uint32_t s_last_read_tick;
static uint32_t s_last_threshold_tick;

static uint8_t TrackUartInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = TRACK_UART_TX_PIN | TRACK_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    s_uart.Instance = TRACK_UART_INSTANCE;
    s_uart.Init.BaudRate = TRACK_UART_BAUD;
    s_uart.Init.WordLength = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits = UART_STOPBITS_1;
    s_uart.Init.Parity = UART_PARITY_NONE;
    s_uart.Init.Mode = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&s_uart) != HAL_OK) {
        return 0U;
    }

    return 1U;
}

static void TrackUartFlush(void)
{
    while (__HAL_UART_GET_FLAG(&s_uart, UART_FLAG_RXNE) != RESET) {
        (void)s_uart.Instance->DR;
    }
    __HAL_UART_CLEAR_OREFLAG(&s_uart);
}

static uint8_t TrackUartReadFrame(uint8_t command, uint8_t *data)
{
    if (!s_uart_ready) {
        return 0U;
    }

    TrackUartFlush();

    if (HAL_UART_Transmit(&s_uart, &command, 1U, 5U) != HAL_OK) {
        return 0U;
    }

    if (HAL_UART_Receive(&s_uart, s_frame, TRACK_UART_FRAME_LEN,
                         TRACK_UART_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }

    if (s_frame[0] != 0x55U || s_frame[1] != 0xAAU ||
        s_frame[2] != command || s_frame[3] != 0x10U) {
        return 0U;
    }

    for (uint8_t i = 0U; i < 16U; i++) {
        data[i] = s_frame[4U + i];
    }

    return 1U;
}

static uint8_t TrackSensor_ReadThresholds(TrackSensor_t *sensor)
{
    uint8_t data[16];

    if (!TrackUartReadFrame(TRACK_CMD_THRESHOLD, data)) {
        sensor->threshold_ok = 0U;
        return 0U;
    }

    uint8_t ok = 1U;
    for (uint8_t i = 0U; i < 8U; i++) {
        sensor->threshold[i] = (uint16_t)((uint16_t)data[2U * i] |
                                          ((uint16_t)data[2U * i + 1U] << 8));
        if (sensor->threshold[i] == 0U || sensor->threshold[i] == 0xFFFFU) {
            ok = 0U;
        }
    }

    sensor->threshold_ok = ok;
    return ok;
}

static void TrackSensor_ComputeError(TrackSensor_t *sensor)
{
    uint8_t mask = 0U;
    uint32_t sum = 0U;
    float weighted = 0.0f;

    if (sensor->threshold_ok) {
        for (uint8_t i = 0U; i < 8U; i++) {
            int32_t dark;
#if TRACK_ANALOG_INVERT
            dark = (int32_t)sensor->analog[i] - (int32_t)sensor->threshold[i];
#else
            dark = (int32_t)sensor->threshold[i] - (int32_t)sensor->analog[i];
#endif
            if (dark > 0) {
                mask |= (uint8_t)(1U << i);
                sum += (uint32_t)dark;
#if TRACK_REVERSE_ORDER
                weighted += (float)dark * ((3.5f - (float)i) / 3.5f);
#else
                weighted += (float)dark * (((float)i - 3.5f) / 3.5f);
#endif
            }
        }
    } else {
        uint16_t max_value = 0U;
        uint16_t min_value = 0xFFFFU;

        for (uint8_t i = 0U; i < 8U; i++) {
            if (sensor->analog[i] > max_value) {
                max_value = sensor->analog[i];
            }
            if (sensor->analog[i] < min_value) {
                min_value = sensor->analog[i];
            }
        }

        for (uint8_t i = 0U; i < 8U; i++) {
            int32_t dark;
#if TRACK_ANALOG_INVERT
            dark = (int32_t)sensor->analog[i] - (int32_t)min_value;
#else
            dark = (int32_t)max_value - (int32_t)sensor->analog[i];
#endif
            if (dark > 0) {
                mask |= (uint8_t)(1U << i);
                sum += (uint32_t)dark;
#if TRACK_REVERSE_ORDER
                weighted += (float)dark * ((3.5f - (float)i) / 3.5f);
#else
                weighted += (float)dark * (((float)i - 3.5f) / 3.5f);
#endif
            }
        }
    }

    sensor->active_mask = mask;
    sensor->stable_mask = mask;
    sensor->raw_mask = mask;

    uint8_t count = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        if (mask & (uint8_t)(1U << i)) {
            count++;
        }
    }

    if (count == 0U || sum == 0U) {
        sensor->lost = 1U;
        sensor->all_active = 0U;
        return;
    }

    sensor->lost = 0U;
    sensor->last_seen_tick = HAL_GetTick();

    if (count >= 8U) {
        sensor->all_active = 1U;
        sensor->error = 0.0f;
        return;
    }

    sensor->all_active = 0U;
    sensor->error = weighted / (float)sum;

    if (sensor->error > 1.0f) {
        sensor->error = 1.0f;
    }
    if (sensor->error < -1.0f) {
        sensor->error = -1.0f;
    }
}

/**
 * @brief 初始化 UART 传感器
 *
 * 幻尔八路循迹 UART 上电后只接受一次模式命令：
 * 0=手动模式，之后用 2/3 分别读模拟量/阈值。
 */
void TrackSensor_Init(TrackSensor_t *sensor)
{
    if (sensor == NULL) {
        return;
    }

    sensor->raw_mask = 0U;
    sensor->active_mask = 0U;
    sensor->stable_mask = 0U;
    sensor->error = 0.0f;
    sensor->lost = 1U;
    sensor->all_active = 0U;
    sensor->comm_ok = 0U;
    sensor->threshold_ok = 0U;
    sensor->last_seen_tick = HAL_GetTick();

    for (uint8_t i = 0U; i < 8U; i++) {
        sensor->analog[i] = 0U;
        sensor->threshold[i] = 0U;
    }

    s_uart_ready = TrackUartInit();
    sensor->inited = 1U;

    if (!s_uart_ready) {
        return;
    }

    BSP_DelayUs(500000U);   /* 等传感器上电稳定 */

    uint8_t mode = TRACK_CMD_MANUAL;
    (void)HAL_UART_Transmit(&s_uart, &mode, 1U, 5U);
    BSP_DelayUs(50000U);

    (void)TrackSensor_ReadThresholds(sensor);

    s_last_read_tick = HAL_GetTick();
    s_last_threshold_tick = s_last_read_tick;
}

void TrackSensor_Update(TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_read_tick) < TRACK_UART_PERIOD_MS) {
        return;
    }
    s_last_read_tick = now;

    if (!sensor->threshold_ok &&
        (uint32_t)(now - s_last_threshold_tick) >= 1000U) {
        s_last_threshold_tick = now;
        (void)TrackSensor_ReadThresholds(sensor);
    }

    uint8_t data[16];
    if (!TrackUartReadFrame(TRACK_CMD_ANALOG, data)) {
        sensor->comm_ok = 0U;
        sensor->lost = 1U;
        sensor->all_active = 0U;
        sensor->active_mask = 0U;
        sensor->stable_mask = 0U;
        return;
    }

    sensor->comm_ok = 1U;

    for (uint8_t i = 0U; i < 8U; i++) {
        sensor->analog[i] = (uint16_t)((uint16_t)data[2U * i] |
                                       ((uint16_t)data[2U * i + 1U] << 8));
    }

    TrackSensor_ComputeError(sensor);
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

uint16_t TrackSensor_GetAnalog(const TrackSensor_t *sensor, uint8_t index)
{
    if (sensor == NULL || !sensor->inited || index >= 8U) {
        return 0U;
    }
    return sensor->analog[index];
}

uint8_t TrackSensor_GetCommOk(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->comm_ok;
}

uint8_t TrackSensor_GetThresholdOk(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->threshold_ok;
}

uint32_t TrackSensor_GetLastSeenTick(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->last_seen_tick;
}
