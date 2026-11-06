#include "track_sensor.h"

#include <math.h>
#include <stddef.h>

#include "board_config.h"
#include "bsp_delay.h"
#include "main.h"

#define TRACK_I2C_DELAY_US      2U
#define TRACK_READ_PERIOD_MS    10U
#define TRACK_REG_DIGITAL       0x05U
#define TRACK_REG_ANALOG        0x06U
#define TRACK_REG_THRESHOLD     0x16U
#define TRACK_ADDR_WRITE        ((uint8_t)(TRACK_I2C_ADDR << 1))
#define TRACK_ADDR_READ         ((uint8_t)(TRACK_ADDR_WRITE | 1U))

static uint32_t s_last_read_tick;
static uint32_t s_last_threshold_tick;

static void TrackSclHigh(void)
{
    HAL_GPIO_WritePin(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_SET);
}

static void TrackSclLow(void)
{
    HAL_GPIO_WritePin(TRACK_I2C_SCL_PORT, TRACK_I2C_SCL_PIN, GPIO_PIN_RESET);
}

static void TrackSdaHigh(void)
{
    HAL_GPIO_WritePin(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_SET);
}

static void TrackSdaLow(void)
{
    HAL_GPIO_WritePin(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN, GPIO_PIN_RESET);
}

static uint8_t TrackSdaRead(void)
{
    return (HAL_GPIO_ReadPin(TRACK_I2C_SDA_PORT, TRACK_I2C_SDA_PIN) ==
            GPIO_PIN_SET) ? 1U : 0U;
}

static void TrackI2cStart(void)
{
    TrackSdaHigh();
    TrackSclHigh();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSdaLow();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSclLow();
    BSP_DelayUs(TRACK_I2C_DELAY_US);
}

static void TrackI2cStop(void)
{
    TrackSdaLow();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSclHigh();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSdaHigh();
    BSP_DelayUs(TRACK_I2C_DELAY_US);
}

/* 返回 0=收到 ACK，1=NACK/无器件 */
static uint8_t TrackI2cWriteByte(uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        if (value & 0x80U) {
            TrackSdaHigh();
        } else {
            TrackSdaLow();
        }
        BSP_DelayUs(TRACK_I2C_DELAY_US);

        TrackSclHigh();
        BSP_DelayUs(TRACK_I2C_DELAY_US);

        TrackSclLow();
        BSP_DelayUs(TRACK_I2C_DELAY_US);

        value <<= 1;
    }

    TrackSdaHigh();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSclHigh();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    uint8_t nack = TrackSdaRead();

    TrackSclLow();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    return nack;
}

static uint8_t TrackI2cReadByte(uint8_t ack)
{
    uint8_t value = 0U;

    TrackSdaHigh();

    for (uint8_t i = 0U; i < 8U; i++) {
        TrackSclHigh();
        BSP_DelayUs(TRACK_I2C_DELAY_US);

        value = (uint8_t)((value << 1) | TrackSdaRead());

        TrackSclLow();
        BSP_DelayUs(TRACK_I2C_DELAY_US);
    }

    if (ack) {
        TrackSdaLow();
    } else {
        TrackSdaHigh();
    }
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSclHigh();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSclLow();
    BSP_DelayUs(TRACK_I2C_DELAY_US);

    TrackSdaHigh();

    return value;
}

static uint8_t TrackI2cReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    TrackI2cStart();

    if (TrackI2cWriteByte(TRACK_ADDR_WRITE) != 0U) {
        TrackI2cStop();
        return 0U;
    }
    if (TrackI2cWriteByte(reg) != 0U) {
        TrackI2cStop();
        return 0U;
    }

    TrackI2cStart();
    if (TrackI2cWriteByte(TRACK_ADDR_READ) != 0U) {
        TrackI2cStop();
        return 0U;
    }

    for (uint8_t i = 0U; i < len; i++) {
        data[i] = TrackI2cReadByte((uint8_t)(i + 1U < len));
    }

    TrackI2cStop();
    return 1U;
}

static uint8_t TrackI2cReadReg16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (!TrackI2cReadReg(reg, data, 2U)) {
        return 0U;
    }

    *value = (uint16_t)((uint16_t)data[0] |
                        ((uint16_t)data[1] << 8));
    return 1U;
}

static void TrackSensor_I2cInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    TrackSclHigh();
    TrackSdaHigh();

    GPIO_InitStruct.Pin = TRACK_I2C_SCL_PIN | TRACK_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static uint8_t TrackSensor_ReadThresholds(TrackSensor_t *sensor)
{
    uint8_t threshold_ok = 1U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if (!TrackI2cReadReg16((uint8_t)(TRACK_REG_THRESHOLD + 2U * i),
                               &sensor->threshold[i])) {
            threshold_ok = 0U;
            break;
        }
        if (sensor->threshold[i] == 0U || sensor->threshold[i] == 0xFFFFU) {
            threshold_ok = 0U;
        }
    }

    sensor->threshold_ok = threshold_ok;
    return threshold_ok;
}

/**
 * @brief 初始化传感器并读取一次阈值
 *
 * 幻尔模块上电后需要一点时间；阈值由模块的一键学习功能写入。
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
    sensor->i2c_ok = 0U;
    sensor->last_seen_tick = HAL_GetTick();
    sensor->threshold_ok = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        sensor->analog[i] = 0U;
        sensor->threshold[i] = 0U;
    }

    TrackSensor_I2cInit();
    BSP_DelayUs(100000U);  /* 等模块上电稳定 */

    (void)TrackSensor_ReadThresholds(sensor);

    sensor->inited = 1U;
    s_last_read_tick = HAL_GetTick();
    s_last_threshold_tick = s_last_read_tick;
}

void TrackSensor_Update(TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_read_tick) < TRACK_READ_PERIOD_MS) {
        return;
    }
    s_last_read_tick = now;

    /* 模块刚上电/刚按过学习键时阈值可能还读不到，定期重试 */
    if (!sensor->threshold_ok &&
        (uint32_t)(now - s_last_threshold_tick) >= 1000U) {
        s_last_threshold_tick = now;
        (void)TrackSensor_ReadThresholds(sensor);
    }

    uint8_t digital = 0U;
    uint16_t analog[8];
    uint8_t ok = 1U;

    if (!TrackI2cReadReg(TRACK_REG_DIGITAL, &digital, 1U)) {
        ok = 0U;
    }

    for (uint8_t i = 0U; i < 8U; i++) {
        if (!TrackI2cReadReg16((uint8_t)(TRACK_REG_ANALOG + 2U * i),
                               &analog[i])) {
            ok = 0U;
            break;
        }
    }

    if (!ok) {
        sensor->i2c_ok = 0U;
        sensor->lost = 1U;
        sensor->all_active = 0U;
        sensor->active_mask = 0U;
        sensor->stable_mask = 0U;
        return;
    }

    sensor->i2c_ok = 1U;
    sensor->raw_mask = digital;

    for (uint8_t i = 0U; i < 8U; i++) {
        sensor->analog[i] = analog[i];
    }

    uint8_t mask = 0U;
    uint32_t sum = 0U;
    float weighted = 0.0f;

    if (sensor->threshold_ok) {
        for (uint8_t i = 0U; i < 8U; i++) {
            int32_t dark;
#if TRACK_ANALOG_INVERT
            dark = (int32_t)analog[i] - (int32_t)sensor->threshold[i];
#else
            dark = (int32_t)sensor->threshold[i] - (int32_t)analog[i];
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
        /* 没有读到有效阈值时，用 8 路最大/最小值做粗略归一化 */
        uint16_t max_value = 0U;
        uint16_t min_value = 0xFFFFU;

        for (uint8_t i = 0U; i < 8U; i++) {
            if (analog[i] > max_value) {
                max_value = analog[i];
            }
            if (analog[i] < min_value) {
                min_value = analog[i];
            }
        }

        for (uint8_t i = 0U; i < 8U; i++) {
            int32_t dark;
#if TRACK_ANALOG_INVERT
            dark = (int32_t)analog[i] - (int32_t)min_value;
#else
            dark = (int32_t)max_value - (int32_t)analog[i];
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

uint32_t TrackSensor_GetLastSeenTick(const TrackSensor_t *sensor)
{
    if (sensor == NULL || !sensor->inited) {
        return 0U;
    }
    return sensor->last_seen_tick;
}
