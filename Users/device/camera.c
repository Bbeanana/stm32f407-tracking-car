#include "camera.h"

#include <string.h>

#include "board_config.h"
#include "car_config.h"
#include "main.h"

#define CAMERA_FRAME_LEN     14U
#define CAMERA_RX_SIZE       64U
#define CAMERA_RX_MASK       (CAMERA_RX_SIZE - 1U)

static UART_HandleTypeDef s_camera_uart;
static uint8_t s_uart_ready;

static volatile uint8_t s_rx_buf[CAMERA_RX_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;

static uint8_t s_packet[CAMERA_FRAME_LEN];
static uint8_t s_packet_index;
static uint8_t s_in_frame;

static float s_error;
static float s_angle_deg;
static float s_curvature_deg;
static uint8_t s_valid;
static uint32_t s_last_packet_tick;

/* 调试计数器：可通过 ST-Link 直接读取 */
volatile uint32_t g_camera_rx_bytes;
volatile uint32_t g_camera_valid_frames;
volatile uint32_t g_camera_bad_frames;
volatile uint8_t g_camera_last_packet[14];

static float Camera_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

void Camera_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = CAMERA_UART_TX_PIN | CAMERA_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    s_camera_uart.Instance = CAMERA_UART_INSTANCE;
    s_camera_uart.Init.BaudRate = CAMERA_UART_BAUD;
    s_camera_uart.Init.WordLength = UART_WORDLENGTH_8B;
    s_camera_uart.Init.StopBits = UART_STOPBITS_1;
    s_camera_uart.Init.Parity = UART_PARITY_NONE;
    s_camera_uart.Init.Mode = UART_MODE_TX_RX;
    s_camera_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_camera_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    s_uart_ready = 0U;
    if (HAL_UART_Init(&s_camera_uart) == HAL_OK) {
        s_uart_ready = 1U;
    }

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_packet_index = 0U;
    s_in_frame = 0U;
    s_error = 0.0f;
    s_angle_deg = 0.0f;
    s_curvature_deg = 0.0f;
    s_valid = 0U;
    g_camera_rx_bytes = 0U;
    g_camera_valid_frames = 0U;
    g_camera_bad_frames = 0U;
    s_last_packet_tick = HAL_GetTick();

    if (!s_uart_ready) {
        return;
    }

    __HAL_UART_ENABLE_IT(&s_camera_uart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Camera_UartIrqHandler(void)
{
    if ((USART1->SR & USART_SR_RXNE) == 0U) {
        return;
    }

    uint8_t data = (uint8_t)USART1->DR;
    g_camera_rx_bytes++;
    uint8_t next = (uint8_t)((s_rx_head + 1U) & CAMERA_RX_MASK);

    if (next != s_rx_tail) {
        s_rx_buf[s_rx_head] = data;
        s_rx_head = next;
    }
}

static void Camera_ParseFrame(uint8_t length)
{
    float err;
    float ang;
    float curv = 0.0f;

    if (s_packet[length - 1U] != 0x5AU) {
        g_camera_bad_frames++;
        return;
    }

    memcpy(&err, &s_packet[1], sizeof(float));
    memcpy(&ang, &s_packet[5], sizeof(float));

    if (length >= 14U) {
        memcpy(&curv, &s_packet[9], sizeof(float));
    }

    if (err != err || ang != ang || curv != curv) {   /* NaN */
        return;
    }

    s_error = Camera_Clamp(err / 1000.0f, -1.0f, 1.0f);
    s_angle_deg = ang;
    s_curvature_deg = curv;
    s_last_packet_tick = HAL_GetTick();
    s_valid = 1U;
    g_camera_valid_frames++;
}

void Camera_Update(void)
{
    if (!s_uart_ready) {
        return;
    }

    while (s_rx_tail != s_rx_head) {
        uint8_t data = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint8_t)((s_rx_tail + 1U) & CAMERA_RX_MASK);

        if (!s_in_frame) {
            if (data == 0xA5U) {
                s_packet[0] = data;
                s_packet_index = 1U;
                s_in_frame = 1U;
            }
            continue;
        }

        if (s_packet_index < CAMERA_FRAME_LEN) {
            s_packet[s_packet_index++] = data;
        }

        /* 兼容旧 10 字节帧：A5 + ERR + ANG + 5A。
         * 10 字节帧后面紧跟下一帧的 A5 时，可以无歧义识别。 */
        if (s_packet_index == 11U &&
            s_packet[9] == 0x5AU && s_packet[10] == 0xA5U) {
            for (uint8_t i = 0U; i < CAMERA_FRAME_LEN; i++) {
                g_camera_last_packet[i] = (i < 10U) ? s_packet[i] : 0U;
            }

            Camera_ParseFrame(10U);

            /* 当前字节是下一帧的帧头，直接开始下一帧 */
            s_packet[0] = 0xA5U;
            s_packet_index = 1U;
            s_in_frame = 1U;
            continue;
        }

        if (s_packet_index >= CAMERA_FRAME_LEN) {
            for (uint8_t i = 0U; i < CAMERA_FRAME_LEN; i++) {
                g_camera_last_packet[i] = s_packet[i];
            }

            Camera_ParseFrame(CAMERA_FRAME_LEN);

            s_in_frame = 0U;
            s_packet_index = 0U;

            /* 如果本帧最后一字节也是帧头，立即重新同步 */
            if (data == 0xA5U) {
                s_packet[0] = data;
                s_packet_index = 1U;
                s_in_frame = 1U;
            }
        }
    }
}

uint8_t Camera_IsValid(void)
{
    if (!s_uart_ready || !s_valid) {
        return 0U;
    }

    if ((uint32_t)(HAL_GetTick() - s_last_packet_tick) >
        CAR_CAMERA_TIMEOUT_MS) {
        s_valid = 0U;
        return 0U;
    }

    return 1U;
}

float Camera_GetError(void)
{
    return s_error;
}

float Camera_GetAngleDeg(void)
{
    return s_angle_deg;
}

float Camera_GetCurvatureDeg(void)
{
    return s_curvature_deg;
}

uint32_t Camera_GetLastPacketTick(void)
{
    return s_last_packet_tick;
}
