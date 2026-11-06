#include "buzzer.h"

#include "board_config.h"

static uint8_t s_on;
static uint8_t s_timed;
static uint32_t s_deadline_ms;

static GPIO_PinState Buzzer_ActiveState(void)
{
#if BUZZER_ACTIVE_LEVEL
    return GPIO_PIN_SET;
#else
    return GPIO_PIN_RESET;
#endif
}

static GPIO_PinState Buzzer_InactiveState(void)
{
#if BUZZER_ACTIVE_LEVEL
    return GPIO_PIN_RESET;
#else
    return GPIO_PIN_SET;
#endif
}

/**
 * @brief 初始化蜂鸣器 GPIO，默认关闭
 *
 * 先把输出寄存器写成“不响”的电平，再切换为推挽输出，
 * 避免上电初始化瞬间产生一声短鸣。
 */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, Buzzer_InactiveState());

    GPIO_InitStruct.Pin = BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

    s_on = 0U;
    s_timed = 0U;
    s_deadline_ms = 0U;
}

/**
 * @brief 立即打开蜂鸣器（取消之前的定时关闭）
 */
void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, Buzzer_ActiveState());
    s_on = 1U;
    s_timed = 0U;
}

/**
 * @brief 立即关闭蜂鸣器
 */
void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, Buzzer_InactiveState());
    s_on = 0U;
    s_timed = 0U;
}

/**
 * @brief 翻转蜂鸣器状态
 */
void Buzzer_Toggle(void)
{
    if (s_on) {
        Buzzer_Off();
    } else {
        Buzzer_On();
    }
}

/**
 * @brief 非阻塞蜂鸣：到时间后由 Buzzer_Update() 关闭
 */
void Buzzer_Beep(uint32_t duration_ms)
{
    if (duration_ms == 0U) {
        Buzzer_Off();
        return;
    }

    Buzzer_On();
    s_timed = 1U;
    s_deadline_ms = HAL_GetTick() + duration_ms;
}

/**
 * @brief 周期任务：处理定时关闭
 *
 * 用有符号差值比较，HAL_GetTick() 回绕时仍然正确。
 */
void Buzzer_Update(void)
{
    if (s_timed && (int32_t)(HAL_GetTick() - s_deadline_ms) >= 0) {
        Buzzer_Off();
    }
}

uint8_t Buzzer_IsOn(void)
{
    return s_on;
}
