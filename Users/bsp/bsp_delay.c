#include "bsp_delay.h"

#include "main.h"

/**
 * @brief 使能 DWT 周期计数器，用于微秒级延时
 */
void BSP_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 忙等待微秒延时（基于 DWT，168MHz 下约 1µs 分辨率）
 */
void BSP_DelayUs(uint32_t us)
{
    if (us == 0U) {
        return;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}
