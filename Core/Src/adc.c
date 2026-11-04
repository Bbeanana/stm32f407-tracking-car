#include "adc.h"

#include "board_config.h"
#include "bsp_delay.h"

/* 当前 CMSIS 头文件未导出这两个校准位，按 RM0090 定义补齐 */
#ifndef ADC_CR2_CAL
#define ADC_CR2_CAL              (1UL << 2)
#endif
#ifndef ADC_CR2_RSTCAL
#define ADC_CR2_RSTCAL           (1UL << 3)
#endif

/**
 * @brief ADC1 初始化：单通道、软件触发、12 位（裸寄存器实现）
 *
 * 说明：本工程的 HAL 裁剪包没有包含 stm32f4xx_hal_adc.c/h，
 *       而 ADC 只用于读取驱动板电流输出电压，因此直接用寄存器配置，
 *       避免为了一个通道引入整套 HAL ADC 驱动。
 */
void MX_ADC1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC0 模拟输入 */
    GPIO_InitStruct.Pin = CURRENT_ADC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CURRENT_ADC_PORT, &GPIO_InitStruct);

    /* ADC 时钟 = PCLK2 / 4 = 21MHz（上限 36MHz） */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC1->CR1 = 0U;                 /* 12 位、非扫描 */
    ADC1->CR2 = 0U;                 /* 单次转换、软件触发 */
    ADC1->SMPR1 = (ADC1->SMPR1 & ~(7U << 0)) | (4U << 0); /* 通道10，84周期 */
    ADC1->SQR1 = 0U;                /* 1 个转换 */
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = (CURRENT_ADC_CHANNEL & 0x1FU);

    ADC1->CR2 |= ADC_CR2_ADON;
    BSP_DelayUs(10U);               /* 等待 ADC 稳压器稳定 */

    /* 校准（带超时，避免 ADC 异常时卡在启动阶段） */
    uint32_t timeout = 100000U;
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) && (--timeout != 0U)) {
    }

    timeout = 100000U;
    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) && (--timeout != 0U)) {
    }
}

/**
 * @brief 读取一次通道 10（PC0）原始值
 */
uint16_t ADC1_ReadChannel10(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;

    uint32_t timeout = 100000U;
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (--timeout == 0U) {
            return 0U;
        }
    }

    /* 读 DR 自动清 EOC */
    return (uint16_t)(ADC1->DR & 0x0FFFU);
}
