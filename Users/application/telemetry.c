#include "telemetry.h"

#include <stdio.h>

#include "main.h"
#include "board_config.h"
#include "car_config.h"
#include "car_control.h"
#include "motion.h"
#include "current_sense.h"
#include "track_sensor.h"

/**
 * @brief 初始化调试串口输出
 */
void Telemetry_Init(void)
{
#if CAR_TELEMETRY_ENABLE
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\n# tracking_car ready\r\n");
#if CAR_BRINGUP_MODE == 3
    printf("# UART sensor: USART1 115200, PA9=TX PA10=RX\r\n");
    printf("# calibration mode: UART=communication THR=thresholds RAW=mask ACT=analog mask A=8 analog values\r\n");
#else
    printf("# S=state D=cm U=ultra_valid M=mask E=err T=L/R target V=L/R speed C=L/R count A=adc F=fault\r\n");
#endif
#endif
}

#if CAR_BRINGUP_MODE == 3
/**
 * @brief 传感器校准模式：RAW/ACT 掩码 + 8 位可视化条
 *
 * 通道 1 显示在 bar 的最左边。
 * 移动黑线时观察 RAW、ACT 和 A 的变化。
 */
static void Telemetry_PrintCalibration(void)
{
    uint8_t raw = Car_GetTrackRawMask();
    uint8_t act = Car_GetTrackMask();
    char bar[9];

    for (uint8_t i = 0U; i < 8U; i++) {
        bar[i] = (act & (1U << i)) ? '#' : '.';
    }
    bar[8] = '\0';

    printf("CAL UART=%u THR=%u RAW=%02X ACT=%02X |%s| E=%+.2f A=",
           Car_GetTrackCommOk(),
           Car_GetTrackThresholdOk(),
           raw,
           act,
           bar,
           (double)Car_GetTrackError());

    for (uint8_t i = 0U; i < 8U; i++) {
        printf("%u ", Car_GetTrackAnalog(i));
    }

    printf("D=%.1f U=%u\r\n",
           (double)Car_GetDistanceCm(),
           Car_GetUltrasonicValid());
}
#endif

/**
 * @brief 周期打印 CSV 遥测（默认 200ms；校准模式 100ms）
 *
 * 输出示例：
 *   S=LINE D=25.3 U=1 M=18 E=+0.12 T=0.22/0.18 V=0.21/0.18 C=1234/1220 A=0.02 F=0
 */
void Telemetry_Update(void)
{
#if CAR_TELEMETRY_ENABLE
    static uint32_t last_tick;

    uint32_t period = CAR_TELEMETRY_PERIOD_MS;
#if CAR_BRINGUP_MODE == 3
    period = CAR_TELEMETRY_CAL_PERIOD_MS;
#endif

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_tick) < period) {
        return;
    }
    last_tick = now;

#if CAR_BRINGUP_MODE == 3
    Telemetry_PrintCalibration();
#else
    printf("S=%s D=%.1f U=%u M=%02X E=%+.2f T=%.2f/%.2f V=%.2f/%.2f C=%ld/%ld A=%.2f F=%u\r\n",
           Car_GetStateName(),
           (double)Car_GetDistanceCm(),
           Car_GetUltrasonicValid(),
           Car_GetTrackMask(),
           (double)Car_GetTrackError(),
           (double)Motion_GetTargetL(),
           (double)Motion_GetTargetR(),
           (double)Motion_GetSpeedL(),
           (double)Motion_GetSpeedR(),
           (long)Motion_GetCountL(),
           (long)Motion_GetCountR(),
           (double)CurrentSense_ReadVolts(),
           Motion_GetFaultCode());
#endif
#endif
}
