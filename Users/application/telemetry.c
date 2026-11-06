#include "telemetry.h"

#include <stdio.h>

#include "main.h"
#include "car_config.h"
#include "car_control.h"
#include "lap_timer.h"
#include "motion.h"
#include "current_sense.h"

/**
 * @brief 初始化调试串口输出
 */
void Telemetry_Init(void)
{
#if CAR_TELEMETRY_ENABLE
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\n# tracking_car ready\r\n");
#if CAR_BRINGUP_MODE == 3
    printf("# calibration mode: RAW=raw pins ACT=active mask |bar| E=error D=cm U=valid\r\n");
#else
    printf("# S=state D=cm U=ultra_valid M=mask E=err T=L/R target V=L/R speed C=L/R count A=adc F=fault\r\n");
#endif
#endif
}

#if CAR_BRINGUP_MODE == 3
/**
 * @brief 传感器校准模式：RAW/ACT 掩码 + 8 位可视化条
 *
 * 通道 0（PD2，最左）显示在 bar 的最左边。
 * 调整模块电位器时观察 bar 和 ACT 的变化即可。
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

    printf("CAL RAW=%02X ACT=%02X |%s| E=%+.2f D=%.1f U=%u\r\n",
           raw,
           act,
           bar,
           (double)Car_GetTrackError(),
           (double)Car_GetDistanceCm(),
           Car_GetUltrasonicValid());
}
#endif

/**
 * @brief 周期打印 CSV 遥测（默认 200ms；校准模式 100ms）
 *
 * 输出示例：
 *   S=LINE D=25.3 U=1 M=18 E=+0.12 T=0.22/0.18 V=0.21/0.18 C=1234/1220 A=0.02 F=0 L1=12.34 L2=--.-- LAP=3.45 N=2
 */
static void Telemetry_FormatLap(char *out, uint8_t out_size, uint8_t index)
{
    if (LapTimer_HasLap(index)) {
        uint32_t ms = LapTimer_GetLapMs(index);
        snprintf(out, out_size, "%lu.%02lu",
                 (unsigned long)(ms / 1000U),
                 (unsigned long)((ms % 1000U) / 10U));
    } else {
        snprintf(out, out_size, "--.--");
    }
}

static void Telemetry_FormatCurrentLap(char *out, uint8_t out_size)
{
    if (LapTimer_IsRunning()) {
        uint32_t ms = LapTimer_GetCurrentMs();
        snprintf(out, out_size, "%lu.%02lu",
                 (unsigned long)(ms / 1000U),
                 (unsigned long)((ms % 1000U) / 10U));
    } else {
        snprintf(out, out_size, "--.--");
    }
}

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
    char lap1[12];
    char lap2[12];
    char current_lap[12];

    Telemetry_FormatLap(lap1, sizeof(lap1), 0U);
    Telemetry_FormatLap(lap2, sizeof(lap2), 1U);
    Telemetry_FormatCurrentLap(current_lap, sizeof(current_lap));

    printf("S=%s D=%.1f U=%u M=%02X E=%+.2f T=%.2f/%.2f V=%.2f/%.2f C=%ld/%ld A=%.2f F=%u L1=%s L2=%s LAP=%s N=%u\r\n",
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
           Motion_GetFaultCode(),
           lap1,
           lap2,
           current_lap,
           (unsigned)LapTimer_GetCurrentLap());
#endif
#endif
}
