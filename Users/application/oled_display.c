#include "oled_display.h"

#include <stdio.h>
#include <string.h>

#include "lap_timer.h"
#include "ssd1306.h"

#define OLED_LINE_HEIGHT   22U
#define OLED_TEXT_SCALE    2U

static uint32_t s_cached_lap_ms[2];
static uint8_t s_cached_lap_valid[2];
static uint32_t s_cached_obstacle;
static uint8_t s_has_cache;

static void OledDisplay_DrawCentered(uint8_t y, const char *text)
{
    uint8_t len = (uint8_t)strlen(text);
    uint16_t width;

    if (len == 0U) {
        return;
    }

    width = (uint16_t)((uint16_t)len * 6U * OLED_TEXT_SCALE - OLED_TEXT_SCALE);

    uint8_t x = 0U;
    if (width < 128U) {
        x = (uint8_t)((128U - width) / 2U);
    }

    Ssd1306_DrawString(x, y, text, OLED_TEXT_SCALE);
}

static void OledDisplay_Render(void)
{
    char text[24];

    Ssd1306_ClearBuffer();

    for (uint8_t i = 0U; i < 2U; i++) {
        if (LapTimer_HasLap(i)) {
            uint32_t ms = LapTimer_GetLapMs(i);
            snprintf(text, sizeof(text), "L%u %lu.%02lus",
                     (unsigned int)(i + 1U),
                     (unsigned long)(ms / 1000U),
                     (unsigned long)((ms % 1000U) / 10U));
        } else {
            snprintf(text, sizeof(text), "L%u --.--",
                     (unsigned int)(i + 1U));
        }

        OledDisplay_DrawCentered((uint8_t)(i * OLED_LINE_HEIGHT), text);
    }

    snprintf(text, sizeof(text), "OB %lu",
             (unsigned long)LapTimer_GetObstacleCount());
    OledDisplay_DrawCentered((uint8_t)(2U * OLED_LINE_HEIGHT), text);

    Ssd1306_RequestUpdate();
}

void OledDisplay_Init(void)
{
    Ssd1306_Init();

    s_has_cache = 0U;
    OledDisplay_Update();
}

void OledDisplay_Update(void)
{
    uint32_t lap_ms[2];
    uint8_t lap_valid[2];
    uint32_t obstacle;
    uint8_t changed = 0U;

    for (uint8_t i = 0U; i < 2U; i++) {
        lap_ms[i] = LapTimer_GetLapMs(i);
        lap_valid[i] = LapTimer_HasLap(i);
    }
    obstacle = LapTimer_GetObstacleCount();

    if (!s_has_cache) {
        changed = 1U;
    } else {
        for (uint8_t i = 0U; i < 2U; i++) {
            if (lap_ms[i] != s_cached_lap_ms[i] ||
                lap_valid[i] != s_cached_lap_valid[i]) {
                changed = 1U;
            }
        }
        if (obstacle != s_cached_obstacle) {
            changed = 1U;
        }
    }

    if (changed) {
        OledDisplay_Render();

        for (uint8_t i = 0U; i < 2U; i++) {
            s_cached_lap_ms[i] = lap_ms[i];
            s_cached_lap_valid[i] = lap_valid[i];
        }
        s_cached_obstacle = obstacle;
        s_has_cache = 1U;
    }

    /* 每次只发一小块显存，避免长时间占用 5ms 控制循环 */
    Ssd1306_Update();
}
