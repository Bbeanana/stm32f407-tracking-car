#include "matrix_display.h"

#include <stdio.h>

#include "main.h"
#include "lap_timer.h"
#include "max7219.h"

#define MATRIX_STEP_MS       50U
#define MATRIX_TEXT_MAX      24U
#define MATRIX_BITMAP_MAX    64U
#define MATRIX_PAGE_COUNT    3U

typedef struct {
    char ch;
    uint8_t col[5];
} MatrixGlyph_t;

/* 5x7 字库，bit0=最上行，每列一个字节 */
static const MatrixGlyph_t s_font[] = {
    {'0', {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}},
    {'1', {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}},
    {'2', {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}},
    {'3', {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}},
    {'4', {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}},
    {'5', {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}},
    {'6', {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}},
    {'7', {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}},
    {'8', {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'9', {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}},
    {'L', {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}},
    {'O', {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}},
    {'B', {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'-', {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}},
    {'.', {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}},
    {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
};

static uint8_t s_bitmap[MATRIX_BITMAP_MAX];
static uint16_t s_bitmap_width;
static int16_t s_scroll_x;
static uint8_t s_page;
static uint8_t s_page_valid;
static uint32_t s_page_value;
static uint32_t s_last_step_tick;
static uint8_t s_frame[8];

static const uint8_t *MatrixDisplay_Glyph(char ch)
{
    static const uint8_t space[5] = {0U, 0U, 0U, 0U, 0U};

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(s_font) / sizeof(s_font[0])); i++) {
        if (s_font[i].ch == ch) {
            return s_font[i].col;
        }
    }

    return space;
}

static uint16_t MatrixDisplay_BuildBitmap(const char *text)
{
    uint16_t width = 0U;

    while (*text != '\0' && width < MATRIX_BITMAP_MAX) {
        const uint8_t *glyph = MatrixDisplay_Glyph(*text);

        for (uint8_t i = 0U; i < 5U && width < MATRIX_BITMAP_MAX; i++) {
            s_bitmap[width++] = glyph[i];
        }
        if (width < MATRIX_BITMAP_MAX) {
            s_bitmap[width++] = 0U;   /* 字符间隔 */
        }
        text++;
    }

    return width;
}

static void MatrixDisplay_BuildPage(void)
{
    char text[MATRIX_TEXT_MAX];

    switch (s_page) {
        case 0U:
            if (LapTimer_HasLap(0U)) {
                uint32_t ms = LapTimer_GetLapMs(0U);
                snprintf(text, sizeof(text), "L1 %lu.%02lu",
                         (unsigned long)(ms / 1000U),
                         (unsigned long)((ms % 1000U) / 10U));
            } else {
                snprintf(text, sizeof(text), "L1 --.--");
            }
            break;

        case 1U:
            if (LapTimer_HasLap(1U)) {
                uint32_t ms = LapTimer_GetLapMs(1U);
                snprintf(text, sizeof(text), "L2 %lu.%02lu",
                         (unsigned long)(ms / 1000U),
                         (unsigned long)((ms % 1000U) / 10U));
            } else {
                snprintf(text, sizeof(text), "L2 --.--");
            }
            break;

        default:
            snprintf(text, sizeof(text), "OB %lu",
                     (unsigned long)LapTimer_GetObstacleCount());
            break;
    }

    s_bitmap_width = MatrixDisplay_BuildBitmap(text);
    s_scroll_x = -8;   /* 从右侧进入 */

    /* 记录当前页的数据，数据变化时立即重新开始滚动 */
    if (s_page == 0U) {
        s_page_valid = LapTimer_HasLap(0U);
        s_page_value = LapTimer_GetLapMs(0U);
    } else if (s_page == 1U) {
        s_page_valid = LapTimer_HasLap(1U);
        s_page_value = LapTimer_GetLapMs(1U);
    } else {
        s_page_valid = 1U;
        s_page_value = LapTimer_GetObstacleCount();
    }
}

static void MatrixDisplay_Render(void)
{
    for (uint8_t x = 0U; x < 8U; x++) {
        int16_t index = (int16_t)(s_scroll_x + (int16_t)x);

        if (index >= 0 && index < (int16_t)s_bitmap_width) {
            s_frame[x] = s_bitmap[index];
        } else {
            s_frame[x] = 0U;
        }
    }

    Max7219_Display(s_frame);
}

void MatrixDisplay_Init(void)
{
    Max7219_Init();

    s_page = 0U;
    s_last_step_tick = HAL_GetTick();
    MatrixDisplay_BuildPage();
    MatrixDisplay_Render();
}

void MatrixDisplay_Update(void)
{
    uint8_t valid;
    uint32_t value;
    uint32_t now = HAL_GetTick();

    if (s_page == 0U) {
        valid = LapTimer_HasLap(0U);
        value = LapTimer_GetLapMs(0U);
    } else if (s_page == 1U) {
        valid = LapTimer_HasLap(1U);
        value = LapTimer_GetLapMs(1U);
    } else {
        valid = 1U;
        value = LapTimer_GetObstacleCount();
    }

    if (valid != s_page_valid || value != s_page_value) {
        MatrixDisplay_BuildPage();
        MatrixDisplay_Render();
        return;
    }

    if ((uint32_t)(now - s_last_step_tick) < MATRIX_STEP_MS) {
        return;
    }
    s_last_step_tick = now;

    s_scroll_x++;

    if (s_scroll_x > (int16_t)s_bitmap_width) {
        s_page = (uint8_t)((s_page + 1U) % MATRIX_PAGE_COUNT);
        MatrixDisplay_BuildPage();
    }

    MatrixDisplay_Render();
}
