#include "ssd1306.h"

#include <string.h>

#include "board_config.h"
#include "bsp_delay.h"

#define SSD1306_I2C_ADDR      ((uint8_t)(OLED_I2C_ADDR << 1))
#define SSD1306_I2C_DELAY_US  2U
#define SSD1306_CHUNK         16U
#define SSD1306_WIDTH         128U
#define SSD1306_PAGES         8U
#define SSD1306_COL_OFFSET    (OLED_SH1106 ? 2U : 0U)

typedef struct {
    char ch;
    uint8_t col[5];
} Ssd1306Glyph_t;

/* 5x7 字库，bit0=最上行，每列一个字节 */
static const Ssd1306Glyph_t s_font[] = {
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
    {'s', {0x48U, 0x54U, 0x54U, 0x54U, 0x24U}},
    {'-', {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}},
    {'.', {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}},
    {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
};

static uint8_t s_fb[SSD1306_PAGES][SSD1306_WIDTH];

static uint8_t s_dirty;
static uint8_t s_pending;
static uint8_t s_page;
static uint8_t s_col;

static void Ssd1306_SclHigh(void)
{
    HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
}

static void Ssd1306_SclLow(void)
{
    HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET);
}

static void Ssd1306_SdaHigh(void)
{
    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET);
}

static void Ssd1306_SdaLow(void)
{
    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET);
}

static void Ssd1306_I2cStart(void)
{
    Ssd1306_SdaHigh();
    Ssd1306_SclHigh();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);

    Ssd1306_SdaLow();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);

    Ssd1306_SclLow();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);
}

static void Ssd1306_I2cStop(void)
{
    Ssd1306_SdaLow();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);

    Ssd1306_SclHigh();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);

    Ssd1306_SdaHigh();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);
}

static void Ssd1306_I2cWriteByte(uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        if (value & 0x80U) {
            Ssd1306_SdaHigh();
        } else {
            Ssd1306_SdaLow();
        }
        BSP_DelayUs(SSD1306_I2C_DELAY_US);

        Ssd1306_SclHigh();
        BSP_DelayUs(SSD1306_I2C_DELAY_US);

        Ssd1306_SclLow();
        BSP_DelayUs(SSD1306_I2C_DELAY_US);

        value <<= 1;
    }

    /* 第 9 个时钟：释放 SDA，主机不读取 ACK */
    Ssd1306_SdaHigh();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);
    Ssd1306_SclHigh();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);
    Ssd1306_SclLow();
    BSP_DelayUs(SSD1306_I2C_DELAY_US);
}

static void Ssd1306_I2cWrite(const uint8_t *data, uint16_t len)
{
    Ssd1306_I2cStart();
    Ssd1306_I2cWriteByte(SSD1306_I2C_ADDR);

    for (uint16_t i = 0U; i < len; i++) {
        Ssd1306_I2cWriteByte(data[i]);
    }

    Ssd1306_I2cStop();
}

static void Ssd1306_WriteCommand(uint8_t command)
{
    uint8_t buffer[2] = {0x00U, command};
    Ssd1306_I2cWrite(buffer, sizeof(buffer));
}

static void Ssd1306_WriteCommand2(uint8_t command, uint8_t value)
{
    uint8_t buffer[3] = {0x00U, command, value};
    Ssd1306_I2cWrite(buffer, sizeof(buffer));
}

static void Ssd1306_SetCursor(uint8_t page, uint8_t col)
{
    uint8_t col_addr = (uint8_t)(col + SSD1306_COL_OFFSET);
    uint8_t buffer[4] = {
        0x00U,
        (uint8_t)(0xB0U | (page & 0x07U)),
        (uint8_t)(0x00U | (col_addr & 0x0FU)),
        (uint8_t)(0x10U | ((col_addr >> 4U) & 0x0FU)),
    };

    Ssd1306_I2cWrite(buffer, sizeof(buffer));
}

static void Ssd1306_WriteData(const uint8_t *data, uint8_t len)
{
    uint8_t buffer[1U + SSD1306_CHUNK];

    buffer[0] = 0x40U;
    memcpy(&buffer[1], data, len);
    Ssd1306_I2cWrite(buffer, (uint16_t)(len + 1U));
}

static const uint8_t *Ssd1306_Glyph(char ch)
{
    static const uint8_t space[5] = {0U, 0U, 0U, 0U, 0U};

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(s_font) / sizeof(s_font[0])); i++) {
        if (s_font[i].ch == ch) {
            return s_font[i].col;
        }
    }

    return space;
}

static void Ssd1306_SetPixel(uint8_t x, uint8_t y)
{
    if (x >= SSD1306_WIDTH || y >= 64U) {
        return;
    }

    s_fb[y >> 3U][x] |= (uint8_t)(1U << (y & 0x07U));
}

static void Ssd1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t scale)
{
    const uint8_t *glyph = Ssd1306_Glyph(ch);

    for (uint8_t col = 0U; col < 5U; col++) {
        for (uint8_t row = 0U; row < 7U; row++) {
            if (!(glyph[col] & (uint8_t)(1U << row))) {
                continue;
            }

            for (uint8_t dx = 0U; dx < scale; dx++) {
                for (uint8_t dy = 0U; dy < scale; dy++) {
                    uint16_t px = (uint16_t)x + (uint16_t)col * scale + dx;
                    uint16_t py = (uint16_t)y + (uint16_t)row * scale + dy;

                    if (px < SSD1306_WIDTH && py < 64U) {
                        Ssd1306_SetPixel((uint8_t)px, (uint8_t)py);
                    }
                }
            }
        }
    }
}

void Ssd1306_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    Ssd1306_SclHigh();
    Ssd1306_SdaHigh();

    GPIO_InitStruct.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    memset(s_fb, 0, sizeof(s_fb));

#if OLED_FLIP_X
    const uint8_t seg_remap = 0xA0U;
#else
    const uint8_t seg_remap = 0xA1U;
#endif

#if OLED_FLIP_Y
    const uint8_t com_scan = 0xC0U;
#else
    const uint8_t com_scan = 0xC8U;
#endif

    Ssd1306_WriteCommand(0xAEU);          /* display off */
    Ssd1306_WriteCommand2(0xD5U, 0x80U);  /* clock divide */
    Ssd1306_WriteCommand2(0xA8U, 0x3FU);  /* multiplex 64 */
    Ssd1306_WriteCommand2(0xD3U, 0x00U);  /* display offset */
    Ssd1306_WriteCommand(0x40U);          /* start line 0 */
#if OLED_SH1106
    Ssd1306_WriteCommand2(0xADU, 0x8BU);  /* SH1106 DC-DC on */
#else
    Ssd1306_WriteCommand2(0x8DU, 0x14U);  /* SSD1306 charge pump on */
    Ssd1306_WriteCommand2(0x20U, 0x02U);  /* SSD1306 page addressing */
#endif
    Ssd1306_WriteCommand(seg_remap);
    Ssd1306_WriteCommand(com_scan);
    Ssd1306_WriteCommand2(0xDAU, 0x12U);  /* COM pins */
    Ssd1306_WriteCommand2(0x81U, 0xCFU);  /* contrast */
    Ssd1306_WriteCommand2(0xD9U, 0xF1U);  /* pre-charge */
    Ssd1306_WriteCommand2(0xDBU, 0x40U);  /* VCOMH */
    Ssd1306_WriteCommand(0xA4U);          /* resume display */
    Ssd1306_WriteCommand(0xA6U);          /* normal (not inverse) */
    Ssd1306_WriteCommand(0x2EU);          /* deactivate scroll */
    Ssd1306_WriteCommand(0xAFU);          /* display on */

    /* 初始化时整屏清一次，时间很短，可以阻塞 */
    for (uint8_t page = 0U; page < SSD1306_PAGES; page++) {
        Ssd1306_SetCursor(page, 0U);
        for (uint8_t col = 0U; col < SSD1306_WIDTH; col += SSD1306_CHUNK) {
            uint8_t zeros[SSD1306_CHUNK] = {0};
            Ssd1306_WriteData(zeros, SSD1306_CHUNK);
        }
    }

    s_dirty = 0U;
    s_pending = 0U;
    s_page = 0U;
    s_col = 0U;
}

void Ssd1306_ClearBuffer(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void Ssd1306_DrawString(uint8_t x, uint8_t y, const char *text, uint8_t scale)
{
    if (scale == 0U) {
        scale = 1U;
    }

    while (*text != '\0') {
        Ssd1306_DrawChar(x, y, *text, scale);
        x = (uint8_t)(x + 6U * scale);
        text++;
    }
}

void Ssd1306_RequestUpdate(void)
{
    if (s_dirty) {
        s_pending = 1U;
        return;
    }

    s_dirty = 1U;
    s_page = 0U;
    s_col = 0U;
}

void Ssd1306_Update(void)
{
    if (!s_dirty) {
        return;
    }

    if (s_page >= SSD1306_PAGES) {
        s_dirty = 0U;
        if (s_pending) {
            s_pending = 0U;
            s_dirty = 1U;
            s_page = 0U;
            s_col = 0U;
        }
        return;
    }

    uint8_t len = (uint8_t)(SSD1306_WIDTH - s_col);
    if (len > SSD1306_CHUNK) {
        len = SSD1306_CHUNK;
    }

    Ssd1306_SetCursor(s_page, s_col);
    Ssd1306_WriteData(&s_fb[s_page][s_col], len);

    s_col = (uint8_t)(s_col + len);
    if (s_col >= SSD1306_WIDTH) {
        s_col = 0U;
        s_page++;
    }

    if (s_page >= SSD1306_PAGES) {
        s_dirty = 0U;
        if (s_pending) {
            s_pending = 0U;
            s_dirty = 1U;
            s_page = 0U;
            s_col = 0U;
        }
    }
}
