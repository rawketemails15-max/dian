/*
 * K230 touch-screen UART feasibility test.
 *
 * K230 sends one ASCII byte ('A', 'B', or 'C') over UART1.  The receive
 * interrupt records the newest valid byte and the main loop renders it on
 * the C07A/S28A OLED.  OLED I/O follows the existing four-wire GPIO driver
 * used by this repository's 24H project.
 */

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define OLED_WIDTH             (128U)
#define OLED_PAGE_COUNT        (8U)
#define OLED_GLYPH_WIDTH       (5U)
#define OLED_GLYPH_HEIGHT      (7U)
#define OLED_GLYPH_SCALE       (8U)
#define OLED_GLYPH_X           ((OLED_WIDTH - OLED_GLYPH_WIDTH * OLED_GLYPH_SCALE) / 2U)
#define OLED_GLYPH_Y           (4U)
#define OLED_RESET_DELAY_CYCLES ((CPUCLK_FREQ / 1000U) * 120U)

#define DISPLAY_RST_PORT OLED_RST_PORT
#define DISPLAY_RST_PIN  OLED_RST_RST_PIN
#define DISPLAY_DC_PORT  OLED_DC_PORT
#define DISPLAY_DC_PIN   OLED_DC_DC_PIN
#define DISPLAY_SCL_PORT OLED_SCL_PORT
#define DISPLAY_SCL_PIN  OLED_SCL_SCL_PIN
#define DISPLAY_SDA_PORT OLED_SDA_PORT
#define DISPLAY_SDA_PIN  OLED_SDA_SDA_PIN

static volatile uint8_t gPendingLetter;
static volatile bool gLetterPending;
static uint8_t gOledFrame[OLED_PAGE_COUNT][OLED_WIDTH];

static void oled_write_byte(uint8_t value, bool data)
{
    if (data) {
        DL_GPIO_setPins(DISPLAY_DC_PORT, DISPLAY_DC_PIN);
    } else {
        DL_GPIO_clearPins(DISPLAY_DC_PORT, DISPLAY_DC_PIN);
    }

    for (uint8_t bit = 0U; bit < 8U; bit++) {
        DL_GPIO_clearPins(DISPLAY_SCL_PORT, DISPLAY_SCL_PIN);
        if ((value & 0x80U) != 0U) {
            DL_GPIO_setPins(DISPLAY_SDA_PORT, DISPLAY_SDA_PIN);
        } else {
            DL_GPIO_clearPins(DISPLAY_SDA_PORT, DISPLAY_SDA_PIN);
        }
        DL_GPIO_setPins(DISPLAY_SCL_PORT, DISPLAY_SCL_PIN);
        value <<= 1U;
    }
}

static void oled_command(uint8_t command)
{
    oled_write_byte(command, false);
}

static void oled_set_position(uint8_t column, uint8_t page)
{
    oled_command((uint8_t) (0xB0U | (page & 0x07U)));
    oled_command((uint8_t) (column & 0x0FU));
    oled_command((uint8_t) (0x10U | ((column >> 4U) & 0x0FU)));
}

static void oled_flush_frame(void)
{
    for (uint8_t page = 0U; page < OLED_PAGE_COUNT; page++) {
        oled_set_position(0U, page);
        for (uint8_t column = 0U; column < OLED_WIDTH; column++) {
            oled_write_byte(gOledFrame[page][column], true);
        }
    }
}

static void oled_clear_frame(void)
{
    for (uint8_t page = 0U; page < OLED_PAGE_COUNT; page++) {
        for (uint8_t column = 0U; column < OLED_WIDTH; column++) {
            gOledFrame[page][column] = 0U;
        }
    }
}

static const uint8_t *oled_glyph(char letter)
{
    static const uint8_t glyphA[OLED_GLYPH_WIDTH] =
        {0x7EU, 0x09U, 0x09U, 0x09U, 0x7EU};
    static const uint8_t glyphB[OLED_GLYPH_WIDTH] =
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U};
    static const uint8_t glyphC[OLED_GLYPH_WIDTH] =
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
    static const uint8_t glyphDash[OLED_GLYPH_WIDTH] =
        {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};

    switch (letter) {
        case 'A':
            return glyphA;
        case 'B':
            return glyphB;
        case 'C':
            return glyphC;
        default:
            return glyphDash;
    }
}

static void oled_show_letter(char letter)
{
    const uint8_t *glyph = oled_glyph(letter);

    oled_clear_frame();
    for (uint8_t glyphColumn = 0U;
         glyphColumn < OLED_GLYPH_WIDTH; glyphColumn++) {
        for (uint8_t glyphRow = 0U;
             glyphRow < OLED_GLYPH_HEIGHT; glyphRow++) {
            if ((glyph[glyphColumn] & (uint8_t) (1U << glyphRow)) == 0U) {
                continue;
            }

            for (uint8_t xScale = 0U; xScale < OLED_GLYPH_SCALE; xScale++) {
                for (uint8_t yScale = 0U; yScale < OLED_GLYPH_SCALE; yScale++) {
                    uint8_t x = (uint8_t) (OLED_GLYPH_X +
                        glyphColumn * OLED_GLYPH_SCALE + xScale);
                    uint8_t y = (uint8_t) (OLED_GLYPH_Y +
                        glyphRow * OLED_GLYPH_SCALE + yScale);

                    gOledFrame[y / 8U][x] |=
                        (uint8_t) (1U << (y % 8U));
                }
            }
        }
    }
    oled_flush_frame();
}

static void oled_init(void)
{
    DL_GPIO_clearPins(DISPLAY_RST_PORT, DISPLAY_RST_PIN);
    delay_cycles(OLED_RESET_DELAY_CYCLES);
    DL_GPIO_setPins(DISPLAY_RST_PORT, DISPLAY_RST_PIN);

    oled_command(0xAEU);
    oled_command(0xD5U);
    oled_command(0x50U);
    oled_command(0xA8U);
    oled_command(0x3FU);
    oled_command(0xD3U);
    oled_command(0x00U);
    oled_command(0x40U);
    oled_command(0x8DU);
    oled_command(0x14U);
    oled_command(0x20U);
    oled_command(0x02U);
    oled_command(0xA0U);
    oled_command(0xC0U);
    oled_command(0xDAU);
    oled_command(0x12U);
    oled_command(0x81U);
    oled_command(0xEFU);
    oled_command(0xD9U);
    oled_command(0xF1U);
    oled_command(0xDBU);
    oled_command(0x30U);
    oled_command(0xA4U);
    oled_command(0xA6U);
    oled_command(0xAFU);
    oled_show_letter('-');
}

int main(void)
{
    SYSCFG_DL_init();
    oled_init();

    while (!DL_UART_Main_isRXFIFOEmpty(UART_K230_INST)) {
        (void) DL_UART_Main_receiveData(UART_K230_INST);
    }
    DL_UART_Main_clearInterruptStatus(
        UART_K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);

    while (1) {
        bool updateDisplay = false;
        uint8_t letter = 0U;

        __disable_irq();
        if (gLetterPending) {
            letter = gPendingLetter;
            gLetterPending = false;
            updateDisplay = true;
        }
        __enable_irq();

        if (updateDisplay) {
            oled_show_letter((char) letter);
        } else {
            __NOP();
        }
    }
}

void UART_K230_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_K230_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_K230_INST)) {
                uint8_t received =
                    DL_UART_Main_receiveData(UART_K230_INST);

                if ((received == (uint8_t) 'A') ||
                    (received == (uint8_t) 'B') ||
                    (received == (uint8_t) 'C')) {
                    gPendingLetter = received;
                    gLetterPending = true;
                }
            }
            break;
        default:
            break;
    }
}
