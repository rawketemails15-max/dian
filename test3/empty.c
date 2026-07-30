/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 */

#include "ti_msp_dl_config.h"

#include "app_config.h"
#include "ball_rod_control.h"

#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_RST_PORT OLED_RST_PORT
#define DISPLAY_RST_PIN  OLED_RST_RST_PIN
#define DISPLAY_DC_PORT  OLED_DC_PORT
#define DISPLAY_DC_PIN   OLED_DC_DC_PIN
#define DISPLAY_SCL_PORT OLED_SCL_PORT
#define DISPLAY_SCL_PIN  OLED_SCL_SCL_PIN
#define DISPLAY_SDA_PORT OLED_SDA_PORT
#define DISPLAY_SDA_PIN  OLED_SDA_SDA_PIN

#define OLED_WIDTH_PIXELS      (128U)
#define OLED_SCALE             (5U)
#define OLED_FONT_HEIGHT       (7U)
#define OLED_FIRST_PAGE        (1U)
#define OLED_RENDER_PAGE_COUNT (5U)
#define OLED_TOP_PIXEL         (12U)
#define OLED_HALF_CLOCK_CYCLES (CPUCLK_FREQ / 4000000U)

static volatile uint32_t gMs;
static volatile uint8_t gControlTicksPending;
static uint32_t gLastOledUpdateMs;
static uint32_t gFaultLedLastToggleMs;

static bool elapsed_ms(uint32_t startMs, uint32_t durationMs)
{
    return (uint32_t) (gMs - startMs) >= durationMs;
}

static void wait_ms(uint32_t durationMs)
{
    while (durationMs-- != 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
    }
}

static void chassis_brake(void)
{
    /*
     * The car must remain stationary for requirement 3.  Stop TIMG6, return
     * both PWM pins to GPIO-low, and clear all TB6612 direction inputs.
     */
    DL_TimerG_stopCounter(PWM_0_INST);

    DL_GPIO_initDigitalOutput(GPIO_PWM_0_C0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_PWM_0_C1_IOMUX);
    DL_GPIO_clearPins(
        GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN | GPIO_PWM_0_C1_PIN);
    DL_GPIO_enableOutput(
        GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN | GPIO_PWM_0_C1_PIN);

    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
}

static void oled_write_byte(uint8_t value, bool data)
{
    uint32_t primask = __get_PRIMASK();

    /*
     * Keep each software-serial OLED byte intact.  The critical section is
     * short enough that it does not materially disturb a 300 Hz STEP train.
     */
    __disable_irq();
    if (data) {
        DL_GPIO_setPins(DISPLAY_DC_PORT, DISPLAY_DC_PIN);
    } else {
        DL_GPIO_clearPins(DISPLAY_DC_PORT, DISPLAY_DC_PIN);
    }

    for (uint8_t bit = 0U; bit < 8U; bit++) {
        DL_GPIO_clearPins(DISPLAY_SCL_PORT, DISPLAY_SCL_PIN);
        DL_Common_delayCycles(OLED_HALF_CLOCK_CYCLES);
        if ((value & 0x80U) != 0U) {
            DL_GPIO_setPins(DISPLAY_SDA_PORT, DISPLAY_SDA_PIN);
        } else {
            DL_GPIO_clearPins(DISPLAY_SDA_PORT, DISPLAY_SDA_PIN);
        }
        DL_GPIO_setPins(DISPLAY_SCL_PORT, DISPLAY_SCL_PIN);
        DL_Common_delayCycles(OLED_HALF_CLOCK_CYCLES);
        value <<= 1U;
    }

    if (primask == 0U) {
        __enable_irq();
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

static void oled_clear(void)
{
    for (uint8_t page = 0U; page < 8U; page++) {
        oled_set_position(0U, page);
        for (uint8_t column = 0U; column < OLED_WIDTH_PIXELS; column++) {
            oled_write_byte(0U, true);
        }
    }
}

static void oled_init(void)
{
    DL_GPIO_clearPins(DISPLAY_RST_PORT, DISPLAY_RST_PIN);
    wait_ms(120U);
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
    oled_clear();
}

static const uint8_t *oled_glyph(char character, uint8_t *width)
{
    static const char characters[] = "0123456789.Ers";
    static const uint8_t glyphs[][3] = {
        {0x7FU, 0x41U, 0x7FU},
        {0x42U, 0x7FU, 0x40U},
        {0x79U, 0x49U, 0x4FU},
        {0x49U, 0x49U, 0x7FU},
        {0x0FU, 0x08U, 0x7FU},
        {0x4FU, 0x49U, 0x79U},
        {0x7FU, 0x49U, 0x79U},
        {0x01U, 0x01U, 0x7FU},
        {0x7FU, 0x49U, 0x7FU},
        {0x4FU, 0x49U, 0x7FU},
        {0x40U, 0x00U, 0x00U},
        {0x7FU, 0x49U, 0x49U},
        {0x7CU, 0x04U, 0x04U},
        {0x4FU, 0x49U, 0x79U},
    };
    static const uint8_t blank[3] = {0U, 0U, 0U};

    for (uint8_t index = 0U;
         index < (uint8_t) (sizeof(characters) - 1U); index++) {
        if (characters[index] == character) {
            *width = (character == '.') ? 1U : 3U;
            return glyphs[index];
        }
    }
    *width = 3U;
    return blank;
}

static uint8_t oled_text_width(const char *text)
{
    uint16_t width = 0U;

    while (*text != '\0') {
        uint8_t glyphWidth;

        (void) oled_glyph(*text++, &glyphWidth);
        width += (uint16_t) glyphWidth * OLED_SCALE;
        if (*text != '\0') {
            width += OLED_SCALE;
        }
    }
    return (uint8_t) width;
}

static void oled_write_text(const char *text)
{
    static uint8_t pixels[OLED_RENDER_PAGE_COUNT][OLED_WIDTH_PIXELS];
    uint8_t textWidth = oled_text_width(text);
    uint8_t x = (uint8_t) ((OLED_WIDTH_PIXELS - textWidth) / 2U);

    for (uint8_t page = 0U; page < OLED_RENDER_PAGE_COUNT; page++) {
        for (uint8_t column = 0U; column < OLED_WIDTH_PIXELS; column++) {
            pixels[page][column] = 0U;
        }
    }

    while (*text != '\0') {
        uint8_t glyphWidth;
        const uint8_t *glyph = oled_glyph(*text++, &glyphWidth);

        for (uint8_t glyphColumn = 0U;
             glyphColumn < glyphWidth; glyphColumn++) {
            for (uint8_t xScale = 0U; xScale < OLED_SCALE; xScale++) {
                uint8_t outputX = (uint8_t) (x + xScale);

                for (uint8_t glyphRow = 0U;
                     glyphRow < OLED_FONT_HEIGHT; glyphRow++) {
                    if ((glyph[glyphColumn] &
                            (uint8_t) (1U << glyphRow)) != 0U) {
                        for (uint8_t yScale = 0U;
                             yScale < OLED_SCALE; yScale++) {
                            uint8_t outputY = (uint8_t)
                                (OLED_TOP_PIXEL +
                                    glyphRow * OLED_SCALE + yScale);
                            uint8_t outputPage =
                                (uint8_t) (outputY / 8U);

                            pixels[outputPage - OLED_FIRST_PAGE][outputX] |=
                                (uint8_t) (1U << (outputY % 8U));
                        }
                    }
                }
            }
            x = (uint8_t) (x + OLED_SCALE);
        }
        if (*text != '\0') {
            x = (uint8_t) (x + OLED_SCALE);
        }
    }

    for (uint8_t page = 0U; page < OLED_RENDER_PAGE_COUNT; page++) {
        oled_set_position(0U, (uint8_t) (OLED_FIRST_PAGE + page));
        for (uint8_t column = 0U; column < OLED_WIDTH_PIXELS; column++) {
            oled_write_byte(pixels[page][column], true);
        }
    }
}

static void format_elapsed_time(uint32_t elapsedMs, char text[6])
{
    uint32_t tenths = (elapsedMs + 50U) / 100U;
    uint32_t seconds;
    uint8_t index = 0U;

    if (tenths > 99U) {
        tenths = 99U;
    }
    seconds = tenths / 10U;
    text[index++] = (char) ('0' + seconds);
    text[index++] = '.';
    text[index++] = (char) ('0' + (tenths % 10U));
    text[index++] = 's';
    text[index] = '\0';
}

static void oled_service(const BallRodTelemetry *telemetry)
{
    char text[6];

    if (!elapsed_ms(gLastOledUpdateMs, APP_OLED_UPDATE_MS)) {
        return;
    }
    gLastOledUpdateMs = gMs;

    if (telemetry->state == BALL_ROD_FAULT) {
        oled_write_text("Err");
        return;
    }

    format_elapsed_time(telemetry->runElapsedMs, text);
    oled_write_text(text);
}

static bool state_is_active(BallRodState state)
{
    return ((state >= BALL_ROD_WAKE) && (state <= BALL_ROD_SETTLE)) ||
        (state == BALL_ROD_TIMEOUT_LEVEL);
}

static void led_service(BallRodState state)
{
    if (state == BALL_ROD_FAULT) {
        if (elapsed_ms(gFaultLedLastToggleMs,
                APP_FAULT_LED_TOGGLE_MS)) {
            gFaultLedLastToggleMs = gMs;
            DL_GPIO_togglePins(LED_PORT, LED_led_PIN);
        }
    } else if (state_is_active(state)) {
        DL_GPIO_setPins(LED_PORT, LED_led_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
    }
}

static bool take_control_tick(void)
{
    bool available;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    available = gControlTicksPending != 0U;
    if (available) {
        gControlTicksPending--;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return available;
}

int main(void)
{
    SYSCFG_DL_init();

    chassis_brake();
    DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
    ball_rod_init(0U);

    /*
     * Finish the software-serial OLED initialization before enabling the
     * STEP and scheduler interrupts.
     */
    oled_init();
    oled_write_text("0.0s");

    NVIC_ClearPendingIRQ(PWM_BALL_STEP_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(PWM_BALL_STEP_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    __disable_irq();
    gControlTicksPending = 0U;
    __enable_irq();

    gLastOledUpdateMs = 0U;
    gFaultLedLastToggleMs = 0U;

    while (1) {
        if (take_control_tick()) {
            BallRodTelemetry telemetry;
            bool buttonPressed =
                ((DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) &
                     KEY_key_PIN) != 0U) ==
                (APP_BUTTON_ACTIVE_LEVEL != 0U);

            ball_rod_tick_5ms(gMs, buttonPressed);
            telemetry = ball_rod_get_telemetry();
            led_service(telemetry.state);
            oled_service(&telemetry);
        } else {
            __WFI();
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) ==
        DL_TIMERG_IIDX_ZERO) {
        gMs += APP_CONTROL_TICK_MS;
        if (gControlTicksPending < 4U) {
            gControlTicksPending++;
        }
    }
}

void PWM_BALL_STEP_INST_IRQHandler(void)
{
    ball_rod_step_isr();
}
