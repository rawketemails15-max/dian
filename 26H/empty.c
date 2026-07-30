/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include "app_config.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

#define LINE_ADDR0_PORT LINE_SENSOR_PORT
#define LINE_ADDR0_PIN  LINE_SENSOR_AD0_PIN
#define LINE_ADDR1_PORT LINE_SENSOR_PORT
#define LINE_ADDR1_PIN  LINE_SENSOR_AD1_PIN
#define LINE_ADDR2_PORT LINE_SENSOR_PORT
#define LINE_ADDR2_PIN  LINE_SENSOR_AD2_PIN
#define LINE_OUT_PORT   LINE_SENSOR_PORT
#define LINE_OUT_PIN    LINE_SENSOR_OUT_PIN

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

#define LINE_SETTLE_CYCLES \
    ((CPUCLK_FREQ / 1000000U) * APP_LINE_MUX_SETTLE_US)

typedef enum {
    DRIVE_IDLE = 0,
    DRIVE_LEAVING_START,
    DRIVE_TRACKING,
    DRIVE_FINAL_APPROACH,
    DRIVE_COMPLETE,
    DRIVE_FAULT
} DriveState;

static volatile uint32_t gMs;
static volatile uint8_t gControlTicksPending;

static DriveState gDriveState = DRIVE_IDLE;
static uint32_t gRunStartMs;
static uint32_t gFrozenElapsedMs;
static uint32_t gLastOledUpdateMs;
static uint32_t gStartMarkerClearMs;
static uint32_t gLineLostStartMs;
static uint32_t gFaultLedLastToggleMs;
static int32_t gStartEncoderA;
static int32_t gStartEncoderB;
static int16_t gLastLineError;
static int32_t gFilteredLineError;
static int32_t gLinePidIntegral;
static int32_t gLinePidPreviousError;
static int32_t gLinePidDerivative;
static int32_t gLinePidCorrection;
static uint16_t gCommandPwmA;
static uint16_t gCommandPwmB;
static uint8_t gFinishMarkerScans;
static bool gStartMarkerClearActive;
static bool gLineLostActive;
static bool gFinishArmed;
static bool gDisplayDirty = true;

static uint8_t gButtonRaw;
static uint8_t gButtonStable;
static uint32_t gButtonRawChangedMs;

static bool elapsed_ms(uint32_t startMs, uint32_t durationMs)
{
    return (uint32_t) (gMs - startMs) >= durationMs;
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t slew_i32(int32_t current, int32_t target, int32_t maximumStep)
{
    if (target > (current + maximumStep)) {
        return current + maximumStep;
    }
    if (target < (current - maximumStep)) {
        return current - maximumStep;
    }
    return target;
}

static int32_t line_pid_update(int32_t error)
{
    int32_t errorMagnitude = (error < 0) ? -error : error;
    int32_t derivativeInput = error - gLinePidPreviousError;
    int32_t proportionalTerm;
    int32_t integralTerm;
    int32_t derivativeTerm;

    /*
     * Integrate only close to the line.  In a large bend the proportional
     * term must dominate; accumulating there would cause an opposite kick
     * when the car returns to the center.
     */
    if ((error != 0) &&
        (errorMagnitude <= APP_LINE_PID_INTEGRAL_ACTIVE_ERROR)) {
        gLinePidIntegral = clamp_i32(gLinePidIntegral + error,
            -APP_LINE_PID_INTEGRAL_LIMIT,
            APP_LINE_PID_INTEGRAL_LIMIT);
    } else {
        gLinePidIntegral = (gLinePidIntegral * 7) / 8;
    }

    /*
     * Low-pass the discrete derivative because the digital grayscale
     * position changes in steps as adjacent probes switch.
     */
    gLinePidDerivative +=
        (derivativeInput - gLinePidDerivative) /
        APP_LINE_PID_D_FILTER_DIVISOR;
    if ((derivativeInput == 0) &&
        (gLinePidDerivative > -APP_LINE_PID_D_FILTER_DIVISOR) &&
        (gLinePidDerivative < APP_LINE_PID_D_FILTER_DIVISOR)) {
        gLinePidDerivative = 0;
    }
    gLinePidPreviousError = error;

    proportionalTerm = error * APP_LINE_PID_KP;
    integralTerm = (gLinePidIntegral *
        APP_LINE_PID_KI_NUMERATOR) / APP_LINE_PID_KI_DIVISOR;
    derivativeTerm = gLinePidDerivative * APP_LINE_PID_KD;

    return clamp_i32(proportionalTerm + integralTerm + derivativeTerm,
        -APP_LINE_PID_OUTPUT_LIMIT, APP_LINE_PID_OUTPUT_LIMIT);
}

static uint32_t abs_delta_i32(int32_t value, int32_t origin)
{
    int64_t delta = (int64_t) value - (int64_t) origin;

    if (delta < 0) {
        delta = -delta;
    }
    return (uint32_t) delta;
}

static void wait_ms(uint32_t durationMs)
{
    uint32_t startMs = gMs;

    while (!elapsed_ms(startMs, durationMs)) {
        __WFI();
    }
}

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

    if (tenths > 200U) {
        tenths = 200U;
    }
    seconds = tenths / 10U;
    if (seconds >= 10U) {
        text[index++] = (char) ('0' + (seconds / 10U));
    }
    text[index++] = (char) ('0' + (seconds % 10U));
    text[index++] = '.';
    text[index++] = (char) ('0' + (tenths % 10U));
    text[index++] = 's';
    text[index] = '\0';
}

static void oled_service(void)
{
    char text[6];
    uint32_t shownMs;

    if (!gDisplayDirty &&
        !elapsed_ms(gLastOledUpdateMs, APP_OLED_UPDATE_MS)) {
        return;
    }
    gLastOledUpdateMs = gMs;
    gDisplayDirty = false;

    if (gDriveState == DRIVE_FAULT) {
        oled_write_text("Err");
        return;
    }

    if (gDriveState == DRIVE_COMPLETE) {
        shownMs = gFrozenElapsedMs;
    } else if ((gDriveState == DRIVE_LEAVING_START) ||
               (gDriveState == DRIVE_TRACKING) ||
               (gDriveState == DRIVE_FINAL_APPROACH)) {
        shownMs = (uint32_t) (gMs - gRunStartMs);
    } else {
        shownMs = 0U;
    }
    format_elapsed_time(shownMs, text);
    oled_write_text(text);
}

static void line_sensor_select(uint8_t channel)
{
    if ((channel & 0x01U) != 0U) {
        DL_GPIO_setPins(LINE_ADDR0_PORT, LINE_ADDR0_PIN);
    } else {
        DL_GPIO_clearPins(LINE_ADDR0_PORT, LINE_ADDR0_PIN);
    }
    if ((channel & 0x02U) != 0U) {
        DL_GPIO_setPins(LINE_ADDR1_PORT, LINE_ADDR1_PIN);
    } else {
        DL_GPIO_clearPins(LINE_ADDR1_PORT, LINE_ADDR1_PIN);
    }
    if ((channel & 0x04U) != 0U) {
        DL_GPIO_setPins(LINE_ADDR2_PORT, LINE_ADDR2_PIN);
    } else {
        DL_GPIO_clearPins(LINE_ADDR2_PORT, LINE_ADDR2_PIN);
    }
}

static void line_sensor_init(void)
{
    line_sensor_select(0U);
    DL_Common_delayCycles(LINE_SETTLE_CYCLES);
}

static uint8_t line_sensor_read_mask(void)
{
    uint8_t mask = 0U;

    for (uint8_t channel = 0U;
         channel < APP_LINE_CHANNEL_COUNT; channel++) {
        uint8_t level;

        line_sensor_select(channel);
        DL_Common_delayCycles(LINE_SETTLE_CYCLES);
        level = ((DL_GPIO_readPins(LINE_OUT_PORT, LINE_OUT_PIN) &
                     LINE_OUT_PIN) != 0U) ? 1U : 0U;
        if (level == APP_LINE_BLACK_LEVEL) {
            mask |= (uint8_t) (1U << channel);
        }
    }
    return mask;
}

static int16_t line_error_from_mask(uint8_t mask)
{
    static const int16_t weights[8] =
        {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum = 0;
    uint8_t count = 0U;

    if (mask == 0xFFU) {
        gLastLineError = 0;
        return 0;
    }

    for (uint8_t channel = 0U; channel < 8U; channel++) {
        if ((mask & (uint8_t) (1U << channel)) != 0U) {
            uint8_t index = (APP_LINE_REVERSE_ORDER != 0U) ?
                (uint8_t) (7U - channel) : channel;
            sum += weights[index];
            count++;
        }
    }

    if (count != 0U) {
        gLastLineError = (int16_t) (sum / count);
    }
    return gLastLineError;
}

static uint8_t maximum_contiguous_black(uint8_t mask)
{
    uint8_t longest = 0U;
    uint8_t current = 0U;

    for (uint8_t index = 0U; index < 8U; index++) {
        if ((mask & (uint8_t) (1U << index)) != 0U) {
            current++;
            if (current > longest) {
                longest = current;
            }
        } else {
            current = 0U;
        }
    }
    return longest;
}

static bool is_wide_marker(uint8_t mask)
{
    return maximum_contiguous_black(mask) >=
        APP_MARKER_MIN_CONTIGUOUS_CHANNELS;
}

static uint32_t traveled_counts(void)
{
    MotorTelemetry telemetry = motor_control_get_telemetry();
    uint32_t distanceA =
        abs_delta_i32(telemetry.encoderCountA, gStartEncoderA);
    uint32_t distanceB =
        abs_delta_i32(telemetry.encoderCountB, gStartEncoderB);

    return (distanceA + distanceB) / 2U;
}

static bool button_pressed_event(void)
{
    uint8_t raw = ((DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) &
                       KEY_key_PIN) != 0U) ? 1U : 0U;

    if (raw != gButtonRaw) {
        gButtonRaw = raw;
        gButtonRawChangedMs = gMs;
    }
    if ((gButtonStable != gButtonRaw) &&
        elapsed_ms(gButtonRawChangedMs, APP_BUTTON_DEBOUNCE_MS)) {
        gButtonStable = gButtonRaw;
        return gButtonStable == APP_BUTTON_ACTIVE_LEVEL;
    }
    return false;
}

static void enter_fault(void)
{
    motor_control_brake();
    gDriveState = DRIVE_FAULT;
    gFaultLedLastToggleMs = gMs;
    DL_GPIO_setPins(LED_PORT, LED_led_PIN);
    gDisplayDirty = true;
}

static void enter_complete(void)
{
    gFrozenElapsedMs = (uint32_t) (gMs - gRunStartMs);
    motor_control_brake();
    gDriveState = DRIVE_COMPLETE;
    DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
    gDisplayDirty = true;
}

static void start_run(void)
{
    MotorTelemetry telemetry = motor_control_get_telemetry();

    gStartEncoderA = telemetry.encoderCountA;
    gStartEncoderB = telemetry.encoderCountB;
    gRunStartMs = gMs;
    gFrozenElapsedMs = 0U;
    gStartMarkerClearMs = gMs;
    gLineLostStartMs = gMs;
    gLastLineError = 0;
    gFilteredLineError = 0;
    gLinePidIntegral = 0;
    gLinePidPreviousError = 0;
    gLinePidDerivative = 0;
    gLinePidCorrection = 0;
    gCommandPwmA = 0U;
    gCommandPwmB = 0U;
    gFinishMarkerScans = 0U;
    gStartMarkerClearActive = false;
    gLineLostActive = false;
    gFinishArmed = false;
    gDriveState = DRIVE_LEAVING_START;
    DL_GPIO_setPins(LED_PORT, LED_led_PIN);
    motor_control_start(gMs);
    gDisplayDirty = true;
}

static void update_finish_logic(uint8_t lineMask, uint32_t distance,
    uint32_t elapsed)
{
    bool wideMarker = is_wide_marker(lineMask);

    if (gDriveState == DRIVE_LEAVING_START) {
        if (!wideMarker) {
            if (!gStartMarkerClearActive) {
                gStartMarkerClearActive = true;
                gStartMarkerClearMs = gMs;
            }
        } else {
            gStartMarkerClearActive = false;
        }

        if (gStartMarkerClearActive &&
            elapsed_ms(gStartMarkerClearMs, APP_START_MARKER_CLEAR_MS) &&
            (distance >= APP_START_MARKER_CLEAR_COUNTS)) {
            gDriveState = DRIVE_TRACKING;
        }
        return;
    }

    if (!gFinishArmed &&
        (distance >= APP_FINISH_ARM_COUNTS) &&
        (elapsed >= APP_FINISH_ARM_MS)) {
        gFinishArmed = true;
    }
    if ((gDriveState == DRIVE_TRACKING) &&
        (distance >= APP_FINAL_APPROACH_COUNTS)) {
        gDriveState = DRIVE_FINAL_APPROACH;
    }

    if (gFinishArmed && wideMarker) {
        if (gFinishMarkerScans < APP_MARKER_CONFIRM_SCANS) {
            gFinishMarkerScans++;
        }
        if (gFinishMarkerScans >= APP_MARKER_CONFIRM_SCANS) {
            enter_complete();
        }
    } else {
        gFinishMarkerScans = 0U;
    }
}

static void update_line_control(uint8_t lineMask)
{
    int32_t basePwm;
    int32_t correction;
    int32_t targetPwmA;
    int32_t targetPwmB;
    int32_t controlError;
    int16_t lineError = line_error_from_mask(lineMask);

    if (lineMask == 0U) {
        if (!gLineLostActive) {
            gLineLostActive = true;
            gLineLostStartMs = gMs;
        } else if (elapsed_ms(gLineLostStartMs,
                       APP_LINE_LOST_TIMEOUT_MS)) {
            enter_fault();
            return;
        }
    } else {
        gLineLostActive = false;
        gFilteredLineError +=
            ((int32_t) lineError - gFilteredLineError) /
            APP_TRACK_ERROR_FILTER_DIVISOR;

        controlError = gFilteredLineError;
        if ((controlError >= -APP_TRACK_CENTER_DEADBAND) &&
            (controlError <= APP_TRACK_CENTER_DEADBAND)) {
            controlError = 0;
        }
        gLinePidCorrection = line_pid_update(controlError);
    }

    /*
     * Motor A is the right wheel and Motor B is the left wheel.  During a
     * brief line loss, freeze the last PID correction so the car continues
     * searching in the established direction without integral windup.
     */
    basePwm = (gDriveState == DRIVE_FINAL_APPROACH) ?
        APP_TRACK_FINAL_PWM : APP_TRACK_BASE_PWM;
    correction = gLinePidCorrection;
    targetPwmA = clamp_i32(basePwm - correction, 0, 8000);
    targetPwmB = clamp_i32(basePwm + correction, 0, 8000);

    gCommandPwmA = (uint16_t) slew_i32((int32_t) gCommandPwmA,
        targetPwmA, APP_TRACK_PWM_SLEW_PER_TICK);
    gCommandPwmB = (uint16_t) slew_i32((int32_t) gCommandPwmB,
        targetPwmB, APP_TRACK_PWM_SLEW_PER_TICK);
    motor_control_drive_pwm_5ms(gMs, gCommandPwmA, gCommandPwmB);
    if (motor_control_stalled()) {
        enter_fault();
    }
}

static void drive_tick_5ms(void)
{
    uint8_t lineMask;
    uint32_t distance;
    uint32_t elapsed;

    if (button_pressed_event() &&
        ((gDriveState == DRIVE_IDLE) ||
         (gDriveState == DRIVE_COMPLETE) ||
         (gDriveState == DRIVE_FAULT))) {
        start_run();
    }

    if ((gDriveState == DRIVE_IDLE) ||
        (gDriveState == DRIVE_COMPLETE)) {
        DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
        return;
    }
    if (gDriveState == DRIVE_FAULT) {
        if (elapsed_ms(gFaultLedLastToggleMs,
                APP_FAULT_LED_TOGGLE_MS)) {
            gFaultLedLastToggleMs = gMs;
            DL_GPIO_togglePins(LED_PORT, LED_led_PIN);
        }
        return;
    }

    lineMask = line_sensor_read_mask();
    distance = traveled_counts();
    elapsed = (uint32_t) (gMs - gRunStartMs);

    update_finish_logic(lineMask, distance, elapsed);
    if (gDriveState == DRIVE_COMPLETE) {
        return;
    }

    update_line_control(lineMask);
    if ((gDriveState != DRIVE_FAULT) &&
        (elapsed >= APP_RUN_TIMEOUT_MS)) {
        enter_fault();
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
    line_sensor_init();
    motor_control_init();
    DL_GPIO_clearPins(LED_PORT, LED_led_PIN);

    gButtonRaw = ((DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) &
                      KEY_key_PIN) != 0U) ? 1U : 0U;
    gButtonStable = gButtonRaw;
    gButtonRawChangedMs = 0U;

    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    oled_init();
    __disable_irq();
    gControlTicksPending = 0U;
    __enable_irq();
    oled_service();

    while (1) {
        if (take_control_tick()) {
            drive_tick_5ms();
            oled_service();
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

void GROUP1_IRQHandler(void)
{
    uint32_t encoderAStatus = DL_GPIO_getEnabledInterruptStatus(
        ENCODERA_PORT, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    uint32_t encoderBStatus = DL_GPIO_getEnabledInterruptStatus(
        ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if ((encoderAStatus &
            (ENCODERA_E1A_PIN | ENCODERA_E1B_PIN)) != 0U) {
        motor_control_handle_encoder_a_irq();
    }
    if ((encoderBStatus &
            (ENCODERB_E2A_PIN | ENCODERB_E2B_PIN)) != 0U) {
        motor_control_handle_encoder_b_irq();
    }

    DL_GPIO_clearInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}
