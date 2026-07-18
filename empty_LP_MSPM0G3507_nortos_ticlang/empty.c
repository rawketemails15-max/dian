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

#define CONTROL_PERIOD_MS                   (5U)
#define KEY_DEBOUNCE_MS                     (40U)
#define KEY_LONG_PRESS_MS                   (1000U)
#define KEY_DOUBLE_PRESS_MS                 (1000U)
#define SEGMENT_TIMEOUT_MS                  (29500U)
#define TASK3_HALF_ARC_COUNT                (8U)
#define TASK3_HEADING_OFFSET_MDEG           (40000L)
#define TASK3_HALF_TURN_MDEG                (180000L)
#define TASK3_FULL_TURN_MDEG                (360000L)
#define TASK3_HEADING_TOLERANCE_MDEG        (1500L)
#define TASK3_TURN_SETTLE_TICKS             (20U)
#define TASK3_TURN_FAST_PWM                 MOTOR_PWM_CRUISE
#define TASK3_TURN_SLOW_PWM                 (1800U)
#define TASK3_TURN_FINE_PWM                 (1600U)

#define MOTOR_PWM_PERIOD                    (8000)
#define MOTOR_PWM_CRUISE                    (2300)

#define OLED_UPDATE_PERIOD_MS               (100U)

/*
 * Field geometry target:
 *   straight AB/CD length = 1000 mm -> about 7640 encoder counts.
 *   semicircle BC/DA radius = 400 mm -> about 9600 encoder counts.
 *
 * The odometry values are derived from a 65 mm wheel and 390 encoder pulses
 * per wheel revolution sampled on all four quadrature edges. They remain the
 * main on-car calibration points because wheel diameter and encoder PPR have
 * not yet been measured on this chassis.
 */
#define STRAIGHT_MIN_COUNTS_FOR_LINE        (5600U)
#define STRAIGHT_TARGET_COUNTS              (7640U)
#define ARC_FORCE_ENABLE_COUNTS             (8000U)
#define ARC_TARGET_COUNTS                    (9600U)
#define ARC_MAX_COUNTS                       (11000U)
#define ARC_HEADING_CHANGE_MDEG              (186000L)
#define ARC_LINE_LOSS_CONFIRM_TICKS          (4U)
#define ARC_TURN_DIRECTION_MIN_MDEG          (5000L)
#define ARC_FORCE_TURN_SLOW_ZONE_MDEG        (30000L)
#define ARC_FORCE_TURN_FINE_ZONE_MDEG        (8000L)
#define ARC_FORCE_TURN_TOLERANCE_MDEG        (1500L)
#define ARC_FORCE_TURN_SETTLE_TICKS          (20U)
#define ARC_FORCE_TURN_FAST_PWM               MOTOR_PWM_CRUISE
#define ARC_FORCE_TURN_SLOW_PWM              (1800U)
#define ARC_FORCE_TURN_FINE_PWM              (1600U)

/*
 * Fixed C07A + S28A grayscale binding from hardware.md:
 *   AD0 -> PA22, AD1 -> PA8, AD2 -> PA12, OUT -> PA27.
 * Power the module from 5 V and share GND with the controller. OUT is a
 * digital signal whose project-confirmed maximum level is 3.3 V.
 */
#define LINE_SENSOR_CHANNEL_COUNT           (8U)
#define LINE_ADDR0_PORT                     LINE_SENSOR_PORT
#define LINE_ADDR0_PIN                      LINE_SENSOR_AD0_PIN
#define LINE_ADDR1_PORT                     LINE_SENSOR_PORT
#define LINE_ADDR1_PIN                      LINE_SENSOR_AD1_PIN
#define LINE_ADDR2_PORT                     LINE_SENSOR_PORT
#define LINE_ADDR2_PIN                      LINE_SENSOR_AD2_PIN
#define LINE_OUT_PORT                       LINE_SENSOR_PORT
#define LINE_OUT_PIN                        LINE_SENSOR_OUT_PIN

/* Calibrated on the installed sensor: black line outputs high level. */
#define LINE_BLACK_LEVEL                    (1U)
#define LINE_REVERSE_ORDER                  (0U)
/* Initial bring-up value; calibrate it on the installed sensor and track. */
#define LINE_MUX_SETTLE_US                  (100U)
#define LINE_MUX_SETTLE_CYCLES              \
    ((CPUCLK_FREQ / 1000000U) * LINE_MUX_SETTLE_US)
#define LINE_ERROR_MAX                      (350)
#define LINE_PWM_CORRECTION_MAX             (4000)

/* Fixed C07A OLED serial-GPIO binding from the board reference project. */
#define DISPLAY_RST_PORT                    OLED_RST_PORT
#define DISPLAY_RST_PIN                     OLED_RST_RST_PIN
#define DISPLAY_DC_PORT                     OLED_DC_PORT
#define DISPLAY_DC_PIN                      OLED_DC_DC_PIN
#define DISPLAY_SCL_PORT                    OLED_SCL_PORT
#define DISPLAY_SCL_PIN                     OLED_SCL_SCL_PIN
#define DISPLAY_SDA_PORT                    OLED_SDA_PORT
#define DISPLAY_SDA_PIN                     OLED_SDA_SDA_PIN

#define MPU6050_ADDRESS                     (0x68U)
#define MPU6050_REG_SMPLRT_DIV              (0x19U)
#define MPU6050_REG_CONFIG                  (0x1AU)
#define MPU6050_REG_GYRO_CONFIG             (0x1BU)
#define MPU6050_REG_INT_PIN_CFG             (0x37U)
#define MPU6050_REG_INT_ENABLE              (0x38U)
#define MPU6050_REG_GYRO_ZOUT_H             (0x47U)
#define MPU6050_REG_PWR_MGMT_1              (0x6BU)
#define MPU6050_REG_PWR_MGMT_2              (0x6CU)
#define MPU6050_REG_WHO_AM_I                (0x75U)
#define MPU6050_WHO_AM_I_VALUE              (0x68U)
#define MPU6050_GYRO_Z_SIGN                 (1)
#define MPU6050_I2C_TIMEOUT_MS              (10U)
#define MPU6050_CALIBRATION_SAMPLES         (200U)
#define MPU6050_CALIBRATION_TIMEOUT_MS      (2000U)
#define MPU6050_MIN_CALIBRATION_SAMPLES     (160U)
#define MPU6050_FALLBACK_SAMPLE_MS          (5U)
/* At +/-500 dps, 65.5 LSB/dps = 131/2 LSB/dps. */
#define MPU6050_GYRO_SCALE_DENOMINATOR      (131L)

#define MOTOR_A_PORT                        AIN_PORT
#define MOTOR_AIN1_PIN                      AIN_AIN1_PIN
#define MOTOR_AIN2_PIN                      AIN_AIN2_PIN
#define MOTOR_B_PORT                        BIN_PORT
#define MOTOR_BIN1_PIN                      BIN_BIN1_PIN
#define MOTOR_BIN2_PIN                      BIN_BIN2_PIN

#define ENCODER_A_PORT                      ENCODERA_PORT
#define ENCODER_A1_PIN                      ENCODERA_E1A_PIN
#define ENCODER_A2_PIN                      ENCODERA_E1B_PIN
#define ENCODER_B_PORT                      ENCODERB_PORT
#define ENCODER_B1_PIN                      ENCODERB_E2A_PIN
#define ENCODER_B2_PIN                      ENCODERB_E2B_PIN

#define USER_KEY_PORT                       KEY_PORT
#define USER_KEY_PIN                        KEY_key_PIN

typedef enum {
    DRIVE_IDLE = 0,
    DRIVE_AB_STRAIGHT,
    DRIVE_BC_ARC,
    DRIVE_CD_STRAIGHT,
    DRIVE_DA_ARC,
    DRIVE_TASK3_ALIGN_AXIS,
    DRIVE_TASK3_ALIGN_OFFSET,
    DRIVE_TASK3_STRAIGHT,
    DRIVE_TASK3_ARC,
    DRIVE_COMPLETE,
    DRIVE_FAULT
} DriveMode;

typedef enum {
    ROUTE_MISSION_FIRST_STAGE = 0,
    ROUTE_MISSION_FULL,
    ROUTE_MISSION_TASK3
} RouteMission;

static volatile uint32_t gMs;
static volatile int32_t gEncoderA;
static volatile int32_t gEncoderB;
static volatile uint8_t gControlTicksPending;
static volatile bool gImuDataReady;

static DriveMode gDriveMode = DRIVE_IDLE;
static RouteMission gRouteMission = ROUTE_MISSION_FIRST_STAGE;
static int16_t gLastLineError;
static uint8_t gEncoderAPreviousState;
static uint8_t gEncoderBPreviousState;
static bool gStraightHasSeenWhite;
static bool gArcForceHeading;
static bool gArcFallbackStraight;
static bool gArcLineSeen;
static uint8_t gArcLineLostTicks;
static int8_t gArcHeadingDirection;
static bool gArcForceTurnRight;
static uint8_t gArcTurnSettleTicks;
static uint32_t gSegmentStartMs;
static int32_t gRouteZeroHeadingMdeg;
static int32_t gArcStartHeadingMdeg;
static uint16_t gStraightTargetCount;
static uint32_t gStraightLastControlMs;
static bool gBuzzerActive;
static uint32_t gBuzzerStartMs;
static bool gPointLightActive;
static uint32_t gPointLightStartMs;
static uint8_t gTask3HalfArcCount;
static uint8_t gTask3TurnSettleTicks;
static int32_t gTask3AxisTargetMdeg;

static bool gImuReady;
static int32_t gGyroZBiasRaw;
static int32_t gHeadingMdeg;
static int32_t gHeadingRemainder;
static uint32_t gImuLastSampleMs;

static int32_t relative_heading_mdeg(void)
{
    /* Positive heading is a left turn; right turns reduce the angle. */
    return gHeadingMdeg - gRouteZeroHeadingMdeg;
}

static int32_t task3_heading_error_mdeg(
    int32_t targetHeadingMdeg, int32_t currentHeadingMdeg)
{
    int32_t errorMdeg = targetHeadingMdeg - currentHeadingMdeg;

    /* 0 and 360 deg are the same chassis direction. Keep +180 deg so the
     * first C-to-B half-circle makeup uses the required left turn. */
    while (errorMdeg > TASK3_HALF_TURN_MDEG) {
        errorMdeg -= TASK3_FULL_TURN_MDEG;
    }
    while (errorMdeg < -TASK3_HALF_TURN_MDEG) {
        errorMdeg += TASK3_FULL_TURN_MDEG;
    }
    return errorMdeg;
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static uint32_t abs_encoder_count(int32_t count)
{
    return (uint32_t) ((count < 0) ? -count : count);
}

static bool elapsed_ms(uint32_t startMs, uint32_t durationMs)
{
    return (uint32_t) (gMs - startMs) >= durationMs;
}

static void buzzer_pulse(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_buzzer_PIN);
    gBuzzerStartMs = gMs;
    gBuzzerActive = true;
}

static void buzzer_service(void)
{
    if (gBuzzerActive &&
        elapsed_ms(gBuzzerStartMs, APP_BUZZER_PULSE_MS)) {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_buzzer_PIN);
        gBuzzerActive = false;
    }
}

static void signal_route_point(void)
{
    buzzer_pulse();
    if (gRouteMission == ROUTE_MISSION_TASK3) {
        DL_GPIO_setPins(LED_PORT, LED_led_PIN);
        gPointLightActive = true;
        gPointLightStartMs = gMs;
    }
}

static void point_light_service(void)
{
    if (gPointLightActive &&
        elapsed_ms(gPointLightStartMs, APP_BUZZER_PULSE_MS)) {
        DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
        gPointLightActive = false;
    }
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
        for (uint8_t column = 0U; column < 128U; column++) {
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

static const uint8_t *oled_glyph(char character)
{
    static const char characters[] = " 0123456789+-.:ADEGLN";
    static const uint8_t glyphs[][5] = {
        {0x00U, 0x00U, 0x00U, 0x00U, 0x00U},
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
        {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
        {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
        {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
        {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU},
        {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U},
        {0x08U, 0x08U, 0x08U, 0x08U, 0x08U},
        {0x00U, 0x60U, 0x60U, 0x00U, 0x00U},
        {0x00U, 0x36U, 0x36U, 0x00U, 0x00U},
        {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
        {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
        {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
        {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
        {0x7FU, 0x02U, 0x0CU, 0x10U, 0x7FU},
    };

    for (uint8_t index = 0U;
         index < (uint8_t) (sizeof(characters) - 1U); index++) {
        if (characters[index] == character) {
            return glyphs[index];
        }
    }
    return glyphs[0];
}

static void oled_write_text(uint8_t column, uint8_t page, const char *text)
{
    oled_set_position(column, page);
    while (*text != '\0') {
        const uint8_t *glyph = oled_glyph(*text++);

        for (uint8_t glyphColumn = 0U; glyphColumn < 5U; glyphColumn++) {
            oled_write_byte(glyph[glyphColumn], true);
        }
        oled_write_byte(0U, true);
    }
}

static void oled_update_angle(void)
{
    static uint32_t lastUpdateMs;
    char text[] = "ANGLE:+000.0 DEG";
    int32_t angleMdeg = relative_heading_mdeg();
    uint32_t angleTenths;

    if (!elapsed_ms(lastUpdateMs, OLED_UPDATE_PERIOD_MS)) {
        return;
    }
    lastUpdateMs = gMs;

    if (angleMdeg < 0) {
        text[6] = '-';
        angleTenths = (uint32_t) ((-angleMdeg + 50L) / 100L);
    } else {
        angleTenths = (uint32_t) ((angleMdeg + 50L) / 100L);
    }
    if (angleTenths > 9999U) {
        angleTenths = 9999U;
    }

    text[7] = (char) ('0' + ((angleTenths / 1000U) % 10U));
    text[8] = (char) ('0' + ((angleTenths / 100U) % 10U));
    text[9] = (char) ('0' + ((angleTenths / 10U) % 10U));
    text[11] = (char) ('0' + (angleTenths % 10U));
    oled_write_text(16U, 3U, text);
}

static void line_sensor_init(void)
{
    DL_GPIO_clearPins(LINE_ADDR0_PORT, LINE_ADDR0_PIN);
    DL_GPIO_clearPins(LINE_ADDR1_PORT, LINE_ADDR1_PIN);
    DL_GPIO_clearPins(LINE_ADDR2_PORT, LINE_ADDR2_PIN);

    /* Allow the address and sensor output to settle before the first scan. */
    DL_Common_delayCycles(LINE_MUX_SETTLE_CYCLES);
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

static uint8_t line_sensor_read_mask(void)
{
    uint8_t mask = 0U;

    for (uint8_t channel = 0U; channel < LINE_SENSOR_CHANNEL_COUNT; channel++) {
        line_sensor_select(channel);
        DL_Common_delayCycles(LINE_MUX_SETTLE_CYCLES);

        uint8_t level = ((DL_GPIO_readPins(LINE_OUT_PORT, LINE_OUT_PIN) & LINE_OUT_PIN) != 0U)
                            ? 1U
                            : 0U;
        if (level == LINE_BLACK_LEVEL) {
            mask |= (uint8_t) (1U << channel);
        }
    }

    return mask;
}

static int16_t line_error_from_mask(uint8_t mask)
{
    static const int16_t weights[8] = {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum = 0;
    uint8_t count = 0U;

    if (mask == 0xFFU) {
        gLastLineError = 0;
        return 0;
    }

    for (uint8_t channel = 0U; channel < 8U; channel++) {
        if ((mask & (uint8_t) (1U << channel)) != 0U) {
            uint8_t weightIndex = (LINE_REVERSE_ORDER != 0U) ? (uint8_t) (7U - channel) : channel;
            sum += weights[weightIndex];
            count++;
        }
    }

    if (count == 0U) {
        return gLastLineError;
    }

    gLastLineError = (int16_t) (sum / count);
    return gLastLineError;
}

static bool i2c_wait_for_idle(void)
{
    uint32_t startMs = gMs;

    while ((DL_I2C_getControllerStatus(I2C_IMU_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (elapsed_ms(startMs, MPU6050_I2C_TIMEOUT_MS)) {
            return false;
        }
    }

    return true;
}

static void i2c_reset_transfer(void)
{
    DL_I2C_resetControllerTransfer(I2C_IMU_INST);
    DL_I2C_flushControllerTXFIFO(I2C_IMU_INST);
    DL_I2C_flushControllerRXFIFO(I2C_IMU_INST);
    DL_I2C_clearInterruptStatus(I2C_IMU_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
}

static bool i2c_transmit(const uint8_t *data, uint8_t length)
{
    const uint32_t errorMask = DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST;
    uint32_t startMs;

    if (!i2c_wait_for_idle()) {
        i2c_reset_transfer();
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_IMU_INST);
    DL_I2C_clearInterruptStatus(I2C_IMU_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | errorMask);

    if (DL_I2C_fillControllerTXFIFO(I2C_IMU_INST, data, length) != length) {
        i2c_reset_transfer();
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_IMU_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);
    startMs = gMs;

    while (true) {
        uint32_t status = DL_I2C_getRawInterruptStatus(I2C_IMU_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | errorMask);

        if ((status & errorMask) != 0U) {
            i2c_reset_transfer();
            return false;
        }
        if ((status & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            return true;
        }
        if (elapsed_ms(startMs, MPU6050_I2C_TIMEOUT_MS)) {
            i2c_reset_transfer();
            return false;
        }
    }
}

static bool i2c_receive(uint8_t *data, uint8_t length)
{
    const uint32_t errorMask = DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST;
    uint32_t startMs;
    uint8_t received = 0U;

    if (!i2c_wait_for_idle()) {
        i2c_reset_transfer();
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_IMU_INST);
    DL_I2C_clearInterruptStatus(I2C_IMU_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | errorMask);
    DL_I2C_startControllerTransfer(I2C_IMU_INST, MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);
    startMs = gMs;

    while (true) {
        uint32_t status = DL_I2C_getRawInterruptStatus(I2C_IMU_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | errorMask);

        while (!DL_I2C_isControllerRXFIFOEmpty(I2C_IMU_INST)) {
            uint8_t value = DL_I2C_receiveControllerData(I2C_IMU_INST);

            if (received < length) {
                data[received++] = value;
            }
        }

        if ((status & errorMask) != 0U) {
            i2c_reset_transfer();
            return false;
        }
        if ((status & DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) != 0U) {
            return received == length;
        }
        if (elapsed_ms(startMs, MPU6050_I2C_TIMEOUT_MS)) {
            i2c_reset_transfer();
            return false;
        }
    }
}

static bool mpu6050_write_register(uint8_t registerAddress, uint8_t value)
{
    uint8_t data[2] = {registerAddress, value};

    return i2c_transmit(data, 2U);
}

static bool mpu6050_read_registers(
    uint8_t registerAddress, uint8_t *data, uint8_t length)
{
    return i2c_transmit(&registerAddress, 1U) && i2c_receive(data, length);
}

static bool mpu6050_read_gyro_z(int16_t *gyroZ)
{
    uint8_t data[2];

    if (!mpu6050_read_registers(MPU6050_REG_GYRO_ZOUT_H, data, 2U)) {
        return false;
    }

    *gyroZ = (int16_t) (((uint16_t) data[0] << 8U) | data[1]);
    return true;
}

static bool mpu6050_init_and_calibrate(void)
{
    uint8_t whoAmI;
    int64_t biasSum = 0;
    uint16_t sampleCount = 0U;
    uint32_t calibrationStartMs;
    uint32_t lastAttemptMs;

    if (!mpu6050_read_registers(MPU6050_REG_WHO_AM_I, &whoAmI, 1U) ||
        (whoAmI != MPU6050_WHO_AM_I_VALUE)) {
        return false;
    }

    if (!mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x80U)) {
        return false;
    }
    wait_ms(100U);

    if (!mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x01U) ||
        !mpu6050_write_register(MPU6050_REG_PWR_MGMT_2, 0x00U) ||
        !mpu6050_write_register(MPU6050_REG_CONFIG, 0x03U) ||
        !mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, 0x08U) ||
        !mpu6050_write_register(MPU6050_REG_SMPLRT_DIV, 0x04U) ||
        !mpu6050_write_register(MPU6050_REG_INT_PIN_CFG, 0x00U) ||
        !mpu6050_write_register(MPU6050_REG_INT_ENABLE, 0x01U)) {
        return false;
    }

    wait_ms(20U);
    gImuDataReady = false;
    calibrationStartMs = gMs;
    lastAttemptMs = gMs;

    while ((sampleCount < MPU6050_CALIBRATION_SAMPLES) &&
        !elapsed_ms(calibrationStartMs, MPU6050_CALIBRATION_TIMEOUT_MS)) {
        if (gImuDataReady || elapsed_ms(lastAttemptMs, CONTROL_PERIOD_MS)) {
            int16_t rawGyroZ;

            gImuDataReady = false;
            lastAttemptMs = gMs;
            if (mpu6050_read_gyro_z(&rawGyroZ)) {
                biasSum += rawGyroZ;
                sampleCount++;
            }
        } else {
            __WFI();
        }
    }

    if (sampleCount < MPU6050_MIN_CALIBRATION_SAMPLES) {
        return false;
    }

    gGyroZBiasRaw = (int32_t) (biasSum / sampleCount);
    gHeadingMdeg = 0;
    gHeadingRemainder = 0;
    gImuLastSampleMs = gMs;
    return true;
}

static void imu_update_heading(void)
{
    uint32_t nowMs = gMs;
    uint32_t deltaMs;
    int16_t rawGyroZ;
    int32_t rateRaw;
    int32_t numerator;

    if (!gImuReady ||
        (!gImuDataReady &&
            !elapsed_ms(gImuLastSampleMs, MPU6050_FALLBACK_SAMPLE_MS))) {
        return;
    }

    gImuDataReady = false;
    if (!mpu6050_read_gyro_z(&rawGyroZ)) {
        return;
    }

    deltaMs = (uint32_t) (nowMs - gImuLastSampleMs);
    if (deltaMs == 0U) {
        return;
    }
    if (deltaMs > 20U) {
        deltaMs = 20U;
    }

    rateRaw = ((int32_t) rawGyroZ - gGyroZBiasRaw) * MPU6050_GYRO_Z_SIGN;
    numerator = (rateRaw * (int32_t) deltaMs * 2) + gHeadingRemainder;
    gHeadingMdeg += numerator / MPU6050_GYRO_SCALE_DENOMINATOR;
    gHeadingRemainder = numerator % MPU6050_GYRO_SCALE_DENOMINATOR;
    gImuLastSampleMs = nowMs;
}

static void imu_reset_route_angle(void)
{
    /*
     * Gyro bias is calibrated at power-up, but angle integration belongs to
     * the route.  Start a fresh route angle without recalibrating the sensor.
     */
    gHeadingMdeg = 0;
    gHeadingRemainder = 0;
    gRouteZeroHeadingMdeg = 0;
    gImuLastSampleMs = gMs;
    gImuDataReady = false;
}

static void imu_begin_arc_sampling(bool resetRouteAngle)
{
    if (resetRouteAngle) {
        imu_reset_route_angle();
    } else {
        /* Exclude the preceding straight section from the next integration. */
        gImuLastSampleMs = gMs;
        gImuDataReady = false;
    }
}

static uint16_t clamp_forward_pwm(int32_t pwm)
{
    return (uint16_t) clamp_i32(pwm, 0, MOTOR_PWM_PERIOD);
}

static void set_motor_forward_pwm(uint16_t motorAPwm, uint16_t motorBPwm)
{
    motorAPwm = clamp_forward_pwm(motorAPwm);
    motorBPwm = clamp_forward_pwm(motorBPwm);

    /* Calibrated on the installed chassis: positive PWM means forward. */
    if (motorAPwm > 0) {
        DL_GPIO_setPins(MOTOR_A_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_A_PORT, MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_A_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    }

    if (motorBPwm > 0) {
        DL_GPIO_setPins(MOTOR_B_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    }

    DL_Timer_setCaptureCompareValue(
        PWM_0_INST, motorAPwm, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(
        PWM_0_INST, motorBPwm, GPIO_PWM_0_C1_IDX);
}

static void stop_car(void)
{
    set_motor_forward_pwm(0U, 0U);
}

static void apply_forward_steering(int32_t basePwm, int32_t correction)
{
    correction = clamp_i32(
        correction, -MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
    set_motor_forward_pwm(
        clamp_forward_pwm(basePwm - correction),
        clamp_forward_pwm(basePwm + correction));
}

static uint8_t encoder_state(
    GPIO_Regs *port, uint32_t channelAPin, uint32_t channelBPin)
{
    uint32_t pins = DL_GPIO_readPins(port, channelAPin | channelBPin);

    return (uint8_t) ((((pins & channelAPin) != 0U) ? 2U : 0U) |
        (((pins & channelBPin) != 0U) ? 1U : 0U));
}

static int8_t quadrature_delta(uint8_t previousState, uint8_t currentState)
{
    static const int8_t transitionTable[16] = {
        0, 1, -1, 0,
        -1, 0, 0, 1,
        1, 0, 0, -1,
        0, -1, 1, 0
    };

    return transitionTable[((previousState & 0x03U) << 2U) |
        (currentState & 0x03U)];
}

static void reset_segment_encoders(void)
{
    __disable_irq();
    gEncoderA = 0;
    gEncoderB = 0;
    __enable_irq();
}

static void read_segment_encoder_counts(
    uint32_t *motorACounts, uint32_t *motorBCounts)
{
    int32_t encoderA;
    int32_t encoderB;

    /* Snapshot both wheels at the same instant for speed comparison. */
    __disable_irq();
    encoderA = gEncoderA;
    encoderB = gEncoderB;
    __enable_irq();

    *motorACounts = abs_encoder_count(encoderA);
    *motorBCounts = abs_encoder_count(encoderB);
}

static uint32_t segment_average_counts(void)
{
    uint32_t motorACounts;
    uint32_t motorBCounts;

    read_segment_encoder_counts(&motorACounts, &motorBCounts);

    return (motorACounts + motorBCounts) / 2U;
}

static void update_straight_target(void)
{
    uint16_t nextTarget = (uint16_t) (gStraightTargetCount +
        APP_TRACK_BASE_RAMP_COUNTS);

    if (nextTarget > APP_TRACK_BASE_TARGET_COUNTS) {
        nextTarget = APP_TRACK_BASE_TARGET_COUNTS;
    }
    gStraightTargetCount = nextTarget;
}

static bool route_is_active(void)
{
    return (gDriveMode == DRIVE_AB_STRAIGHT) ||
        (gDriveMode == DRIVE_BC_ARC) ||
        (gDriveMode == DRIVE_CD_STRAIGHT) ||
        (gDriveMode == DRIVE_DA_ARC) ||
        (gDriveMode == DRIVE_TASK3_ALIGN_AXIS) ||
        (gDriveMode == DRIVE_TASK3_ALIGN_OFFSET) ||
        (gDriveMode == DRIVE_TASK3_STRAIGHT) ||
        (gDriveMode == DRIVE_TASK3_ARC);
}

static void stop_route(DriveMode stoppedMode)
{
    gDriveMode = stoppedMode;
    gLastLineError = 0;
    motor_control_brake();
    stop_car();
}

static void begin_straight(DriveMode straightMode, bool alreadyOnWhite)
{
    reset_segment_encoders();
    gStraightTargetCount = 0U;
    gStraightLastControlMs = gMs;
    motor_control_start(gMs);
    gSegmentStartMs = gMs;
    gStraightHasSeenWhite = alreadyOnWhite;
    gLastLineError = 0;
    gDriveMode = straightMode;
}

static void begin_arc(DriveMode arcMode)
{
    reset_segment_encoders();
    gSegmentStartMs = gMs;
    /* BC is the angular origin; DA continues from BC's accumulated angle. */
    imu_begin_arc_sampling(arcMode == DRIVE_BC_ARC);
    gArcStartHeadingMdeg = gHeadingMdeg;
    gArcForceHeading = false;
    gArcFallbackStraight = false;
    gArcLineSeen = false;
    gArcLineLostTicks = 0U;
    gArcHeadingDirection = -1;
    gArcForceTurnRight = true;
    gArcTurnSettleTicks = 0U;
    gLastLineError = 0;
    gDriveMode = arcMode;
}

static void begin_task3_heading(DriveMode headingMode)
{
    stop_car();
    gTask3TurnSettleTicks = 0U;
    gSegmentStartMs = gMs;
    gDriveMode = headingMode;
}

static void begin_task3_straight(void)
{
    begin_straight(DRIVE_TASK3_STRAIGHT, false);
}

static void begin_task3_arc(void)
{
    begin_arc(DRIVE_TASK3_ARC);
}

static void start_route(RouteMission mission)
{
    /*
     * Gyro bias is calibrated once at power-up.  Angle integration does not
     * begin here: AB must display 0 deg, and BC will establish route 0 deg.
     */
    stop_car();
    gRouteMission = mission;
    if (!gImuReady) {
        gImuReady = mpu6050_init_and_calibrate();
    }
    imu_reset_route_angle();
    gPointLightActive = false;
    if (mission == ROUTE_MISSION_TASK3) {
        /* Task 3 owns PB9 as the point indicator; old missions stay unchanged. */
        DL_GPIO_clearPins(LED_PORT, LED_led_PIN);
        gTask3HalfArcCount = 0U;
        gTask3AxisTargetMdeg = 0;
        motor_control_start(gMs);
    } else {
        DL_GPIO_setPins(LED_PORT, LED_led_PIN);
    }
    __disable_irq();
    gControlTicksPending = 0U;
    __enable_irq();
    if ((mission == ROUTE_MISSION_TASK3) && !gImuReady) {
        stop_route(DRIVE_FAULT);
    } else if (mission == ROUTE_MISSION_TASK3) {
        begin_task3_heading(DRIVE_TASK3_ALIGN_AXIS);
    } else {
        begin_straight(DRIVE_AB_STRAIGHT, false);
    }
}

static void toggle_first_stage_route(void)
{
    if (route_is_active()) {
        stop_route(DRIVE_IDLE);
    } else {
        start_route(ROUTE_MISSION_FIRST_STAGE);
    }
}

static void finish_straight(void)
{
    if (gDriveMode == DRIVE_TASK3_STRAIGHT) {
        begin_task3_arc();
    } else if (gDriveMode == DRIVE_AB_STRAIGHT) {
        if (gRouteMission == ROUTE_MISSION_FIRST_STAGE) {
            stop_route(DRIVE_COMPLETE);
        } else {
            begin_arc(DRIVE_BC_ARC);
        }
    } else {
        begin_arc(DRIVE_DA_ARC);
    }
}

static void finish_arc(void)
{
    if (gDriveMode == DRIVE_BC_ARC) {
        begin_straight(DRIVE_CD_STRAIGHT, false);
    } else {
        stop_route(DRIVE_COMPLETE);
    }
}

static int32_t arc_forced_turn_remaining_mdeg(void)
{
    int32_t halfTurnCount =
        (gDriveMode == DRIVE_DA_ARC) ? 2L : 1L;
    int32_t targetHeadingMdeg = gRouteZeroHeadingMdeg +
        ((int32_t) gArcHeadingDirection * halfTurnCount *
            ARC_HEADING_CHANGE_MDEG);

    return (gArcHeadingDirection > 0)
               ? targetHeadingMdeg - gHeadingMdeg
               : gHeadingMdeg - targetHeadingMdeg;
}

static void drive_task3_heading_5ms(void)
{
    int32_t targetHeadingMdeg = gTask3AxisTargetMdeg;
    int32_t errorMdeg;
    int32_t absoluteErrorMdeg;
    uint16_t turnPwm;

    if (gDriveMode == DRIVE_TASK3_ALIGN_OFFSET) {
        if (gTask3HalfArcCount == 0U) {
            /* A-to-C initial diagonal: 0 -> -40 deg is a right turn. */
            targetHeadingMdeg -= TASK3_HEADING_OFFSET_MDEG;
        } else if ((gTask3HalfArcCount & 0x01U) != 0U) {
            /* After C-to-B: 180 -> 220 deg is a left turn. */
            targetHeadingMdeg += TASK3_HEADING_OFFSET_MDEG;
        } else {
            /* After D-to-A: 360 -> 320 deg is a right turn. */
            targetHeadingMdeg -= TASK3_HEADING_OFFSET_MDEG;
        }
    }

    errorMdeg = task3_heading_error_mdeg(
        targetHeadingMdeg, relative_heading_mdeg());
    absoluteErrorMdeg = (errorMdeg < 0) ? -errorMdeg : errorMdeg;

    if (absoluteErrorMdeg <= TASK3_HEADING_TOLERANCE_MDEG) {
        stop_car();
        if (gTask3TurnSettleTicks < TASK3_TURN_SETTLE_TICKS) {
            gTask3TurnSettleTicks++;
        }
        if (gTask3TurnSettleTicks >= TASK3_TURN_SETTLE_TICKS) {
            if (gDriveMode == DRIVE_TASK3_ALIGN_AXIS) {
                begin_task3_heading(DRIVE_TASK3_ALIGN_OFFSET);
            } else {
                begin_task3_straight();
            }
        }
        return;
    }
    gTask3TurnSettleTicks = 0U;

    if (absoluteErrorMdeg <= ARC_FORCE_TURN_FINE_ZONE_MDEG) {
        turnPwm = TASK3_TURN_FINE_PWM;
    } else if (absoluteErrorMdeg <= ARC_FORCE_TURN_SLOW_ZONE_MDEG) {
        turnPwm = TASK3_TURN_SLOW_PWM;
    } else {
        turnPwm = TASK3_TURN_FAST_PWM;
    }

    /* Positive heading error commands a left pivot; negative commands right. */
    if (errorMdeg > 0) {
        set_motor_forward_pwm(turnPwm, 0U);
    } else {
        set_motor_forward_pwm(0U, turnPwm);
    }
}

static void drive_arc_forced_turn_5ms(void)
{
    int32_t remainingMdeg;
    int32_t absoluteRemainingMdeg;
    uint16_t turnPwm;
    bool turnRight;

    if (!gImuReady) {
        /* A missing IMU must not turn a line-loss event into an immediate
         * stop. Fall back to straight travel until the encoder limit. */
        gArcForceHeading = false;
        gArcFallbackStraight = true;
        gLastLineError = 0;
        set_motor_forward_pwm(MOTOR_PWM_CRUISE, MOTOR_PWM_CRUISE);
        return;
    }

    remainingMdeg = arc_forced_turn_remaining_mdeg();
    absoluteRemainingMdeg = (remainingMdeg < 0)
                                ? -remainingMdeg
                                : remainingMdeg;

    if (absoluteRemainingMdeg <= ARC_FORCE_TURN_TOLERANCE_MDEG) {
        stop_car();
        if (gArcTurnSettleTicks < ARC_FORCE_TURN_SETTLE_TICKS) {
            gArcTurnSettleTicks++;
        }
        if (gArcTurnSettleTicks >= ARC_FORCE_TURN_SETTLE_TICKS) {
            finish_arc();
        }
        return;
    }
    gArcTurnSettleTicks = 0U;

    if (absoluteRemainingMdeg <= ARC_FORCE_TURN_FINE_ZONE_MDEG) {
        turnPwm = ARC_FORCE_TURN_FINE_PWM;
    } else if (absoluteRemainingMdeg <= ARC_FORCE_TURN_SLOW_ZONE_MDEG) {
        turnPwm = ARC_FORCE_TURN_SLOW_PWM;
    } else {
        turnPwm = ARC_FORCE_TURN_FAST_PWM;
    }

    /* A negative remaining angle means the target was crossed. Pivot the
     * other way at the reduced speed instead of accepting the overshoot. */
    turnRight = (remainingMdeg > 0) ? gArcForceTurnRight
                                    : !gArcForceTurnRight;

    if (turnRight) {
        set_motor_forward_pwm(0U, turnPwm);
    } else {
        set_motor_forward_pwm(turnPwm, 0U);
    }
}

static void drive_straight_5ms(void)
{
    uint8_t lineMask = line_sensor_read_mask();
    uint32_t motorACounts;
    uint32_t motorBCounts;
    uint32_t averageCounts;
    bool blackLineDetected;

    read_segment_encoder_counts(&motorACounts, &motorBCounts);
    averageCounts = (motorACounts + motorBCounts) / 2U;

    if (lineMask == 0U) {
        gStraightHasSeenWhite = true;
    }

    blackLineDetected = (lineMask != 0U) && gStraightHasSeenWhite &&
        (averageCounts >= STRAIGHT_MIN_COUNTS_FOR_LINE);

    if (blackLineDetected ||
        ((gRouteMission != ROUTE_MISSION_TASK3) &&
            (averageCounts >= STRAIGHT_TARGET_COUNTS))) {
        if (blackLineDetected) {
            signal_route_point();
        }
        finish_straight();
        return;
    }

    if ((uint32_t) (gMs - gStraightLastControlMs) >=
        APP_CONTROL_PERIOD_MS) {
        gStraightLastControlMs += APP_CONTROL_PERIOD_MS;
        update_straight_target();
        motor_control_update_20ms(gMs, gStraightTargetCount,
            gStraightTargetCount);
        if (motor_control_stalled()) {
            stop_route(DRIVE_FAULT);
        }
    }
}

static void drive_arc_5ms(void)
{
    uint8_t lineMask = line_sensor_read_mask();
    uint32_t averageCounts = segment_average_counts();
    int32_t lineCorrection;
    bool lineLossConfirmed;

    if (gArcForceHeading) {
        drive_arc_forced_turn_5ms();
        return;
    }

    if (gArcFallbackStraight) {
        set_motor_forward_pwm(MOTOR_PWM_CRUISE, MOTOR_PWM_CRUISE);
        if (averageCounts >= ARC_MAX_COUNTS) {
            finish_arc();
        }
        return;
    }

    lineCorrection = ((int32_t) line_error_from_mask(lineMask) *
                         LINE_PWM_CORRECTION_MAX) /
        LINE_ERROR_MAX;

    apply_forward_steering(MOTOR_PWM_CRUISE, lineCorrection);

    if (lineMask != 0U) {
        gArcLineSeen = true;
        gArcLineLostTicks = 0U;
    } else if (gArcLineSeen &&
               (averageCounts > ARC_FORCE_ENABLE_COUNTS)) {
        if (gArcLineLostTicks < ARC_LINE_LOSS_CONFIRM_TICKS) {
            gArcLineLostTicks++;
        }
    } else {
        gArcLineLostTicks = 0U;
    }

    lineLossConfirmed =
        gArcLineLostTicks >= ARC_LINE_LOSS_CONFIRM_TICKS;

    if (lineLossConfirmed || (averageCounts >= ARC_MAX_COUNTS)) {
        int32_t headingDeltaMdeg =
            gHeadingMdeg - gArcStartHeadingMdeg;
        uint32_t motorACounts;
        uint32_t motorBCounts;

        if (lineLossConfirmed) {
            signal_route_point();
        }

        if (gDriveMode == DRIVE_TASK3_ARC) {
            if (!lineLossConfirmed) {
                stop_route(DRIVE_FAULT);
                return;
            }
            if (gTask3HalfArcCount < TASK3_HALF_ARC_COUNT) {
                gTask3HalfArcCount++;
            }
            if (gTask3HalfArcCount >= TASK3_HALF_ARC_COUNT) {
                stop_route(DRIVE_COMPLETE);
            } else if (!gImuReady) {
                stop_route(DRIVE_FAULT);
            } else {
                /* Accumulate the makeup axis: 180, 360, 540... degrees.
                 * Odd arcs then add 40 deg left; even arcs subtract 40 deg
                 * right. Wrapped heading error makes 360 deg equal 0 deg. */
                gTask3AxisTargetMdeg =
                    (int32_t) gTask3HalfArcCount * TASK3_HALF_TURN_MDEG;
                begin_task3_heading(DRIVE_TASK3_ALIGN_AXIS);
            }
            return;
        }

        read_segment_encoder_counts(&motorACounts, &motorBCounts);

        if (headingDeltaMdeg >= ARC_TURN_DIRECTION_MIN_MDEG) {
            gArcHeadingDirection = 1;
        } else if (headingDeltaMdeg <=
                   -ARC_TURN_DIRECTION_MIN_MDEG) {
            gArcHeadingDirection = -1;
        }

        if (motorBCounts > motorACounts) {
            gArcForceTurnRight = true;
        } else if (motorACounts > motorBCounts) {
            gArcForceTurnRight = false;
        } else if (gLastLineError != 0) {
            gArcForceTurnRight = gLastLineError > 0;
        }
        gArcForceHeading = true;
        gArcTurnSettleTicks = 0U;
        gSegmentStartMs = gMs;
        gLastLineError = 0;
        drive_arc_forced_turn_5ms();
    }
}

static bool key_pressed_raw(void)
{
    return (DL_GPIO_readPins(USER_KEY_PORT, USER_KEY_PIN) & USER_KEY_PIN) != 0U;
}

static void key_scan_5ms(void)
{
    static bool rawPrev;
    static bool debouncedPressed;
    static uint8_t stableTicks;
    static uint32_t pressStartMs;
    static bool longPressHandled;
    static bool firstShortPending;
    static uint32_t firstShortReleaseMs;
    static bool secondShortCandidate;
    bool rawPressed = key_pressed_raw();

    if (rawPressed == rawPrev) {
        if (stableTicks < (KEY_DEBOUNCE_MS / CONTROL_PERIOD_MS)) {
            stableTicks++;
        }
    } else {
        rawPrev = rawPressed;
        stableTicks = 0;
        if (rawPressed && firstShortPending) {
            if ((uint32_t) (gMs - firstShortReleaseMs) <=
                KEY_DOUBLE_PRESS_MS) {
                secondShortCandidate = true;
            } else {
                firstShortPending = false;
                secondShortCandidate = false;
                toggle_first_stage_route();
            }
        }
    }

    if ((stableTicks == (KEY_DEBOUNCE_MS / CONTROL_PERIOD_MS)) &&
        (rawPressed != debouncedPressed)) {
        debouncedPressed = rawPressed;
        if (debouncedPressed) {
            pressStartMs = gMs;
            longPressHandled = false;
            if (secondShortCandidate) {
                firstShortPending = false;
            }
        } else {
            if (!longPressHandled) {
                if (secondShortCandidate) {
                    secondShortCandidate = false;
                    start_route(ROUTE_MISSION_TASK3);
                } else {
                    firstShortPending = true;
                    firstShortReleaseMs = gMs;
                }
            }
        }
    }

    if (debouncedPressed && !longPressHandled &&
        ((gMs - pressStartMs) >= KEY_LONG_PRESS_MS)) {
        longPressHandled = true;
        firstShortPending = false;
        secondShortCandidate = false;
        start_route(ROUTE_MISSION_FULL);
    }

    if (firstShortPending && !rawPressed && !debouncedPressed &&
        ((uint32_t) (gMs - firstShortReleaseMs) >=
            KEY_DOUBLE_PRESS_MS)) {
        firstShortPending = false;
        secondShortCandidate = false;
        toggle_first_stage_route();
    }
}

static void drive_tick_5ms(void)
{
    if (route_is_active() &&
        elapsed_ms(gSegmentStartMs, SEGMENT_TIMEOUT_MS)) {
        stop_route(DRIVE_FAULT);
        return;
    }

    switch (gDriveMode) {
    case DRIVE_AB_STRAIGHT:
    case DRIVE_CD_STRAIGHT:
    case DRIVE_TASK3_STRAIGHT:
        drive_straight_5ms();
        break;

    case DRIVE_BC_ARC:
    case DRIVE_DA_ARC:
    case DRIVE_TASK3_ARC:
        drive_arc_5ms();
        break;

    case DRIVE_TASK3_ALIGN_AXIS:
    case DRIVE_TASK3_ALIGN_OFFSET:
        drive_task3_heading_5ms();
        break;

    case DRIVE_IDLE:
    case DRIVE_COMPLETE:
    case DRIVE_FAULT:
    default:
        stop_car();
        break;
    }
}

static bool take_control_tick(void)
{
    bool available;

    __disable_irq();
    available = gControlTicksPending != 0U;
    if (available) {
        gControlTicksPending--;
    }
    __enable_irq();

    return available;
}

int main(void)
{
    SYSCFG_DL_init();
    line_sensor_init();
    motor_control_init();

    DL_GPIO_setPins(LED_PORT, LED_led_PIN);
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_buzzer_PIN);
    stop_car();

    gEncoderAPreviousState =
        encoder_state(ENCODER_A_PORT, ENCODER_A1_PIN, ENCODER_A2_PIN);
    gEncoderBPreviousState =
        encoder_state(ENCODER_B_PORT, ENCODER_B1_PIN, ENCODER_B2_PIN);

    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(TIMER_0_INST);

    oled_init();
    gImuReady = mpu6050_init_and_calibrate();
    imu_reset_route_angle();
    __disable_irq();
    gControlTicksPending = 0U;
    __enable_irq();

    while (1) {
        if (take_control_tick()) {
            key_scan_5ms();
            if ((gDriveMode == DRIVE_BC_ARC) ||
                (gDriveMode == DRIVE_DA_ARC) ||
                ((gRouteMission == ROUTE_MISSION_TASK3) &&
                    route_is_active())) {
                imu_update_heading();
            }
            drive_tick_5ms();
            buzzer_service();
            point_light_service();
            oled_update_angle();
        } else {
            __WFI();
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        gMs += CONTROL_PERIOD_MS;
        if (gControlTicksPending < 2U) {
            gControlTicksPending++;
        }
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t gpioAStatus = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_A_PORT, ENCODER_A1_PIN | ENCODER_A2_PIN | IMU_INT_int_PIN);
    uint32_t encoderBStatus = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_B_PORT, ENCODER_B1_PIN | ENCODER_B2_PIN);

    if ((gpioAStatus & (ENCODER_A1_PIN | ENCODER_A2_PIN)) != 0U) {
        motor_control_handle_encoder_a_irq();
        uint8_t currentState =
            encoder_state(ENCODER_A_PORT, ENCODER_A1_PIN, ENCODER_A2_PIN);
        gEncoderA += quadrature_delta(gEncoderAPreviousState, currentState);
        gEncoderAPreviousState = currentState;
    }

    if ((encoderBStatus & (ENCODER_B1_PIN | ENCODER_B2_PIN)) != 0U) {
        motor_control_handle_encoder_b_irq();
        uint8_t currentState =
            encoder_state(ENCODER_B_PORT, ENCODER_B1_PIN, ENCODER_B2_PIN);
        gEncoderB += quadrature_delta(gEncoderBPreviousState, currentState);
        gEncoderBPreviousState = currentState;
    }

    if ((gpioAStatus & IMU_INT_int_PIN) != 0U) {
        gImuDataReady = true;
    }

    DL_GPIO_clearInterruptStatus(ENCODER_A_PORT,
        ENCODER_A1_PIN | ENCODER_A2_PIN | IMU_INT_int_PIN);
    DL_GPIO_clearInterruptStatus(ENCODER_B_PORT, ENCODER_B1_PIN | ENCODER_B2_PIN);
}
