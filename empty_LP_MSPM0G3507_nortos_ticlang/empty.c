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

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_PERIOD_MS               (5U)
#define KEY_DEBOUNCE_MS                 (40U)
#define KEY_LONG_PRESS_MS               (1000U)

#define MOTOR_PWM_PERIOD                (3000)
#define MOTOR_PWM_CRUISE                (2300)

/*
 * Fixed C07A + S28A grayscale binding from hardware.md:
 *   AD0 -> PA22, AD1 -> PA8, AD2 -> PA12, OUT -> PA27.
 * Power the module from 5 V and share GND with the controller. OUT is a
 * digital signal whose project-confirmed maximum level is 3.3 V.
 */
#define LINE_SENSOR_CHANNEL_COUNT       (8U)
#define LINE_ADDR0_PORT                 LINE_SENSOR_PORT
#define LINE_ADDR0_PIN                  LINE_SENSOR_AD0_PIN
#define LINE_ADDR1_PORT                 LINE_SENSOR_PORT
#define LINE_ADDR1_PIN                  LINE_SENSOR_AD1_PIN
#define LINE_ADDR2_PORT                 LINE_SENSOR_PORT
#define LINE_ADDR2_PIN                  LINE_SENSOR_AD2_PIN
#define LINE_OUT_PORT                   LINE_SENSOR_PORT
#define LINE_OUT_PIN                    LINE_SENSOR_OUT_PIN

/* Calibrated on the installed sensor: black line outputs high level. */
#define LINE_BLACK_LEVEL                (1U)
#define LINE_REVERSE_ORDER              (0U)
/* Initial bring-up value; calibrate it on the installed sensor and track. */
#define LINE_MUX_SETTLE_US              (100U)
#define LINE_MUX_SETTLE_CYCLES          \
    ((CPUCLK_FREQ / 1000000U) * LINE_MUX_SETTLE_US)
#define LINE_PWM_GAIN_NUM               (3)
#define LINE_PWM_GAIN_DEN               (2)
#define LINE_CURVE_SLOWDOWN_NUM          (1)
#define LINE_CURVE_SLOWDOWN_DEN          (1)
/* Calibrated on the installed chassis: Motor A is right, Motor B is left. */
#define LINE_MOTOR_A_CORRECTION_SIGN     (-1)
#define LINE_MOTOR_B_CORRECTION_SIGN     (1)

#define MOTOR_A_PORT                    AIN_PORT
#define MOTOR_AIN1_PIN                  AIN_AIN1_PIN
#define MOTOR_AIN2_PIN                  AIN_AIN2_PIN
#define MOTOR_B_PORT                    BIN_PORT
#define MOTOR_BIN1_PIN                  BIN_BIN1_PIN
#define MOTOR_BIN2_PIN                  BIN_BIN2_PIN

#define ENCODER_A_PORT                  ENCODERA_PORT
#define ENCODER_A1_PIN                  ENCODERA_E1A_PIN
#define ENCODER_A2_PIN                  ENCODERA_E1B_PIN
#define ENCODER_B_PORT                  ENCODERB_PORT
#define ENCODER_B1_PIN                  ENCODERB_E2A_PIN
#define ENCODER_B2_PIN                  ENCODERB_E2B_PIN

#define USER_KEY_PORT                   KEY_PORT
#define USER_KEY_PIN                    KEY_key_PIN
#define USER_LED_PORT                   LED_PORT
#define USER_LED_PIN                    LED_led_PIN

typedef enum {
    DRIVE_IDLE = 0,
    DRIVE_LINE_FOLLOW
} DriveMode;

static volatile uint32_t gMs;
static volatile int32_t gEncoderA;
static volatile int32_t gEncoderB;

static DriveMode gDriveMode = DRIVE_IDLE;
static int16_t gLastLineError;

static int16_t clamp_pwm(int16_t pwm)
{
    if (pwm > MOTOR_PWM_PERIOD) {
        return MOTOR_PWM_PERIOD;
    }
    if (pwm < -MOTOR_PWM_PERIOD) {
        return -MOTOR_PWM_PERIOD;
    }
    return pwm;
}

static uint16_t abs_pwm(int16_t pwm)
{
    return (uint16_t) ((pwm < 0) ? -pwm : pwm);
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

static void indicator_on(void)
{
    DL_GPIO_clearPins(USER_LED_PORT, USER_LED_PIN);
}

static void indicator_off(void)
{
    DL_GPIO_setPins(USER_LED_PORT, USER_LED_PIN);
}

static void set_motor_pwm(int16_t motorAPwm, int16_t motorBPwm)
{
    motorAPwm = clamp_pwm(motorAPwm);
    motorBPwm = clamp_pwm(motorBPwm);

    /* Calibrated on the installed chassis: positive PWM means forward. */
    if (motorAPwm > 0) {
        DL_GPIO_setPins(MOTOR_A_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_A_PORT, MOTOR_AIN2_PIN);
    } else if (motorAPwm < 0) {
        DL_GPIO_setPins(MOTOR_A_PORT, MOTOR_AIN2_PIN);
        DL_GPIO_clearPins(MOTOR_A_PORT, MOTOR_AIN1_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_A_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    }

    if (motorBPwm > 0) {
        DL_GPIO_setPins(MOTOR_B_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_BIN2_PIN);
    } else if (motorBPwm < 0) {
        DL_GPIO_setPins(MOTOR_B_PORT, MOTOR_BIN2_PIN);
        DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_BIN1_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_B_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    }

    DL_Timer_setCaptureCompareValue(
        PWM_0_INST, abs_pwm(motorAPwm), GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(
        PWM_0_INST, abs_pwm(motorBPwm), GPIO_PWM_0_C1_IDX);
}

static void stop_car(void)
{
    set_motor_pwm(0, 0);
}

static void stop_line_follow(void)
{
    gDriveMode = DRIVE_IDLE;
    gLastLineError = 0;
    stop_car();
    indicator_off();
}

static void start_line_follow(void)
{
    gLastLineError = 0;
    gDriveMode = DRIVE_LINE_FOLLOW;
    indicator_on();
}

static void toggle_line_follow(void)
{
    if (gDriveMode == DRIVE_LINE_FOLLOW) {
        stop_line_follow();
    } else {
        start_line_follow();
    }
}

static void drive_line_follow_5ms(void)
{
    int16_t error = line_error_from_mask(line_sensor_read_mask());
    int32_t pwmCorrection =
        ((int32_t) error * LINE_PWM_GAIN_NUM) / LINE_PWM_GAIN_DEN;
    int32_t correctionMagnitude =
        (pwmCorrection < 0) ? -pwmCorrection : pwmCorrection;
    int32_t basePwm = (int32_t) MOTOR_PWM_CRUISE -
        ((correctionMagnitude * LINE_CURVE_SLOWDOWN_NUM) /
            LINE_CURVE_SLOWDOWN_DEN);

    if (basePwm < 0) {
        basePwm = 0;
    }

    set_motor_pwm(
        (int16_t) (basePwm +
            (pwmCorrection * LINE_MOTOR_A_CORRECTION_SIGN)),
        (int16_t) (basePwm +
            (pwmCorrection * LINE_MOTOR_B_CORRECTION_SIGN)));
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
    bool rawPressed = key_pressed_raw();

    if (rawPressed == rawPrev) {
        if (stableTicks < (KEY_DEBOUNCE_MS / CONTROL_PERIOD_MS)) {
            stableTicks++;
        }
    } else {
        rawPrev = rawPressed;
        stableTicks = 0;
    }

    if ((stableTicks == (KEY_DEBOUNCE_MS / CONTROL_PERIOD_MS)) &&
        (rawPressed != debouncedPressed)) {
        debouncedPressed = rawPressed;
        if (debouncedPressed) {
            pressStartMs = gMs;
            longPressHandled = false;
        } else {
            if (!longPressHandled) {
                toggle_line_follow();
            }
        }
    }

    if (debouncedPressed && !longPressHandled &&
        ((gMs - pressStartMs) >= KEY_LONG_PRESS_MS)) {
        longPressHandled = true;
        stop_line_follow();
    }
}

static void drive_tick_5ms(void)
{
    switch (gDriveMode) {
    case DRIVE_LINE_FOLLOW:
        drive_line_follow_5ms();
        break;

    case DRIVE_IDLE:
    default:
        stop_car();
        break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    line_sensor_init();

    indicator_off();
    stop_car();

    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(TIMER_0_INST);

    while (1) {
        __WFI();
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        gMs += CONTROL_PERIOD_MS;
        key_scan_5ms();
        drive_tick_5ms();
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t encoderAStatus = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_A_PORT, ENCODER_A1_PIN | ENCODER_A2_PIN);
    uint32_t encoderBStatus = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_B_PORT, ENCODER_B1_PIN | ENCODER_B2_PIN);

    if ((encoderAStatus & ENCODER_A1_PIN) != 0U) {
        if ((DL_GPIO_readPins(ENCODER_A_PORT, ENCODER_A2_PIN) & ENCODER_A2_PIN) == 0U) {
            gEncoderA--;
        } else {
            gEncoderA++;
        }
    } else if ((encoderAStatus & ENCODER_A2_PIN) != 0U) {
        if ((DL_GPIO_readPins(ENCODER_A_PORT, ENCODER_A1_PIN) & ENCODER_A1_PIN) == 0U) {
            gEncoderA++;
        } else {
            gEncoderA--;
        }
    }

    if ((encoderBStatus & ENCODER_B1_PIN) != 0U) {
        if ((DL_GPIO_readPins(ENCODER_B_PORT, ENCODER_B2_PIN) & ENCODER_B2_PIN) == 0U) {
            gEncoderB--;
        } else {
            gEncoderB++;
        }
    } else if ((encoderBStatus & ENCODER_B2_PIN) != 0U) {
        if ((DL_GPIO_readPins(ENCODER_B_PORT, ENCODER_B1_PIN) & ENCODER_B1_PIN) == 0U) {
            gEncoderB++;
        } else {
            gEncoderB--;
        }
    }

    DL_GPIO_clearInterruptStatus(ENCODER_A_PORT, ENCODER_A1_PIN | ENCODER_A2_PIN);
    DL_GPIO_clearInterruptStatus(ENCODER_B_PORT, ENCODER_B1_PIN | ENCODER_B2_PIN);
}
