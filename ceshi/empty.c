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

#include <stdbool.h>

#include "ti_msp_dl_config.h"

#define RUN_DUTY_PERCENT          (55U)
#define LINE_BLACK_ACTIVE_LOW     (0U)
#define START_IGNORE_DELAY_MS     (100U)
#define DEBOUNCE_DELAY_LOOPS      (800000U)
#define POLL_DELAY_LOOPS          (20000U)

/* SysConfig does not expose PA12/PA13/PA17 as GPIO choices in this project. */
#define LINE_SENSOR_S1_PORT       (GPIOA)
#define LINE_SENSOR_S1_PIN        (DL_GPIO_PIN_25)
#define LINE_SENSOR_S1_IOMUX      (IOMUX_PINCM55)
#define LINE_SENSOR_S2_PORT       (GPIOA)
#define LINE_SENSOR_S2_PIN        (DL_GPIO_PIN_17)
#define LINE_SENSOR_S2_IOMUX      (IOMUX_PINCM39)
#define LINE_SENSOR_S3_PORT       (GPIOA)
#define LINE_SENSOR_S3_PIN        (DL_GPIO_PIN_13)
#define LINE_SENSOR_S3_IOMUX      (IOMUX_PINCM35)
#define LINE_SENSOR_S4_PORT       (GPIOA)
#define LINE_SENSOR_S4_PIN        (DL_GPIO_PIN_12)
#define LINE_SENSOR_S4_IOMUX      (IOMUX_PINCM34)

static void delayLoops(uint32_t loops)
{
    while (loops--) {
        __NOP();
    }
}

static void delayMs(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}

static uint32_t dutyPercentToCompareValue(GPTIMER_Regs *timer, uint32_t dutyPercent)
{
    uint32_t loadValue = DL_Timer_getLoadValue(timer);

    if (dutyPercent >= 100U) {
        return 0U;
    }

    return loadValue - ((loadValue * dutyPercent) / 100U);
}

static void setMotorADuty(uint32_t dutyPercent)
{
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_A_INST,
        dutyPercentToCompareValue(PWM_MOTOR_A_INST, dutyPercent),
        GPIO_PWM_MOTOR_A_C1_IDX);
}

static void setMotorBDuty(uint32_t dutyPercent)
{
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_B_INST,
        dutyPercentToCompareValue(PWM_MOTOR_B_INST, dutyPercent),
        GPIO_PWM_MOTOR_B_C0_IDX);
}

static void setMotorForward(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN2_PIN);

    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN2_PIN);
}

static void initLineSensorPins(void)
{
    DL_GPIO_initDigitalInputFeatures(LINE_SENSOR_S1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE_SENSOR_S2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE_SENSOR_S3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE_SENSOR_S4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void startMotors(void)
{
    setMotorForward();
    setMotorADuty(RUN_DUTY_PERCENT);
    setMotorBDuty(RUN_DUTY_PERCENT);
}

static void stopMotors(void)
{
    setMotorADuty(0U);
    setMotorBDuty(0U);

    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT,
        GPIO_MOTOR_DIR_AIN1_PIN | GPIO_MOTOR_DIR_AIN2_PIN |
            GPIO_MOTOR_DIR_BIN1_PIN | GPIO_MOTOR_DIR_BIN2_PIN);
}

static bool isStartButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_START_BUTTON_PORT, GPIO_START_BUTTON_S2_PIN) ==
            0U);
}

static bool isSensorOnBlack(GPIO_Regs *port, uint32_t pin)
{
    bool levelHigh = (DL_GPIO_readPins(port, pin) != 0U);

#if LINE_BLACK_ACTIVE_LOW
    return !levelHigh;
#else
    return levelHigh;
#endif
}

static bool isAnyLineSensorOnBlack(void)
{
    return isSensorOnBlack(LINE_SENSOR_S1_PORT, LINE_SENSOR_S1_PIN) ||
           isSensorOnBlack(LINE_SENSOR_S2_PORT, LINE_SENSOR_S2_PIN) ||
           isSensorOnBlack(LINE_SENSOR_S3_PORT, LINE_SENSOR_S3_PIN) ||
           isSensorOnBlack(LINE_SENSOR_S4_PORT, LINE_SENSOR_S4_PIN);
}

static void waitForStartButtonPress(void)
{
    while (!isStartButtonPressed()) {
        delayLoops(POLL_DELAY_LOOPS);
    }

    delayLoops(DEBOUNCE_DELAY_LOOPS);
}

static void waitForStartButtonRelease(void)
{
    while (isStartButtonPressed()) {
        delayLoops(POLL_DELAY_LOOPS);
    }
}

int main(void)
{
    SYSCFG_DL_init();
    initLineSensorPins();

    stopMotors();
    DL_Timer_startCounter(PWM_MOTOR_A_INST);
    DL_Timer_startCounter(PWM_MOTOR_B_INST);

    while (1) {
        waitForStartButtonPress();
        startMotors();

        delayMs(START_IGNORE_DELAY_MS);

        while (!isAnyLineSensorOnBlack()) {
            delayLoops(POLL_DELAY_LOOPS);
        }

        stopMotors();
        waitForStartButtonRelease();
    }
}
