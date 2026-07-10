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
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define PI_F                              (3.1415926f)
#define Q1_TARGET_DISTANCE_MM             (1000.0f)
#define Q2_ARC_RADIUS_MM                  (400.0f)
#define Q2_STRAIGHT_AB_DISTANCE_MM        (1000.0f)
#define Q2_ARC_BC_DISTANCE_MM             (PI_F * Q2_ARC_RADIUS_MM)
#define Q2_STRAIGHT_CD_DISTANCE_MM        (1000.0f)
#define Q2_ARC_DA_DISTANCE_MM             (PI_F * Q2_ARC_RADIUS_MM)
#define Q2_TARGET_DISTANCE_MM             (Q2_STRAIGHT_AB_DISTANCE_MM + \
                                           Q2_ARC_BC_DISTANCE_MM +      \
                                           Q2_STRAIGHT_CD_DISTANCE_MM + \
                                           Q2_ARC_DA_DISTANCE_MM)
#define WHEEL_DIAMETER_MM                 (65.0f)
#define ENCODER_COUNTS_PER_WHEEL_REV      (390.0f)
#define RIGHT_ENCODER_ENABLED             (1U)

#define QUESTION_1_MODE                   (1U)
#define QUESTION_2_MODE                   (2U)
#define MODE_SELECT_LONG_PRESS_MS         (1000U)

#define BASE_DUTY_PERCENT                 (48U)
#define Q2_BASE_DUTY_PERCENT              (42U)
#define LEFT_MOTOR_TRIM_PERCENT           (0)
#define RIGHT_MOTOR_TRIM_PERCENT          (0)
#define MIN_DUTY_PERCENT                  (25U)
#define MAX_DUTY_PERCENT                  (75U)
#define CONTROL_PERIOD_MS                 (10U)
#define CONTROL_PERIOD_SEC                (0.010f)

#define LINE_AUTO_CALIBRATE_ON_START      (1U)
#define LINE_BLACK_ACTIVE_LOW             (0U)
#define LINE_FOLLOW_ENABLE                (0U)
#define LINE_KP                           (5.0f)
#define Q2_LINE_KP                        (7.0f)
#define Q2_LINE_KD                        (3.0f)
#define Q2_ENCODER_BALANCE_KP             (0.035f)
#define Q2_MAX_STEER_PERCENT              (30)
#define Q2_LOST_LINE_MAX_COUNT            (120U)
#define Q2_POINT_SIGNAL_TICKS             (18U)
#define Q2_POINT_SIGNAL_BLINK_TICKS       (4U)
#define Q2_MAX_RUNTIME_MS                 (30000U)
#define MPU_YAW_CORRECTION_ENABLE         (1U)
#define MPU_GYRO_Z_SIGN                   (1.0f)
#define YAW_KP                            (0.8f)
#define YAW_DEADBAND_DEG                  (0.6f)
#define YAW_MAX_STEER_PERCENT             (8.0f)
#define ENCODER_BALANCE_KP                (0.120f)
#define MAX_STEER_PERCENT                 (24)

#define STOP_LINE_MIN_BLACK_SENSOR_COUNT  (1U)
#define LINE_FOUND_BLINK_COUNT            (6U)
#define TARGET_REACHED_BLINK_COUNT        (3U)
#define BRAKE_TIME_MS                     (80U)
#define DEBOUNCE_DELAY_LOOPS              (800000U)
#define POLL_DELAY_LOOPS                  (20000U)

#define MPU6050_ADDR                      (0x68U)
#define MPU6050_WHO_AM_I                  (0x75U)
#define MPU6050_PWR_MGMT_1                (0x6BU)
#define MPU6050_CONFIG                    (0x1AU)
#define MPU6050_GYRO_CONFIG               (0x1BU)
#define MPU6050_GYRO_ZOUT_H               (0x47U)
#define MPU6050_WHO_AM_I_VALUE            (0x68U)
#define MPU6050_GYRO_LSB_PER_DPS          (65.5f)
#define MPU6050_CALIBRATION_SAMPLES       (120U)
#define I2C_TIMEOUT_LOOPS                 (50000U)

#define HMI_SOUND_ENABLE                  (1U)
#define HMI_TX_BYTE_DELAY_CYCLES          (3200U)

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

static int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t clampInt32(int32_t value, int32_t minValue, int32_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
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

static void setLeftMotorDuty(uint32_t dutyPercent)
{
    setMotorADuty(dutyPercent);
}

static void setRightMotorDuty(uint32_t dutyPercent)
{
    setMotorBDuty(dutyPercent);
}

static void setDriveDuty(int32_t leftDuty, int32_t rightDuty)
{
    leftDuty = clampInt32(leftDuty, MIN_DUTY_PERCENT, MAX_DUTY_PERCENT);
    rightDuty = clampInt32(rightDuty, MIN_DUTY_PERCENT, MAX_DUTY_PERCENT);

    setLeftMotorDuty((uint32_t) leftDuty);
    setRightMotorDuty((uint32_t) rightDuty);
}

static void setMotorForward(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN2_PIN);

    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN2_PIN);
}

static void setMotorBrake(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN2_PIN);

    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN2_PIN);
}

static void setMotorCoast(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_AIN2_PIN);

    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_PORT, GPIO_MOTOR_DIR_BIN2_PIN);
}

static void stopMotors(void)
{
    setLeftMotorDuty(0U);
    setRightMotorDuty(0U);

    setMotorBrake();
    delayMs(BRAKE_TIME_MS);
    setMotorCoast();
}

static void setStatusLed(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_STATUS_LED_PORT, GPIO_STATUS_LED_BLUE_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_STATUS_LED_PORT, GPIO_STATUS_LED_BLUE_PIN);
    }
}

static void blinkStatusLed(uint32_t count)
{
    while (count--) {
        setStatusLed(true);
        delayMs(120U);
        setStatusLed(false);
        delayMs(120U);
    }
}

#if HMI_SOUND_ENABLE
static void hmiTransmitByte(uint8_t data)
{
    DL_UART_Main_transmitData(UART_HMI_INST, data);
    delay_cycles(HMI_TX_BYTE_DELAY_CYCLES);
}

static void hmiTransmitCommand(const char *command)
{
    while (*command != '\0') {
        hmiTransmitByte((uint8_t) *command);
        command++;
    }

    hmiTransmitByte(0xFFU);
    hmiTransmitByte(0xFFU);
    hmiTransmitByte(0xFFU);
}

static void hmiBeepShort(void)
{
    hmiTransmitCommand("beep 100");
}
#else
static void hmiBeepShort(void)
{
}
#endif

static bool isStartButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_START_BUTTON_PORT, GPIO_START_BUTTON_S2_PIN) ==
            0U);
}

static bool gLineS1WhiteLevelHigh = false;
static bool gLineS2WhiteLevelHigh = false;
static bool gLineS3WhiteLevelHigh = false;
static bool gLineS4WhiteLevelHigh = false;

static bool readSensorLevelHigh(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U);
}

static bool isSensorOnBlack(GPIO_Regs *port, uint32_t pin, bool whiteLevelHigh)
{
    bool levelHigh = readSensorLevelHigh(port, pin);

#if LINE_AUTO_CALIBRATE_ON_START
    return (levelHigh != whiteLevelHigh);
#else
    (void) whiteLevelHigh;
#if LINE_BLACK_ACTIVE_LOW
    return !levelHigh;
#else
    return levelHigh;
#endif
#endif
}

static void calibrateLineSensorsOnWhite(void)
{
    bool s1LevelHigh = readSensorLevelHigh(
        GPIO_LINE_SENSOR_S1_RIGHT_PORT, GPIO_LINE_SENSOR_S1_RIGHT_PIN);
    bool s2LevelHigh = readSensorLevelHigh(GPIO_LINE_SENSOR_S2_MID_RIGHT_PORT,
        GPIO_LINE_SENSOR_S2_MID_RIGHT_PIN);
    bool s3LevelHigh = readSensorLevelHigh(
        GPIO_LINE_SENSOR_S3_MID_LEFT_PORT, GPIO_LINE_SENSOR_S3_MID_LEFT_PIN);
    bool s4LevelHigh = readSensorLevelHigh(
        GPIO_LINE_SENSOR_S4_LEFT_PORT, GPIO_LINE_SENSOR_S4_LEFT_PIN);

    if ((s1LevelHigh == s4LevelHigh) && (s2LevelHigh == s3LevelHigh) &&
        (s1LevelHigh != s2LevelHigh)) {
        gLineS1WhiteLevelHigh = s1LevelHigh;
        gLineS2WhiteLevelHigh = s1LevelHigh;
        gLineS3WhiteLevelHigh = s1LevelHigh;
        gLineS4WhiteLevelHigh = s1LevelHigh;
        return;
    }

    gLineS1WhiteLevelHigh = s1LevelHigh;
    gLineS2WhiteLevelHigh = s2LevelHigh;
    gLineS3WhiteLevelHigh = s3LevelHigh;
    gLineS4WhiteLevelHigh = s4LevelHigh;
}

static void calibrateLineSensorsForTrackStart(void)
{
    bool s1LevelHigh = readSensorLevelHigh(
        GPIO_LINE_SENSOR_S1_RIGHT_PORT, GPIO_LINE_SENSOR_S1_RIGHT_PIN);
    bool s2LevelHigh = readSensorLevelHigh(GPIO_LINE_SENSOR_S2_MID_RIGHT_PORT,
        GPIO_LINE_SENSOR_S2_MID_RIGHT_PIN);
    bool s3LevelHigh = readSensorLevelHigh(
        GPIO_LINE_SENSOR_S3_MID_LEFT_PORT, GPIO_LINE_SENSOR_S3_MID_LEFT_PIN);
    bool s4LevelHigh = readSensorLevelHigh(
        GPIO_LINE_SENSOR_S4_LEFT_PORT, GPIO_LINE_SENSOR_S4_LEFT_PIN);

    if ((s1LevelHigh == s2LevelHigh) && (s2LevelHigh == s3LevelHigh) &&
        (s3LevelHigh == s4LevelHigh)) {
        gLineS1WhiteLevelHigh = !s1LevelHigh;
        gLineS2WhiteLevelHigh = !s2LevelHigh;
        gLineS3WhiteLevelHigh = !s3LevelHigh;
        gLineS4WhiteLevelHigh = !s4LevelHigh;
        return;
    }

    if ((s1LevelHigh == s4LevelHigh) && (s2LevelHigh == s3LevelHigh) &&
        (s1LevelHigh != s2LevelHigh)) {
        gLineS1WhiteLevelHigh = s1LevelHigh;
        gLineS2WhiteLevelHigh = s1LevelHigh;
        gLineS3WhiteLevelHigh = s1LevelHigh;
        gLineS4WhiteLevelHigh = s1LevelHigh;
        return;
    }

    gLineS1WhiteLevelHigh = s1LevelHigh;
    gLineS2WhiteLevelHigh = s2LevelHigh;
    gLineS3WhiteLevelHigh = s3LevelHigh;
    gLineS4WhiteLevelHigh = s4LevelHigh;
}

static bool isLineS1Black(void)
{
    return isSensorOnBlack(
        GPIO_LINE_SENSOR_S1_RIGHT_PORT, GPIO_LINE_SENSOR_S1_RIGHT_PIN,
        gLineS1WhiteLevelHigh);
}

static bool isLineS2Black(void)
{
    return isSensorOnBlack(GPIO_LINE_SENSOR_S2_MID_RIGHT_PORT,
        GPIO_LINE_SENSOR_S2_MID_RIGHT_PIN, gLineS2WhiteLevelHigh);
}

static bool isLineS3Black(void)
{
    return isSensorOnBlack(
        GPIO_LINE_SENSOR_S3_MID_LEFT_PORT, GPIO_LINE_SENSOR_S3_MID_LEFT_PIN,
        gLineS3WhiteLevelHigh);
}

static bool isLineS4Black(void)
{
    return isSensorOnBlack(
        GPIO_LINE_SENSOR_S4_LEFT_PORT, GPIO_LINE_SENSOR_S4_LEFT_PIN,
        gLineS4WhiteLevelHigh);
}

static uint32_t getBlackLineSensorCount(void)
{
    uint32_t blackCount = 0U;

    if (isLineS1Black()) {
        blackCount++;
    }
    if (isLineS2Black()) {
        blackCount++;
    }
    if (isLineS3Black()) {
        blackCount++;
    }
    if (isLineS4Black()) {
        blackCount++;
    }

    return blackCount;
}

static int32_t getLineError(void)
{
    int32_t weightedSum = 0;
    int32_t activeCount = 0;

    if (isLineS4Black()) {
        weightedSum -= 3;
        activeCount++;
    }
    if (isLineS3Black()) {
        weightedSum -= 1;
        activeCount++;
    }
    if (isLineS2Black()) {
        weightedSum += 1;
        activeCount++;
    }
    if (isLineS1Black()) {
        weightedSum += 3;
        activeCount++;
    }

    if (activeCount == 0) {
        return 0;
    }

    return weightedSum / activeCount;
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

static uint32_t waitForRunModeSelect(void)
{
    uint32_t heldMs = 0U;

    waitForStartButtonPress();

    while (isStartButtonPressed()) {
        delayMs(10U);
        heldMs += 10U;

        if (heldMs >= MODE_SELECT_LONG_PRESS_MS) {
            setStatusLed(true);
        }
    }

    waitForStartButtonRelease();
    delayLoops(DEBOUNCE_DELAY_LOOPS);
    setStatusLed(false);

    return (heldMs >= MODE_SELECT_LONG_PRESS_MS) ? QUESTION_2_MODE :
                                                   QUESTION_1_MODE;
}

static volatile int32_t gLeftEncoderCount = 0;
static volatile int32_t gRightEncoderCount = 0;
static float gGyroZBiasDps = 0.0f;
static float gYawDeg = 0.0f;
static bool gMpuReady = false;

static void resetEncoders(void)
{
    __disable_irq();
    gLeftEncoderCount = 0;
    gRightEncoderCount = 0;
    __enable_irq();
}

static int32_t getLeftEncoderCount(void)
{
    int32_t count;

    __disable_irq();
    count = gLeftEncoderCount;
    __enable_irq();

    return count;
}

#if RIGHT_ENCODER_ENABLED
static int32_t getRightEncoderCount(void)
{
    int32_t count;

    __disable_irq();
    count = gRightEncoderCount;
    __enable_irq();

    return count;
}
#endif

static float countsToDistanceMm(int32_t counts)
{
    return ((float) abs32(counts) * WHEEL_DIAMETER_MM * PI_F) /
           ENCODER_COUNTS_PER_WHEEL_REV;
}

static float getAverageDistanceMm(void)
{
#if RIGHT_ENCODER_ENABLED
    return (countsToDistanceMm(getLeftEncoderCount()) +
               countsToDistanceMm(getRightEncoderCount())) *
           0.5f;
#else
    return countsToDistanceMm(getLeftEncoderCount());
#endif
}

static bool i2cWaitIdle(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;

    while (!(DL_I2C_getControllerStatus(I2C_MPU_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

static bool i2cWaitNotBusy(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;
    uint32_t status;

    do {
        status = DL_I2C_getControllerStatus(I2C_MPU_INST);
        if (status & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        if (timeout-- == 0U) {
            return false;
        }
    } while (status & DL_I2C_CONTROLLER_STATUS_BUSY);

    return true;
}

static bool i2cWrite(uint8_t address, const uint8_t *data, uint16_t length)
{
    if (!i2cWaitIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU_INST);
    if (DL_I2C_fillControllerTXFIFO(I2C_MPU_INST, data, length) != length) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        I2C_MPU_INST, address, DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(100U);

    return i2cWaitNotBusy();
}

static bool i2cRead(uint8_t address, uint8_t *data, uint16_t length)
{
    if (!i2cWaitIdle()) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_MPU_INST);
    DL_I2C_startControllerTransfer(
        I2C_MPU_INST, address, DL_I2C_CONTROLLER_DIRECTION_RX, length);
    delay_cycles(100U);

    if (!i2cWaitNotBusy()) {
        return false;
    }

    for (uint16_t i = 0; i < length; i++) {
        uint32_t timeout = I2C_TIMEOUT_LOOPS;

        while (DL_I2C_isControllerRXFIFOEmpty(I2C_MPU_INST)) {
            if (timeout-- == 0U) {
                return false;
            }
        }
        data[i] = DL_I2C_receiveControllerData(I2C_MPU_INST);
    }

    return true;
}

static bool mpuWriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    return i2cWrite(MPU6050_ADDR, tx, sizeof(tx));
}

static bool mpuReadRegs(uint8_t reg, uint8_t *data, uint16_t length)
{
    if (!i2cWrite(MPU6050_ADDR, &reg, 1U)) {
        return false;
    }

    return i2cRead(MPU6050_ADDR, data, length);
}

static bool mpuReadGyroZRaw(int16_t *gyroZRaw)
{
    uint8_t rx[2];

    if (!mpuReadRegs(MPU6050_GYRO_ZOUT_H, rx, sizeof(rx))) {
        return false;
    }

    *gyroZRaw = (int16_t) (((uint16_t) rx[0] << 8) | rx[1]);
    return true;
}

static bool mpuInit(void)
{
    uint8_t whoAmI = 0U;

    if (!mpuReadRegs(MPU6050_WHO_AM_I, &whoAmI, 1U) ||
        (whoAmI != MPU6050_WHO_AM_I_VALUE)) {
        return false;
    }

    if (!mpuWriteReg(MPU6050_PWR_MGMT_1, 0x00U)) {
        return false;
    }
    delayMs(50U);

    return mpuWriteReg(MPU6050_CONFIG, 0x03U) &&
           mpuWriteReg(MPU6050_GYRO_CONFIG, 0x08U);
}

static bool calibrateGyroZ(void)
{
    int32_t sum = 0;
    uint32_t validSamples = 0;

    for (uint32_t i = 0; i < MPU6050_CALIBRATION_SAMPLES; i++) {
        int16_t raw;

        if (mpuReadGyroZRaw(&raw)) {
            sum += raw;
            validSamples++;
        }
        delayMs(3U);
    }

    if (validSamples < (MPU6050_CALIBRATION_SAMPLES / 2U)) {
        gGyroZBiasDps = 0.0f;
        return false;
    }

    gGyroZBiasDps =
        ((float) sum / (float) validSamples) / MPU6050_GYRO_LSB_PER_DPS;
    gYawDeg = 0.0f;

    return true;
}

static void updateYaw(void)
{
    int16_t raw;

    if (!gMpuReady || !mpuReadGyroZRaw(&raw)) {
        return;
    }

    float gyroZDps =
        (((float) raw / MPU6050_GYRO_LSB_PER_DPS) - gGyroZBiasDps) *
        MPU_GYRO_Z_SIGN;
    gYawDeg += gyroZDps * CONTROL_PERIOD_SEC;
}

static int32_t calculateSteer(float targetYawDeg)
{
#if RIGHT_ENCODER_ENABLED
    int32_t leftCount = abs32(getLeftEncoderCount());
    int32_t rightCount = abs32(getRightEncoderCount());
#endif
    float steer = 0.0f;

#if LINE_FOLLOW_ENABLE
    steer += LINE_KP * (float) getLineError();
#endif
#if MPU_YAW_CORRECTION_ENABLE
    if (gMpuReady) {
        float yawError = targetYawDeg - gYawDeg;
        float yawSteer;

        if ((yawError > -YAW_DEADBAND_DEG) &&
            (yawError < YAW_DEADBAND_DEG)) {
            yawError = 0.0f;
        }

        yawSteer = YAW_KP * yawError;
        if (yawSteer > YAW_MAX_STEER_PERCENT) {
            yawSteer = YAW_MAX_STEER_PERCENT;
        } else if (yawSteer < -YAW_MAX_STEER_PERCENT) {
            yawSteer = -YAW_MAX_STEER_PERCENT;
        }

        steer += yawSteer;
    }
#else
    (void) targetYawDeg;
#endif
#if RIGHT_ENCODER_ENABLED
    steer -= ENCODER_BALANCE_KP * (float) (leftCount - rightCount);
#endif

    return clampInt32((int32_t) steer, -MAX_STEER_PERCENT, MAX_STEER_PERCENT);
}

static int32_t calculateQuestion2Steer(int32_t lineError, int32_t lastLineError)
{
    float steer =
        (Q2_LINE_KP * (float) lineError) +
        (Q2_LINE_KD * (float) (lineError - lastLineError));

#if RIGHT_ENCODER_ENABLED
    int32_t leftCount = abs32(getLeftEncoderCount());
    int32_t rightCount = abs32(getRightEncoderCount());

    steer -= Q2_ENCODER_BALANCE_KP * (float) (leftCount - rightCount);
#endif

    return clampInt32(
        (int32_t) steer, -Q2_MAX_STEER_PERCENT, Q2_MAX_STEER_PERCENT);
}

static void startPointSignal(uint32_t *signalTicks)
{
    *signalTicks = Q2_POINT_SIGNAL_TICKS;
    hmiBeepShort();
}

static void servicePointSignal(uint32_t *signalTicks, bool lineDetected)
{
    if (*signalTicks > 0U) {
        if (((*signalTicks / Q2_POINT_SIGNAL_BLINK_TICKS) & 1U) == 0U) {
            setStatusLed(true);
        } else {
            setStatusLed(false);
        }
        (*signalTicks)--;
    } else {
        setStatusLed(lineDetected);
    }
}

static bool shouldTriggerQuestion2Point(float distanceMm, float pointDistanceMm)
{
    if (distanceMm >= pointDistanceMm) {
        return true;
    }

    return false;
}

static void runQuestion1StraightToB(void)
{
    float targetYawDeg = 0.0f;
    bool stopLineDetected = false;

    calibrateLineSensorsOnWhite();
#if MPU_YAW_CORRECTION_ENABLE
    if (gMpuReady) {
        setStatusLed(true);
        gMpuReady = calibrateGyroZ();
        setStatusLed(false);
    }
#endif
    resetEncoders();
    gYawDeg = 0.0f;
    setStatusLed(false);

    setMotorForward();
    setDriveDuty((int32_t) BASE_DUTY_PERCENT + LEFT_MOTOR_TRIM_PERCENT,
        (int32_t) BASE_DUTY_PERCENT + RIGHT_MOTOR_TRIM_PERCENT);

    while (getAverageDistanceMm() < Q1_TARGET_DISTANCE_MM) {
        int32_t steer;
        uint32_t blackLineSensorCount;

        updateYaw();
        blackLineSensorCount = getBlackLineSensorCount();

        setStatusLed(blackLineSensorCount > 0U);

        if (blackLineSensorCount >= STOP_LINE_MIN_BLACK_SENSOR_COUNT) {
            stopLineDetected = true;
            break;
        }

        steer = calculateSteer(targetYawDeg);
        setDriveDuty(
            (int32_t) BASE_DUTY_PERCENT + LEFT_MOTOR_TRIM_PERCENT + steer,
            (int32_t) BASE_DUTY_PERCENT + RIGHT_MOTOR_TRIM_PERCENT - steer);

        delayMs(CONTROL_PERIOD_MS);
    }

    stopMotors();
    setStatusLed(false);
    blinkStatusLed(
        stopLineDetected ? LINE_FOUND_BLINK_COUNT : TARGET_REACHED_BLINK_COUNT);
}

static void runQuestion2LineFollow(void)
{
    static const float question2PointDistancesMm[4] = {
        Q2_STRAIGHT_AB_DISTANCE_MM,
        Q2_STRAIGHT_AB_DISTANCE_MM + Q2_ARC_BC_DISTANCE_MM,
        Q2_STRAIGHT_AB_DISTANCE_MM + Q2_ARC_BC_DISTANCE_MM +
            Q2_STRAIGHT_CD_DISTANCE_MM,
        Q2_TARGET_DISTANCE_MM,
    };
    int32_t lastLineError = 0;
    uint32_t lostLineCount = 0U;
    uint32_t pointIndex = 0U;
    uint32_t signalTicks = 0U;
    uint32_t elapsedMs = 0U;
    bool finishDetected = false;

    calibrateLineSensorsForTrackStart();
#if MPU_YAW_CORRECTION_ENABLE
    if (gMpuReady) {
        setStatusLed(true);
        gMpuReady = calibrateGyroZ();
        setStatusLed(false);
    }
#endif
    resetEncoders();
    gYawDeg = 0.0f;
    setStatusLed(false);

    setMotorForward();
    setDriveDuty((int32_t) Q2_BASE_DUTY_PERCENT + LEFT_MOTOR_TRIM_PERCENT,
        (int32_t) Q2_BASE_DUTY_PERCENT + RIGHT_MOTOR_TRIM_PERCENT);

    while (elapsedMs < Q2_MAX_RUNTIME_MS) {
        float distanceMm = getAverageDistanceMm();
        uint32_t blackLineSensorCount = getBlackLineSensorCount();
        int32_t previousLineError = lastLineError;
        int32_t lineError = getLineError();
        int32_t steer;
        bool lineDetected = (blackLineSensorCount > 0U);

        updateYaw();

        if ((pointIndex < 4U) &&
            shouldTriggerQuestion2Point(distanceMm,
                question2PointDistancesMm[pointIndex])) {
            startPointSignal(&signalTicks);
            pointIndex++;

            if (pointIndex >= 4U) {
                finishDetected = true;
                break;
            }
        }

        if (distanceMm >= Q2_TARGET_DISTANCE_MM) {
            finishDetected = true;
            startPointSignal(&signalTicks);
            break;
        }

        if (blackLineSensorCount == 0U) {
            lostLineCount++;
            lineError = lastLineError;
            if (lostLineCount >= Q2_LOST_LINE_MAX_COUNT) {
                break;
            }
        } else {
            lostLineCount = 0U;
        }

        steer = calculateQuestion2Steer(lineError, previousLineError);
        if (blackLineSensorCount > 0U) {
            lastLineError = lineError;
        }
        setDriveDuty(
            (int32_t) Q2_BASE_DUTY_PERCENT + LEFT_MOTOR_TRIM_PERCENT + steer,
            (int32_t) Q2_BASE_DUTY_PERCENT + RIGHT_MOTOR_TRIM_PERCENT -
                steer);

        servicePointSignal(&signalTicks, lineDetected);
        delayMs(CONTROL_PERIOD_MS);
        elapsedMs += CONTROL_PERIOD_MS;
    }

    stopMotors();
    setStatusLed(false);
    blinkStatusLed(
        finishDetected ? LINE_FOUND_BLINK_COUNT : TARGET_REACHED_BLINK_COUNT);
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT,
        GPIO_ENCODER_LEFT_A_PIN
#if RIGHT_ENCODER_ENABLED
            | GPIO_ENCODER_RIGHT_A_PIN
#endif
    );

    if (pending & GPIO_ENCODER_LEFT_A_PIN) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_LEFT_B_PIN)) {
            gLeftEncoderCount++;
        } else {
            gLeftEncoderCount--;
        }
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_LEFT_A_PIN);
    }

#if RIGHT_ENCODER_ENABLED
    if (pending & GPIO_ENCODER_RIGHT_A_PIN) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_RIGHT_B_PIN)) {
            gRightEncoderCount++;
        } else {
            gRightEncoderCount--;
        }
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_RIGHT_A_PIN);
    }
#endif
}

int main(void)
{
    SYSCFG_DL_init();

    stopMotors();
    DL_Timer_startCounter(PWM_MOTOR_A_INST);
    DL_Timer_startCounter(PWM_MOTOR_B_INST);
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);

    gMpuReady = mpuInit();
    if (gMpuReady) {
        gMpuReady = calibrateGyroZ();
    }

    while (1) {
        uint32_t runMode = waitForRunModeSelect();

        if (runMode == QUESTION_2_MODE) {
            blinkStatusLed(2U);
            runQuestion2LineFollow();
        } else {
            blinkStatusLed(1U);
            runQuestion1StraightToB();
        }
    }
}
