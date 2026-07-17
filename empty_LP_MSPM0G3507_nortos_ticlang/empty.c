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

#define CONTROL_PERIOD_MS                   (5U)
#define KEY_DEBOUNCE_MS                     (40U)
#define KEY_LONG_PRESS_MS                   (1000U)
#define ROUTE_TIMEOUT_MS                    (29500U)

#define MOTOR_PWM_PERIOD                    (4000)
#define MOTOR_PWM_CRUISE                    (2300)
#define STRAIGHT_BALANCE_DIVISOR            (3)
#define STRAIGHT_BALANCE_MAX                (500)
#define STRAIGHT_HEADING_KP_DIVISOR         (40)
#define STRAIGHT_HEADING_CORRECTION_MAX     (600)

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
#define ARC_TARGET_COUNTS                    (9600U)
#define ARC_MAX_COUNTS                       (11000U)
#define ARC_ACQUIRE_LIMIT_COUNTS             (1800U)
#define ARC_HEADING_CHANGE_MDEG              (180000L)
#define ARC_HEADING_KP_DIVISOR               (80)
#define ARC_HEADING_CORRECTION_MAX           (700)

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
    DRIVE_COMPLETE,
    DRIVE_FAULT
} DriveMode;

static volatile uint32_t gMs;
static volatile int32_t gEncoderA;
static volatile int32_t gEncoderB;
static volatile uint8_t gControlTicksPending;
static volatile bool gImuDataReady;

static DriveMode gDriveMode = DRIVE_IDLE;
static int16_t gLastLineError;
static uint8_t gEncoderAPreviousState;
static uint8_t gEncoderBPreviousState;
static bool gStraightHasSeenWhite;
static bool gArcLineAcquired;
static uint32_t gRouteStartMs;
static int32_t gSegmentHeadingTargetMdeg;
static int32_t gArcStartHeadingMdeg;

static bool gImuReady;
static int32_t gGyroZBiasRaw;
static int32_t gHeadingMdeg;
static int32_t gHeadingRemainder;
static uint32_t gImuLastSampleMs;

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

static void wait_ms(uint32_t durationMs)
{
    uint32_t startMs = gMs;

    while (!elapsed_ms(startMs, durationMs)) {
        __WFI();
    }
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

static uint32_t segment_average_counts(void)
{
    uint32_t motorACounts = abs_encoder_count(gEncoderA);
    uint32_t motorBCounts = abs_encoder_count(gEncoderB);

    return (motorACounts + motorBCounts) / 2U;
}

static bool route_is_active(void)
{
    return (gDriveMode == DRIVE_AB_STRAIGHT) ||
        (gDriveMode == DRIVE_BC_ARC) ||
        (gDriveMode == DRIVE_CD_STRAIGHT) ||
        (gDriveMode == DRIVE_DA_ARC);
}

static void stop_route(DriveMode stoppedMode)
{
    gDriveMode = stoppedMode;
    gLastLineError = 0;
    stop_car();
}

static void begin_straight(DriveMode straightMode, bool alreadyOnWhite)
{
    reset_segment_encoders();
    gSegmentHeadingTargetMdeg = gHeadingMdeg;
    gStraightHasSeenWhite = alreadyOnWhite;
    gLastLineError = 0;
    gDriveMode = straightMode;
}

static void begin_arc(DriveMode arcMode, bool lineAlreadyAcquired)
{
    reset_segment_encoders();
    gArcStartHeadingMdeg = gHeadingMdeg;
    gArcLineAcquired = lineAlreadyAcquired;
    gLastLineError = 0;
    gDriveMode = arcMode;
}

static void start_route(void)
{
    gHeadingMdeg = 0;
    gHeadingRemainder = 0;
    gImuLastSampleMs = gMs;
    gRouteStartMs = gMs;
    begin_straight(DRIVE_AB_STRAIGHT, false);
}

static void toggle_route(void)
{
    if (route_is_active()) {
        stop_route(DRIVE_IDLE);
    } else {
        start_route();
    }
}

static void finish_straight(uint8_t lineMask)
{
    if (gDriveMode == DRIVE_AB_STRAIGHT) {
        begin_arc(DRIVE_BC_ARC, lineMask != 0U);
    } else {
        begin_arc(DRIVE_DA_ARC, lineMask != 0U);
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

static bool arc_heading_reached_180(void)
{
    int32_t headingDeltaMdeg;

    if (!gImuReady) {
        return false;
    }

    headingDeltaMdeg = gHeadingMdeg - gArcStartHeadingMdeg;
    if (headingDeltaMdeg < 0) {
        headingDeltaMdeg = -headingDeltaMdeg;
    }

    return headingDeltaMdeg >= ARC_HEADING_CHANGE_MDEG;
}

static void drive_straight_5ms(void)
{
    uint8_t lineMask = line_sensor_read_mask();
    uint32_t motorACounts = abs_encoder_count(gEncoderA);
    uint32_t motorBCounts = abs_encoder_count(gEncoderB);
    uint32_t averageCounts = (motorACounts + motorBCounts) / 2U;
    int32_t balanceCorrection =
        ((int32_t) motorACounts - (int32_t) motorBCounts) /
        STRAIGHT_BALANCE_DIVISOR;
    int32_t headingCorrection = 0;

    balanceCorrection = clamp_i32(balanceCorrection,
        -STRAIGHT_BALANCE_MAX, STRAIGHT_BALANCE_MAX);

    if (gImuReady) {
        headingCorrection =
            (gHeadingMdeg - gSegmentHeadingTargetMdeg) /
            STRAIGHT_HEADING_KP_DIVISOR;
        headingCorrection = clamp_i32(headingCorrection,
            -STRAIGHT_HEADING_CORRECTION_MAX,
            STRAIGHT_HEADING_CORRECTION_MAX);
    }

    apply_forward_steering(
        MOTOR_PWM_CRUISE, balanceCorrection + headingCorrection);

    if (lineMask == 0U) {
        gStraightHasSeenWhite = true;
    }

    if (((lineMask != 0U) && gStraightHasSeenWhite &&
            (averageCounts >= STRAIGHT_MIN_COUNTS_FOR_LINE)) ||
        (averageCounts >= STRAIGHT_TARGET_COUNTS)) {
        finish_straight(lineMask);
    }
}

static void drive_arc_5ms(void)
{
    uint8_t lineMask = line_sensor_read_mask();
    uint32_t averageCounts = segment_average_counts();
    int32_t lineCorrection;
    int32_t headingCorrection = 0;

    if (lineMask != 0U) {
        gArcLineAcquired = true;
    }

    lineCorrection = ((int32_t) line_error_from_mask(lineMask) *
                         LINE_PWM_CORRECTION_MAX) /
        LINE_ERROR_MAX;

    if (gImuReady) {
        uint32_t limitedCounts = averageCounts;
        int32_t expectedHeadingMdeg;
        int32_t headingErrorMdeg;

        if (limitedCounts > ARC_TARGET_COUNTS) {
            limitedCounts = ARC_TARGET_COUNTS;
        }
        expectedHeadingMdeg = gArcStartHeadingMdeg -
            (int32_t) (((int64_t) limitedCounts *
                           ARC_HEADING_CHANGE_MDEG) /
                ARC_TARGET_COUNTS);
        headingErrorMdeg = gHeadingMdeg - expectedHeadingMdeg;
        headingCorrection = headingErrorMdeg / ARC_HEADING_KP_DIVISOR;
        headingCorrection = clamp_i32(headingCorrection,
            -ARC_HEADING_CORRECTION_MAX,
            ARC_HEADING_CORRECTION_MAX);
    }

    apply_forward_steering(
        MOTOR_PWM_CRUISE, lineCorrection + headingCorrection);

    if (arc_heading_reached_180()) {
        finish_arc();
    } else if (!gArcLineAcquired &&
        (averageCounts >= ARC_ACQUIRE_LIMIT_COUNTS)) {
        stop_route(DRIVE_FAULT);
    } else if (averageCounts >= ARC_MAX_COUNTS) {
        finish_arc();
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
                toggle_route();
            }
        }
    }

    if (debouncedPressed && !longPressHandled &&
        ((gMs - pressStartMs) >= KEY_LONG_PRESS_MS)) {
        longPressHandled = true;
        stop_route(DRIVE_IDLE);
    }
}

static void drive_tick_5ms(void)
{
    if (route_is_active() && elapsed_ms(gRouteStartMs, ROUTE_TIMEOUT_MS)) {
        stop_route(DRIVE_FAULT);
        return;
    }

    switch (gDriveMode) {
    case DRIVE_AB_STRAIGHT:
    case DRIVE_CD_STRAIGHT:
        drive_straight_5ms();
        break;

    case DRIVE_BC_ARC:
    case DRIVE_DA_ARC:
        drive_arc_5ms();
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

    DL_GPIO_setPins(LED_PORT, LED_led_PIN);
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

    gImuReady = mpu6050_init_and_calibrate();
    __disable_irq();
    gControlTicksPending = 0U;
    __enable_irq();

    while (1) {
        if (take_control_tick()) {
            key_scan_5ms();
            imu_update_heading();
            drive_tick_5ms();
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
        uint8_t currentState =
            encoder_state(ENCODER_A_PORT, ENCODER_A1_PIN, ENCODER_A2_PIN);
        gEncoderA += quadrature_delta(gEncoderAPreviousState, currentState);
        gEncoderAPreviousState = currentState;
    }

    if ((encoderBStatus & (ENCODER_B1_PIN | ENCODER_B2_PIN)) != 0U) {
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
