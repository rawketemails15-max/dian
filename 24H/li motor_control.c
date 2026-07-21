#include "motor_control.h"

#include <limits.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

/*
 * The li controller expresses PWM in its original 1600-tick domain.  The
 * four-segment project already uses an 8000-tick TIMG6 period for the arc
 * controller, so only the hardware write is scaled; the PI math and all li
 * tuning constants remain unchanged.
 */
#define CHASSIS_PWM_PERIOD_TICKS (8000U)

typedef struct {
    int32_t previousCount;
    int32_t integralTicks;
    uint32_t lastMotionMs;
    uint32_t speedCount;
    uint16_t pwmTicks;
} MotorPiState;

static volatile int32_t gEncoderCountA;
static volatile int32_t gEncoderCountB;
static volatile uint8_t gEncoderStateA;
static volatile uint8_t gEncoderStateB;

static MotorPiState gMotorA;
static MotorPiState gMotorB;
static MotorTelemetry gTelemetry;

static const int8_t gQuadratureStep[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t read_encoder_a_state(void)
{
    uint32_t pins = DL_GPIO_readPins(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    uint8_t state = 0U;

    if ((pins & ENCODERA_E1A_PIN) != 0U) {
        state |= 0x02U;
    }
    if ((pins & ENCODERA_E1B_PIN) != 0U) {
        state |= 0x01U;
    }
    return state;
}

static uint8_t read_encoder_b_state(void)
{
    uint32_t pins = DL_GPIO_readPins(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    uint8_t state = 0U;

    if ((pins & ENCODERB_E2A_PIN) != 0U) {
        state |= 0x02U;
    }
    if ((pins & ENCODERB_E2B_PIN) != 0U) {
        state |= 0x01U;
    }
    return state;
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

static uint32_t magnitude_i32(int32_t value)
{
    if (value >= 0) {
        return (uint32_t) value;
    }
    if (value == INT32_MIN) {
        return ((uint32_t) INT32_MAX + 1U);
    }
    return (uint32_t) (-value);
}

static void snapshot_encoder_counts(int32_t *countA, int32_t *countB)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *countA = gEncoderCountA;
    *countB = gEncoderCountB;
    if (primask == 0U) {
        __enable_irq();
    }
}

static void set_motor_direction(GPIO_Regs *port, uint32_t d1Pin,
    uint32_t d2Pin, bool d1High, bool d2High)
{
    DL_GPIO_clearPins(port, d1Pin | d2Pin);
    if (d1High) {
        DL_GPIO_setPins(port, d1Pin);
    }
    if (d2High) {
        DL_GPIO_setPins(port, d2Pin);
    }
}

static void write_pwm(uint32_t channelIndex, uint16_t dutyTicks)
{
    uint32_t hardwareDutyTicks;

    if (dutyTicks > APP_PWM_PERIOD_TICKS) {
        dutyTicks = APP_PWM_PERIOD_TICKS;
    }
    hardwareDutyTicks =
        (((uint32_t) dutyTicks * CHASSIS_PWM_PERIOD_TICKS) +
            (APP_PWM_PERIOD_TICKS / 2U)) /
        APP_PWM_PERIOD_TICKS;
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, hardwareDutyTicks,
        channelIndex);
}

static void brake_outputs(void)
{
    DL_TimerG_stopCounter(PWM_0_INST);
    write_pwm(GPIO_PWM_0_C0_IDX, 0U);
    write_pwm(GPIO_PWM_0_C1_IDX, 0U);
    DL_GPIO_clearPins(GPIO_PWM_0_C0_PORT,
        GPIO_PWM_0_C0_PIN | GPIO_PWM_0_C1_PIN);
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    gMotorA.pwmTicks = 0U;
    gMotorB.pwmTicks = 0U;
    gTelemetry.targetCountA = 0U;
    gTelemetry.targetCountB = 0U;
    gTelemetry.pwmTicksA = 0U;
    gTelemetry.pwmTicksB = 0U;
    gTelemetry.running = false;
}

static uint16_t run_pi(MotorPiState *motor, uint32_t speed,
    uint16_t target)
{
    int32_t error = (int32_t) target - (int32_t) speed;
    int32_t output;
    uint32_t feedforward;

    if (target == 0U) {
        motor->integralTicks = 0;
        return 0U;
    }

    motor->integralTicks += error * APP_PI_KI_TICKS_PER_COUNT_STEP;
    motor->integralTicks = clamp_i32(motor->integralTicks,
        APP_PI_INTEGRAL_MIN_TICKS, APP_PI_INTEGRAL_MAX_TICKS);

    feedforward = APP_PWM_FEEDFORWARD_MIN_TICKS +
        (APP_PWM_FEEDFORWARD_TICKS_PER_COUNT * target);
    output = (int32_t) feedforward +
        (error * APP_PI_KP_TICKS_PER_COUNT) + motor->integralTicks;
    output = clamp_i32(output, 0, APP_PWM_MAX_TICKS);
    return (uint16_t) output;
}

void motor_control_init(void)
{
    int32_t countA;
    int32_t countB;

    gEncoderCountA = 0;
    gEncoderCountB = 0;
    gEncoderStateA = read_encoder_a_state();
    gEncoderStateB = read_encoder_b_state();
    snapshot_encoder_counts(&countA, &countB);

    gMotorA = (MotorPiState) {0};
    gMotorB = (MotorPiState) {0};
    gMotorA.previousCount = countA;
    gMotorB.previousCount = countB;
    gTelemetry = (MotorTelemetry) {0};
    brake_outputs();
}

void motor_control_start(uint32_t nowMs)
{
    int32_t countA;
    int32_t countB;

    brake_outputs();
    snapshot_encoder_counts(&countA, &countB);
    gMotorA = (MotorPiState) {0};
    gMotorB = (MotorPiState) {0};
    gMotorA.previousCount = countA;
    gMotorB.previousCount = countB;
    gMotorA.lastMotionMs = nowMs;
    gMotorB.lastMotionMs = nowMs;

    gTelemetry = (MotorTelemetry) {0};
    gTelemetry.encoderCountA = countA;
    gTelemetry.encoderCountB = countB;
    gTelemetry.running = true;

    /* In this SysConfig AIN1/BIN1 are physical D2, and AIN2/BIN2 are D1. */
    set_motor_direction(AIN_PORT, AIN_AIN2_PIN, AIN_AIN1_PIN,
        APP_MOTOR_A_FORWARD_D1 != 0U, APP_MOTOR_A_FORWARD_D2 != 0U);
    set_motor_direction(BIN_PORT, BIN_BIN2_PIN, BIN_BIN1_PIN,
        APP_MOTOR_B_FORWARD_D1 != 0U, APP_MOTOR_B_FORWARD_D2 != 0U);

    write_pwm(GPIO_PWM_0_C0_IDX, 0U);
    write_pwm(GPIO_PWM_0_C1_IDX, 0U);
    DL_TimerG_setTimerCount(PWM_0_INST, CHASSIS_PWM_PERIOD_TICKS);
    DL_TimerG_startCounter(PWM_0_INST);
}

void motor_control_brake(void)
{
    brake_outputs();
    gTelemetry.stalled = false;
}

void motor_control_update_20ms(uint32_t nowMs, uint16_t targetCountA,
    uint16_t targetCountB)
{
    int32_t countA;
    int32_t countB;
    int32_t deltaA;
    int32_t deltaB;

    if (!gTelemetry.running) {
        return;
    }

    if (targetCountA > APP_MOTOR_TARGET_MAX_COUNTS) {
        targetCountA = APP_MOTOR_TARGET_MAX_COUNTS;
    }
    if (targetCountB > APP_MOTOR_TARGET_MAX_COUNTS) {
        targetCountB = APP_MOTOR_TARGET_MAX_COUNTS;
    }
    gTelemetry.targetCountA = targetCountA;
    gTelemetry.targetCountB = targetCountB;

    snapshot_encoder_counts(&countA, &countB);
    deltaA = countA - gMotorA.previousCount;
    deltaB = countB - gMotorB.previousCount;
    gMotorA.previousCount = countA;
    gMotorB.previousCount = countB;
    gMotorA.speedCount = magnitude_i32(deltaA);
    gMotorB.speedCount = magnitude_i32(deltaB);

    if (gMotorA.speedCount != 0U) {
        gMotorA.lastMotionMs = nowMs;
    }
    if (gMotorB.speedCount != 0U) {
        gMotorB.lastMotionMs = nowMs;
    }
    if (targetCountA == 0U) {
        gMotorA.lastMotionMs = nowMs;
    }
    if (targetCountB == 0U) {
        gMotorB.lastMotionMs = nowMs;
    }

    if (((targetCountA != 0U) &&
            ((uint32_t) (nowMs - gMotorA.lastMotionMs) >=
                APP_ENCODER_STALL_TIMEOUT_MS)) ||
        ((targetCountB != 0U) &&
            ((uint32_t) (nowMs - gMotorB.lastMotionMs) >=
                APP_ENCODER_STALL_TIMEOUT_MS))) {
        gTelemetry.encoderCountA = countA;
        gTelemetry.encoderCountB = countB;
        gTelemetry.speedCountA = gMotorA.speedCount;
        gTelemetry.speedCountB = gMotorB.speedCount;
        gTelemetry.stalled = true;
        brake_outputs();
        return;
    }

    gMotorA.pwmTicks = run_pi(&gMotorA, gMotorA.speedCount,
        targetCountA);
    gMotorB.pwmTicks = run_pi(&gMotorB, gMotorB.speedCount,
        targetCountB);
    write_pwm(GPIO_PWM_0_C0_IDX, gMotorA.pwmTicks);
    write_pwm(GPIO_PWM_0_C1_IDX, gMotorB.pwmTicks);

    gTelemetry.encoderCountA = countA;
    gTelemetry.encoderCountB = countB;
    gTelemetry.speedCountA = gMotorA.speedCount;
    gTelemetry.speedCountB = gMotorB.speedCount;
    gTelemetry.pwmTicksA = gMotorA.pwmTicks;
    gTelemetry.pwmTicksB = gMotorB.pwmTicks;
}

void motor_control_handle_encoder_a_irq(void)
{
    uint8_t newState = read_encoder_a_state();
    uint8_t transition = (uint8_t) ((gEncoderStateA << 2U) | newState);

    gEncoderCountA += gQuadratureStep[transition];
    gEncoderStateA = newState;
}

void motor_control_handle_encoder_b_irq(void)
{
    uint8_t newState = read_encoder_b_state();
    uint8_t transition = (uint8_t) ((gEncoderStateB << 2U) | newState);

    gEncoderCountB += gQuadratureStep[transition];
    gEncoderStateB = newState;
}

bool motor_control_stalled(void)
{
    return gTelemetry.stalled;
}

MotorTelemetry motor_control_get_telemetry(void)
{
    MotorTelemetry telemetry;
    int32_t countA;
    int32_t countB;

    snapshot_encoder_counts(&countA, &countB);
    telemetry = gTelemetry;
    telemetry.encoderCountA = countA;
    telemetry.encoderCountB = countB;
    return telemetry;
}
