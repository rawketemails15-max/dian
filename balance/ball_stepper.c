#include "ball_stepper.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t gCurrentSteps;
static volatile int32_t gTargetSteps;
static volatile int8_t gStepSign;
static volatile uint16_t gStepHz;
static volatile uint16_t gRequestedHz;
static volatile uint8_t gDirectionLevel;
static volatile bool gEnabled;
static volatile bool gRunning;
static volatile bool gAtLimit;
static uint32_t gEnableMs;

static int32_t clamp_steps(int32_t steps)
{
    if (steps < APP_BALL_MIN_STEPS) {
        return APP_BALL_MIN_STEPS;
    }
    if (steps > APP_BALL_MAX_STEPS) {
        return APP_BALL_MAX_STEPS;
    }
    return steps;
}

static uint16_t clamp_hz(uint16_t hz)
{
    if (hz < APP_BALL_STEP_MIN_HZ) {
        return APP_BALL_STEP_MIN_HZ;
    }
    if (hz > APP_BALL_STEP_MAX_HZ) {
        return APP_BALL_STEP_MAX_HZ;
    }
    return hz;
}

static void stop_step_output(void)
{
    DL_TimerG_stopCounter(PWM_BALL_STEP_INST);
    DL_GPIO_initDigitalOutput(GPIO_PWM_BALL_STEP_C1_IOMUX);
    DL_GPIO_clearPins(
        GPIO_PWM_BALL_STEP_C1_PORT, GPIO_PWM_BALL_STEP_C1_PIN);
    DL_GPIO_enableOutput(
        GPIO_PWM_BALL_STEP_C1_PORT, GPIO_PWM_BALL_STEP_C1_PIN);
    gRunning = false;
    gStepHz = 0U;
}

static void set_direction(int8_t sign)
{
    uint8_t level = (sign > 0) ? 1U : 0U;

    if (APP_BALL_DIR_INVERT != 0U) {
        level ^= 1U;
    }
    if (level != 0U) {
        DL_GPIO_setPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    }
    gDirectionLevel = level;
    gStepSign = sign;
}

static void start_step_output(uint16_t hz)
{
    uint32_t periodTicks;

    hz = clamp_hz(hz);
    periodTicks = APP_BALL_STEP_CLOCK_HZ / (uint32_t) hz;
    if (periodTicks < 4U) {
        periodTicks = 4U;
    }

    /*
     * Caller has already stopped the timer and forced STEP low.  Keeping that
     * ordering explicit guarantees DIR changes only between complete packets.
     */
    DL_TimerG_setLoadValue(PWM_BALL_STEP_INST, periodTicks);
    DL_TimerG_setTimerCount(PWM_BALL_STEP_INST, periodTicks);
    DL_TimerG_setCaptureCompareValue(PWM_BALL_STEP_INST,
        periodTicks / 2U, GPIO_PWM_BALL_STEP_C1_IDX);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_BALL_STEP_C1_IOMUX,
        GPIO_PWM_BALL_STEP_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(
        GPIO_PWM_BALL_STEP_C1_PORT, GPIO_PWM_BALL_STEP_C1_PIN);
    DL_TimerG_startCounter(PWM_BALL_STEP_INST);
    gStepHz = hz;
    gRunning = true;
}

static void update_running_frequency(uint16_t hz)
{
    uint32_t periodTicks;
    uint32_t primask = __get_PRIMASK();

    hz = clamp_hz(hz);
    periodTicks = APP_BALL_STEP_CLOCK_HZ / (uint32_t) hz;
    if (periodTicks < 4U) {
        periodTicks = 4U;
    }

    /*
     * PWM_BALL_STEP is configured for shadow loading in SysConfig.  Updating
     * LOAD/CC while running therefore takes effect together at the next timer
     * boundary and does not truncate the current STEP pulse.
     */
    __disable_irq();
    if (gRunning) {
        DL_TimerG_setLoadValue(PWM_BALL_STEP_INST, periodTicks);
        DL_TimerG_setCaptureCompareValue(PWM_BALL_STEP_INST,
            periodTicks / 2U, GPIO_PWM_BALL_STEP_C1_IDX);
        gStepHz = hz;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

void ball_stepper_init(uint32_t nowMs)
{
    gCurrentSteps = 0;
    gTargetSteps = 0;
    gStepSign = 1;
    gStepHz = 0U;
    gRequestedHz = APP_BALL_STEP_MIN_HZ;
    gDirectionLevel = 0U;
    gEnabled = false;
    gRunning = false;
    gAtLimit = false;
    gEnableMs = nowMs;

    stop_step_output();
    DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    DL_GPIO_clearPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
}

void ball_stepper_enable(uint32_t nowMs)
{
    if (!gEnabled) {
        stop_step_output();
        gTargetSteps = gCurrentSteps;
        DL_GPIO_setPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
        gEnableMs = nowMs;
        gEnabled = true;
    }
}

void ball_stepper_disarm(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    stop_step_output();
    gTargetSteps = gCurrentSteps;
    DL_GPIO_clearPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
    gEnabled = false;
    if (primask == 0U) {
        __enable_irq();
    }
}

void ball_stepper_hold(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    stop_step_output();
    gTargetSteps = gCurrentSteps;
    if (primask == 0U) {
        __enable_irq();
    }
}

void ball_stepper_set_target(int32_t targetSteps)
{
    uint32_t primask = __get_PRIMASK();
    int32_t clamped = clamp_steps(targetSteps);

    __disable_irq();
    gAtLimit = (clamped != targetSteps);
    gTargetSteps = clamped;
    if (gRunning &&
        (((gStepSign > 0) && (gTargetSteps <= gCurrentSteps)) ||
         ((gStepSign < 0) && (gTargetSteps >= gCurrentSteps)))) {
        stop_step_output();
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

void ball_stepper_set_requested_hz(uint16_t stepHz)
{
    gRequestedHz = clamp_hz(stepHz);
}

void ball_stepper_tick_5ms(uint32_t nowMs)
{
    int8_t wantedSign;
    uint16_t nextHz;

    if (!gEnabled ||
        ((uint32_t) (nowMs - gEnableMs) < APP_D36A_WAKE_DELAY_MS)) {
        return;
    }
    if (gCurrentSteps == gTargetSteps) {
        if (gRunning) {
            stop_step_output();
        }
        return;
    }

    wantedSign = (gTargetSteps > gCurrentSteps) ? 1 : -1;
    if ((!gRunning) || (wantedSign != gStepSign)) {
        if (gRunning) {
            stop_step_output();
        }
        set_direction(wantedSign);
        /*
         * One 32 MHz cycle already exceeds the D36A 200 ns DIR setup only
         * after several cycles; use 1 us to retain comfortable margin.
         */
        DL_Common_delayCycles(32U);
        start_step_output(APP_BALL_STEP_MIN_HZ);
        return;
    }

    nextHz = gStepHz;
    if (nextHz < gRequestedHz) {
        uint16_t delta = (uint16_t) (gRequestedHz - nextHz);
        if (delta > APP_BALL_STEP_HZ_SLEW_PER_TICK) {
            delta = APP_BALL_STEP_HZ_SLEW_PER_TICK;
        }
        nextHz = (uint16_t) (nextHz + delta);
    } else if (nextHz > gRequestedHz) {
        uint16_t delta = (uint16_t) (nextHz - gRequestedHz);
        if (delta > APP_BALL_STEP_HZ_SLEW_PER_TICK) {
            delta = APP_BALL_STEP_HZ_SLEW_PER_TICK;
        }
        nextHz = (uint16_t) (nextHz - delta);
    }
    if (nextHz != gStepHz) {
        update_running_frequency(nextHz);
    }
}

void ball_stepper_step_isr(void)
{
    int32_t next;
    DL_TIMER_IIDX pending =
        DL_TimerG_getPendingInterrupt(PWM_BALL_STEP_INST);

    if ((pending != DL_TIMERG_IIDX_CC1_DN) || !gRunning) {
        return;
    }
    if (((gStepSign > 0) && (gTargetSteps <= gCurrentSteps)) ||
        ((gStepSign < 0) && (gTargetSteps >= gCurrentSteps))) {
        stop_step_output();
        return;
    }

    next = gCurrentSteps + (int32_t) gStepSign;
    if ((next < APP_BALL_MIN_STEPS) || (next > APP_BALL_MAX_STEPS)) {
        gAtLimit = true;
        gTargetSteps = gCurrentSteps;
        stop_step_output();
        return;
    }

    gCurrentSteps = next;
    if (gCurrentSteps == gTargetSteps) {
        stop_step_output();
    }
}

BallStepperStatus ball_stepper_get_status(void)
{
    BallStepperStatus status;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    status.currentSteps = gCurrentSteps;
    status.targetSteps = gTargetSteps;
    status.stepHz = gStepHz;
    status.directionLevel = gDirectionLevel;
    status.enabled = gEnabled;
    status.running = gRunning;
    status.atLimit = gAtLimit ||
        (gCurrentSteps <= APP_BALL_MIN_STEPS) ||
        (gCurrentSteps >= APP_BALL_MAX_STEPS);
    if (primask == 0U) {
        __enable_irq();
    }
    return status;
}
