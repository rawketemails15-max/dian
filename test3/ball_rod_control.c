#include "ball_rod_control.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t gCurrentSteps;
static volatile int32_t gTargetSteps;
static volatile bool gStepRunning;
static volatile int8_t gStepSign;

static BallRodTelemetry gTelemetry;
static BallRodState gState;
static uint16_t gStepFrequencyHz;
static uint16_t gCommandFrequencyHz;
static uint32_t gStateStartMs;
static uint32_t gRunStartMs;
static uint32_t gFinalRunMs;

static bool gButtonRaw;
static bool gButtonStable;
static bool gButtonPressEligible;
static uint32_t gButtonRawChangedMs;

static bool elapsed_ms(
    uint32_t nowMs, uint32_t startMs, uint32_t durationMs)
{
    return (uint32_t) (nowMs - startMs) >= durationMs;
}

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

static void step_pin_gpio_low_unlocked(void)
{
    DL_TimerG_stopCounter(PWM_BALL_STEP_INST);
    DL_GPIO_initDigitalOutput(GPIO_PWM_BALL_STEP_C1_IOMUX);
    DL_GPIO_clearPins(
        GPIO_PWM_BALL_STEP_C1_PORT, GPIO_PWM_BALL_STEP_C1_PIN);
    DL_GPIO_enableOutput(
        GPIO_PWM_BALL_STEP_C1_PORT, GPIO_PWM_BALL_STEP_C1_PIN);
    gStepRunning = false;
    gStepFrequencyHz = 0U;
}

static void step_pin_gpio_low(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    step_pin_gpio_low_unlocked();
    if (primask == 0U) {
        __enable_irq();
    }
}

static void set_direction_for_sign_unlocked(int8_t sign)
{
    bool high = sign > 0;

    if (APP_BALL_DIR_INVERT != 0U) {
        high = !high;
    }

    /* Stop STEP and force it low before every DIR transition. */
    step_pin_gpio_low_unlocked();
    if (high) {
        DL_GPIO_setPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    }
    gStepSign = sign;
    gTelemetry.directionLevel = high ? 1U : 0U;
}

static void set_direction_for_sign(int8_t sign)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    set_direction_for_sign_unlocked(sign);
    if (primask == 0U) {
        __enable_irq();
    }
}

static void start_step(uint16_t frequencyHz, int8_t sign)
{
    uint32_t periodCounts;
    uint32_t loadValue;
    uint32_t primask;

    if (frequencyHz < APP_BALL_STEP_MIN_HZ) {
        frequencyHz = APP_BALL_STEP_MIN_HZ;
    } else if (frequencyHz > APP_BALL_STEP_MAX_HZ) {
        frequencyHz = APP_BALL_STEP_MAX_HZ;
    }

    if (gStepRunning && (sign == gStepSign) &&
        (frequencyHz == gStepFrequencyHz)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (sign != gStepSign) {
        set_direction_for_sign_unlocked(sign);
        /*
         * Give D36A at least 1 us of DIR setup time before STEP restarts.
         * This comfortably exceeds the driver's 200 ns minimum.
         */
        DL_Common_delayCycles((CPUCLK_FREQ / 1000000U) + 1U);
    } else {
        step_pin_gpio_low_unlocked();
    }

    periodCounts = APP_BALL_STEP_CLOCK_HZ / frequencyHz;
    if (periodCounts < 2U) {
        periodCounts = 2U;
    }
    loadValue = periodCounts - 1U;

    DL_TimerG_setCaptCompUpdateMethod(PWM_BALL_STEP_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(PWM_BALL_STEP_INST, loadValue);
    DL_TimerG_setTimerCount(PWM_BALL_STEP_INST, loadValue);
    DL_TimerG_setCaptureCompareValue(PWM_BALL_STEP_INST,
        periodCounts / 2U, DL_TIMER_CC_1_INDEX);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_BALL_STEP_C1_IOMUX,
        GPIO_PWM_BALL_STEP_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(
        GPIO_PWM_BALL_STEP_C1_PORT, GPIO_PWM_BALL_STEP_C1_PIN);

    DL_TimerG_clearInterruptStatus(
        PWM_BALL_STEP_INST, DL_TIMERG_INTERRUPT_CC1_DN_EVENT);
    NVIC_ClearPendingIRQ(PWM_BALL_STEP_INST_INT_IRQN);
    gStepFrequencyHz = frequencyHz;
    gStepRunning = true;
    DL_TimerG_startCounter(PWM_BALL_STEP_INST);
    if (primask == 0U) {
        __enable_irq();
    }
}

static void move_toward_target(void)
{
    int32_t distance = gTargetSteps - gCurrentSteps;

    if (distance == 0) {
        step_pin_gpio_low();
        return;
    }
    start_step(gCommandFrequencyHz, (distance > 0) ? 1 : -1);
}

static bool target_reached(void)
{
    bool reached;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    reached = (gCurrentSteps == gTargetSteps) && !gStepRunning;
    if (primask == 0U) {
        __enable_irq();
    }
    return reached;
}

static void set_state(BallRodState state, uint32_t nowMs)
{
    gState = state;
    gStateStartMs = nowMs;
    gTelemetry.state = state;
}

static void set_motion_target(
    BallRodState state, int32_t targetSteps, uint16_t frequencyHz,
    uint32_t nowMs)
{
    /*
     * Stop the current pulse train before publishing a new target.  This
     * prevents the STEP ISR from observing a new target with the old DIR.
     */
    step_pin_gpio_low();
    gTargetSteps = clamp_steps(targetSteps);
    gCommandFrequencyHz = frequencyHz;
    set_state(state, nowMs);
    move_toward_target();
}

static bool button_pressed_event(uint32_t nowMs, bool pressed)
{
    if (pressed != gButtonRaw) {
        gButtonRaw = pressed;
        gButtonRawChangedMs = nowMs;
        if (pressed) {
            /*
             * Remember where the physical press began.  A press that begins
             * during a run stays blocked even if its debounce interval ends
             * after the controller has entered COMPLETE.
             */
            gButtonPressEligible =
                (gState == BALL_ROD_IDLE) ||
                (gState == BALL_ROD_COMPLETE);
        } else {
            gButtonPressEligible = false;
        }
    }

    if ((gButtonStable != gButtonRaw) &&
        elapsed_ms(nowMs, gButtonRawChangedMs, APP_BUTTON_DEBOUNCE_MS)) {
        gButtonStable = gButtonRaw;
        if (gButtonStable && gButtonPressEligible) {
            gButtonPressEligible = false;
            return true;
        }
    }
    return false;
}

static bool state_is_running(BallRodState state)
{
    return (state >= BALL_ROD_WAKE) && (state <= BALL_ROD_SETTLE);
}

static void start_sequence(uint32_t nowMs)
{
    /*
     * The user guarantees that the pipe is horizontal and the ball is at O
     * when BLS is pressed.  A previous completed run also ends at zero steps.
     */
    step_pin_gpio_low();
    gTargetSteps = gCurrentSteps;
    gTelemetry.sequenceTimedOut = false;
    gRunStartMs = nowMs;
    gFinalRunMs = 0U;

    set_direction_for_sign(1);
    DL_GPIO_setPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
    gTelemetry.driverEnabled = true;
    set_state(BALL_ROD_WAKE, nowMs);
}

static void start_timeout_level(uint32_t nowMs)
{
    gTelemetry.sequenceTimedOut = true;
    set_motion_target(
        BALL_ROD_TIMEOUT_LEVEL, 0, APP_OL_LEVEL_HZ, nowMs);
}

void ball_rod_init(uint32_t nowMs)
{
    gCurrentSteps = 0;
    gTargetSteps = 0;
    gStepRunning = false;
    gStepSign = 1;
    gStepFrequencyHz = 0U;
    gCommandFrequencyHz = APP_OL_LAUNCH_HZ;
    gState = BALL_ROD_IDLE;
    gStateStartMs = nowMs;
    gRunStartMs = nowMs;
    gFinalRunMs = 0U;

    gButtonRaw = false;
    gButtonStable = false;
    gButtonPressEligible = false;
    gButtonRawChangedMs = nowMs;

    gTelemetry = (BallRodTelemetry) {0};
    gTelemetry.state = BALL_ROD_IDLE;

    step_pin_gpio_low();
    DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    DL_GPIO_clearPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
}

void ball_rod_tick_5ms(uint32_t nowMs, bool buttonPressed)
{
    bool pressedEvent = button_pressed_event(nowMs, buttonPressed);

    /*
     * Per the selected behavior, every press during a running sequence is
     * ignored.  IDLE and COMPLETE are the only startable states.
     */
    if (pressedEvent &&
        ((gState == BALL_ROD_IDLE) || (gState == BALL_ROD_COMPLETE))) {
        start_sequence(nowMs);
    }

    if (state_is_running(gState) &&
        elapsed_ms(nowMs, gRunStartMs, APP_OL_RUN_TIMEOUT_MS)) {
        start_timeout_level(nowMs);
    }

    switch (gState) {
        case BALL_ROD_IDLE:
        case BALL_ROD_COMPLETE:
        case BALL_ROD_FAULT:
            step_pin_gpio_low();
            break;

        case BALL_ROD_WAKE:
            if (elapsed_ms(nowMs, gStateStartMs,
                    APP_D36A_WAKE_DELAY_MS)) {
                set_motion_target(BALL_ROD_LAUNCH,
                    APP_OL_LAUNCH_STEPS, APP_OL_LAUNCH_HZ, nowMs);
            }
            break;

        case BALL_ROD_LAUNCH:
            move_toward_target();
            if (target_reached()) {
                if (APP_OL_LAUNCH_HOLD_MS == 0U) {
                    set_motion_target(BALL_ROD_REVERSE,
                        APP_OL_REVERSE_STEPS, APP_OL_REVERSE_HZ, nowMs);
                } else {
                    set_state(BALL_ROD_LAUNCH_HOLD, nowMs);
                }
            }
            break;

        case BALL_ROD_LAUNCH_HOLD:
            step_pin_gpio_low();
            if (elapsed_ms(nowMs, gStateStartMs,
                    APP_OL_LAUNCH_HOLD_MS)) {
                set_motion_target(BALL_ROD_REVERSE,
                    APP_OL_REVERSE_STEPS, APP_OL_REVERSE_HZ, nowMs);
            }
            break;

        case BALL_ROD_REVERSE:
            move_toward_target();
            if (target_reached()) {
                set_state(BALL_ROD_RETURN_HOLD, nowMs);
            }
            break;

        case BALL_ROD_RETURN_HOLD:
            step_pin_gpio_low();
            if (elapsed_ms(nowMs, gStateStartMs,
                    APP_OL_RETURN_HOLD_MS)) {
                set_motion_target(
                    BALL_ROD_LEVEL, 0, APP_OL_LEVEL_HZ, nowMs);
            }
            break;

        case BALL_ROD_LEVEL:
            move_toward_target();
            if (target_reached()) {
                set_state(BALL_ROD_SETTLE, nowMs);
            }
            break;

        case BALL_ROD_SETTLE:
            step_pin_gpio_low();
            if (elapsed_ms(nowMs, gStateStartMs,
                    APP_OL_FINAL_SETTLE_MS)) {
                gFinalRunMs = (uint32_t) (nowMs - gRunStartMs);
                set_state(BALL_ROD_COMPLETE, nowMs);
            }
            break;

        case BALL_ROD_TIMEOUT_LEVEL:
            move_toward_target();
            if (target_reached()) {
                gFinalRunMs = (uint32_t) (nowMs - gRunStartMs);
                set_state(BALL_ROD_FAULT, nowMs);
            }
            break;

        default:
            start_timeout_level(nowMs);
            break;
    }

    gTelemetry.currentSteps = gCurrentSteps;
    gTelemetry.targetSteps = gTargetSteps;
    gTelemetry.stepFrequencyHz = gStepFrequencyHz;
    gTelemetry.stepRunning = gStepRunning;
    if (state_is_running(gState) ||
        (gState == BALL_ROD_TIMEOUT_LEVEL)) {
        gTelemetry.runElapsedMs = (uint32_t) (nowMs - gRunStartMs);
    } else {
        gTelemetry.runElapsedMs = gFinalRunMs;
    }
}

void ball_rod_step_isr(void)
{
    if ((DL_TimerG_getPendingInterrupt(PWM_BALL_STEP_INST) ==
            DL_TIMERG_IIDX_CC1_DN) &&
        gStepRunning) {
        gCurrentSteps += gStepSign;

        if (((gStepSign > 0) && (gCurrentSteps >= gTargetSteps)) ||
            ((gStepSign < 0) && (gCurrentSteps <= gTargetSteps))) {
            gCurrentSteps = gTargetSteps;
            step_pin_gpio_low_unlocked();
        }
    }
}

BallRodTelemetry ball_rod_get_telemetry(void)
{
    BallRodTelemetry telemetry;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    telemetry = gTelemetry;
    telemetry.currentSteps = gCurrentSteps;
    telemetry.targetSteps = gTargetSteps;
    telemetry.stepFrequencyHz = gStepFrequencyHz;
    telemetry.stepRunning = gStepRunning;
    if (primask == 0U) {
        __enable_irq();
    }
    return telemetry;
}
