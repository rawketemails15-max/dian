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
static int32_t gRunZeroSteps;
static int8_t gTrimDirection;
static bool gAdjustmentUiActive;

static bool gButtonRaw;
static bool gButtonStable;
static bool gButtonRawPressEligible;
static bool gButtonPressEligible;
static bool gButtonLongHandled;
static bool gSecondClickCandidate;
static bool gClickPending;
static uint32_t gButtonRawChangedMs;
static uint32_t gButtonPressStartMs;
static uint32_t gFirstClickReleaseMs;

typedef enum {
    BUTTON_GESTURE_NONE = 0,
    BUTTON_GESTURE_TRIM,
    BUTTON_GESTURE_TOGGLE_DIRECTION,
    BUTTON_GESTURE_START
} ButtonGesture;

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

static bool state_accepts_gestures(BallRodState state)
{
    return (state == BALL_ROD_IDLE) ||
        (state == BALL_ROD_TRIM) ||
        (state == BALL_ROD_COMPLETE);
}

static bool state_is_sequence_running(BallRodState state)
{
    switch (state) {
        case BALL_ROD_WAKE:
        case BALL_ROD_LAUNCH:
        case BALL_ROD_LAUNCH_HOLD:
        case BALL_ROD_REVERSE:
        case BALL_ROD_RETURN_HOLD:
        case BALL_ROD_LEVEL:
        case BALL_ROD_SETTLE:
            return true;

        default:
            return false;
    }
}

static void clear_pending_gestures(void)
{
    gButtonRawPressEligible = false;
    gButtonPressEligible = false;
    gButtonLongHandled = false;
    gSecondClickCandidate = false;
    gClickPending = false;
}

static ButtonGesture button_gesture_tick(
    uint32_t nowMs, bool pressed, bool gesturesAllowed)
{
    ButtonGesture gesture = BUTTON_GESTURE_NONE;

    if (pressed != gButtonRaw) {
        gButtonRaw = pressed;
        gButtonRawChangedMs = nowMs;
        if (pressed) {
            gButtonRawPressEligible = gesturesAllowed;
        }

        /*
         * A raw second press that begins inside the double-click window keeps
         * the first click from firing while this new edge is being debounced.
         */
        if (pressed && gesturesAllowed && gClickPending &&
            ((uint32_t) (nowMs - gFirstClickReleaseMs) <=
                APP_BUTTON_DOUBLE_CLICK_MS)) {
            gSecondClickCandidate = true;
        }
    }

    if ((gButtonStable != gButtonRaw) &&
        elapsed_ms(nowMs, gButtonRawChangedMs, APP_BUTTON_DEBOUNCE_MS)) {
        gButtonStable = gButtonRaw;

        if (gButtonStable) {
            gButtonPressEligible =
                gesturesAllowed && gButtonRawPressEligible;
            gButtonLongHandled = false;
            gButtonPressStartMs = nowMs;

            if (!gClickPending) {
                gSecondClickCandidate = false;
            }
        } else {
            if (gButtonPressEligible && !gButtonLongHandled) {
                if (gSecondClickCandidate && gClickPending) {
                    gClickPending = false;
                    gSecondClickCandidate = false;
                    gesture = BUTTON_GESTURE_START;
                } else {
                    gClickPending = true;
                    gFirstClickReleaseMs = nowMs;
                }
            }
            gButtonPressEligible = false;
            gButtonLongHandled = false;
            gButtonRawPressEligible = false;
        }
    }

    if (!gesturesAllowed) {
        clear_pending_gestures();
        return BUTTON_GESTURE_NONE;
    }

    if (gButtonStable && gButtonPressEligible &&
        !gButtonLongHandled &&
        elapsed_ms(nowMs, gButtonPressStartMs,
            APP_BUTTON_LONG_PRESS_MS)) {
        gButtonLongHandled = true;
        gClickPending = false;
        gSecondClickCandidate = false;
        return BUTTON_GESTURE_TOGGLE_DIRECTION;
    }

    /*
     * If a raw second press bounced away before becoming stable, release the
     * candidate after its falling edge has itself remained stable.
     */
    if (gSecondClickCandidate && !gButtonStable && !gButtonRaw &&
        elapsed_ms(nowMs, gButtonRawChangedMs,
            APP_BUTTON_DEBOUNCE_MS)) {
        gSecondClickCandidate = false;
    }

    if ((gesture == BUTTON_GESTURE_NONE) && gClickPending &&
        !gSecondClickCandidate &&
        elapsed_ms(nowMs, gFirstClickReleaseMs,
            APP_BUTTON_DOUBLE_CLICK_MS)) {
        gClickPending = false;
        gesture = BUTTON_GESTURE_TRIM;
    }

    return gesture;
}

static void start_sequence(uint32_t nowMs)
{
    /*
     * The double-click captures the electrically trimmed physical position as
     * this run's horizontal zero.  Keep the power-on-relative step count so
     * every later target can still obey the calibrated global soft travel.
     */
    step_pin_gpio_low();
    gRunZeroSteps = gCurrentSteps;
    gTargetSteps = gCurrentSteps;
    gTelemetry.sequenceTimedOut = false;
    gRunStartMs = nowMs;
    gFinalRunMs = 0U;
    gAdjustmentUiActive = false;
    clear_pending_gestures();

    set_direction_for_sign(1);
    DL_GPIO_setPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
    gTelemetry.driverEnabled = true;
    set_state(BALL_ROD_WAKE, nowMs);
}

static void start_timeout_level(uint32_t nowMs)
{
    gTelemetry.sequenceTimedOut = true;
    set_motion_target(
        BALL_ROD_TIMEOUT_LEVEL, gRunZeroSteps, APP_OL_LEVEL_HZ, nowMs);
}

static void start_trim_move(uint32_t nowMs)
{
    step_pin_gpio_low();
    gAdjustmentUiActive = true;
    if (((gTrimDirection > 0) &&
            (gCurrentSteps >= APP_BALL_MAX_STEPS)) ||
        ((gTrimDirection < 0) &&
            (gCurrentSteps <= APP_BALL_MIN_STEPS))) {
        if (gState == BALL_ROD_COMPLETE) {
            set_state(BALL_ROD_IDLE, nowMs);
        }
        return;
    }

    set_motion_target(
        BALL_ROD_TRIM,
        gCurrentSteps + gTrimDirection * APP_TRIM_STEP_SIZE,
        APP_TRIM_HZ, nowMs);
}

static void toggle_trim_direction(uint32_t nowMs)
{
    gTrimDirection = (int8_t) -gTrimDirection;
    gAdjustmentUiActive = true;
    if (gState == BALL_ROD_COMPLETE) {
        set_state(BALL_ROD_IDLE, nowMs);
    }
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
    gRunZeroSteps = 0;
    gTrimDirection = 1;
    gAdjustmentUiActive = true;

    gButtonRaw = false;
    gButtonStable = false;
    gButtonRawPressEligible = false;
    gButtonPressEligible = false;
    gButtonLongHandled = false;
    gSecondClickCandidate = false;
    gClickPending = false;
    gButtonRawChangedMs = nowMs;
    gButtonPressStartMs = nowMs;
    gFirstClickReleaseMs = nowMs;

    gTelemetry = (BallRodTelemetry) {0};
    gTelemetry.state = BALL_ROD_IDLE;
    gTelemetry.trimDirection = gTrimDirection;
    gTelemetry.adjustmentUiActive = true;

    step_pin_gpio_low();
    DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    /*
     * Keep D36A awake from MCU startup.  With the driver's separate power
     * switch on, BLS trim moves can establish the physical horizontal point.
     * The power-on position remains the global step-count origin.
     */
    DL_GPIO_setPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
    gTelemetry.driverEnabled = true;
}

void ball_rod_tick_5ms(uint32_t nowMs, bool buttonPressed)
{
    ButtonGesture gesture = button_gesture_tick(
        nowMs, buttonPressed, state_accepts_gestures(gState));

    switch (gesture) {
        case BUTTON_GESTURE_TRIM:
            start_trim_move(nowMs);
            break;

        case BUTTON_GESTURE_TOGGLE_DIRECTION:
            toggle_trim_direction(nowMs);
            break;

        case BUTTON_GESTURE_START:
            start_sequence(nowMs);
            break;

        default:
            break;
    }

    if (state_is_sequence_running(gState) &&
        elapsed_ms(nowMs, gRunStartMs, APP_OL_RUN_TIMEOUT_MS)) {
        start_timeout_level(nowMs);
    }

    switch (gState) {
        case BALL_ROD_IDLE:
        case BALL_ROD_COMPLETE:
        case BALL_ROD_FAULT:
            step_pin_gpio_low();
            break;

        case BALL_ROD_TRIM:
            move_toward_target();
            if (target_reached()) {
                set_state(BALL_ROD_IDLE, nowMs);
            }
            break;

        case BALL_ROD_WAKE:
            if (elapsed_ms(nowMs, gStateStartMs,
                    APP_D36A_WAKE_DELAY_MS)) {
                set_motion_target(BALL_ROD_LAUNCH,
                    gRunZeroSteps + APP_OL_LAUNCH_STEPS,
                    APP_OL_LAUNCH_HZ, nowMs);
            }
            break;

        case BALL_ROD_LAUNCH:
            move_toward_target();
            if (target_reached()) {
                if (APP_OL_LAUNCH_HOLD_MS == 0U) {
                    set_motion_target(BALL_ROD_REVERSE,
                        gRunZeroSteps + APP_OL_REVERSE_STEPS,
                        APP_OL_REVERSE_HZ, nowMs);
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
                    gRunZeroSteps + APP_OL_REVERSE_STEPS,
                    APP_OL_REVERSE_HZ, nowMs);
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
                set_motion_target(BALL_ROD_LEVEL,
                    gRunZeroSteps, APP_OL_LEVEL_HZ, nowMs);
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
    gTelemetry.runZeroSteps = gRunZeroSteps;
    gTelemetry.stepFrequencyHz = gStepFrequencyHz;
    gTelemetry.trimDirection = gTrimDirection;
    gTelemetry.stepRunning = gStepRunning;
    gTelemetry.adjustmentUiActive = gAdjustmentUiActive;
    if (state_is_sequence_running(gState) ||
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
    telemetry.runZeroSteps = gRunZeroSteps;
    telemetry.stepFrequencyHz = gStepFrequencyHz;
    telemetry.trimDirection = gTrimDirection;
    telemetry.stepRunning = gStepRunning;
    telemetry.adjustmentUiActive = gAdjustmentUiActive;
    if (primask == 0U) {
        __enable_irq();
    }
    return telemetry;
}
