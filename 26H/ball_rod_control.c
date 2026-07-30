#include "ball_rod_control.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

#include <limits.h>

#define BALL_VISION_FLAG_PREDICTED  (0x02U)

static volatile int32_t gCurrentSteps;
static volatile int32_t gTargetSteps;
static volatile bool gStepRunning;
static volatile int8_t gStepSign;

static BallRodTelemetry gTelemetry;
static uint16_t gStepFrequencyHz;
static uint16_t gCommandFrequencyHz;
static uint16_t gLastVisionSequence;
static bool gHaveVisionSequence;
static bool gPositionPidInitialized;
static int32_t gPositionPidLastError;
static int32_t gPositionPidFilteredDerivative;
static uint32_t gPositionPidLastMs;
static uint32_t gEnableStartMs;
static bool gDriverEnabled;
static BallMotionPhase gMotionPhase;
static uint32_t gMotionStartMs;
static bool gMotionTimerStarted;
static bool gSequenceTimedOut;
static uint32_t gReturnToLevelStartMs;
static int32_t gCenterFilteredVelocity;
static int32_t gCenterReferenceSteps;
static int32_t gCenterBiasSteps;
static uint32_t gCenterLastErrorMagnitudeQ4;
static uint32_t gCenterRecoveryStartMs;
static uint32_t gCenterLastForcedActionMs;
static uint8_t gCenterStableFrames;
static uint8_t gCenterNoProgressFrames;
static uint8_t gCenterArmFrames;
static uint8_t gCenterRecoveryPhase;
static int8_t gCenterRecoverySign;
static int8_t gCenterLastCorrectionSign;
static uint16_t gCenterTiltLimit;
static uint16_t gCenterRunId;
static uint16_t gEventCounter;
static uint8_t gLastEvent;
static bool gCenterSettled;
static bool gCenterCandidate;
static bool gCenterMustCorrect;
static bool gCenterApproaching;
static bool gCenterHaveRealError;
static bool gCenterHaveCorrectionSign;
static bool gCenterForcedActionSeen;
static bool gCenterFineMode;
static bool gCenterForceCooldown;
static bool gCenterRecoveryRealFramePermit;
static bool gCenterAtLimit;
static bool gVisionFaultReported;

static bool gButtonRaw;
static bool gButtonStable;
static bool gButtonEmergencyHandled;
static uint32_t gButtonRawChangedMs;
static uint32_t gButtonPressedMs;
static bool gCalibrationPositiveDirection = true;
static bool gClickPending;
static uint32_t gFirstClickMs;

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

static uint32_t absolute_i32(int32_t value)
{
    return (uint32_t) ((value < 0) ? -value : value);
}

static bool elapsed_ms(uint32_t nowMs, uint32_t startMs, uint32_t durationMs)
{
    return (uint32_t) (nowMs - startMs) >= durationMs;
}

static void step_pin_gpio_low(void)
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

static void reset_position_pid(uint32_t nowMs)
{
    gPositionPidInitialized = false;
    gPositionPidLastError = 0;
    gPositionPidFilteredDerivative = 0;
    gPositionPidLastMs = nowMs;
}

static void reset_center_hold_state(void)
{
    gCenterFilteredVelocity = 0;
    gCenterReferenceSteps = gCurrentSteps;
    gCenterBiasSteps = 0;
    gCenterLastErrorMagnitudeQ4 = 0U;
    gCenterRecoveryStartMs = 0U;
    gCenterLastForcedActionMs = 0U;
    gCenterStableFrames = 0U;
    gCenterNoProgressFrames = 0U;
    gCenterArmFrames = 0U;
    gCenterRecoveryPhase = BALL_CENTER_RECOVERY_NONE;
    gCenterRecoverySign = 0;
    gCenterLastCorrectionSign = 0;
    gCenterTiltLimit = APP_BALL_CENTER_INITIAL_TILT_STEPS;
    gCenterSettled = false;
    gCenterCandidate = false;
    gCenterMustCorrect = false;
    gCenterApproaching = false;
    gCenterHaveRealError = false;
    gCenterHaveCorrectionSign = false;
    gCenterForcedActionSeen = false;
    gCenterFineMode = false;
    gCenterForceCooldown = false;
    gCenterRecoveryRealFramePermit = false;
    gCenterAtLimit = false;
    gVisionFaultReported = false;
}

static void publish_status_event(BallStatusEvent event)
{
    gLastEvent = (uint8_t) event;
    gEventCounter++;
}

static void stop_and_hold_current_position(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    step_pin_gpio_low();
    gTargetSteps = gCurrentSteps;
    if (primask == 0U) {
        __enable_irq();
    }
}

static void set_direction_for_sign(int8_t sign)
{
    bool high = sign > 0;

    if (APP_BALL_DIR_INVERT != 0U) {
        high = !high;
    }
    step_pin_gpio_low();
    if (high) {
        DL_GPIO_setPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    }
    gStepSign = sign;
    gTelemetry.directionLevel = high ? 1U : 0U;
}

static void configure_and_start_step(uint16_t frequencyHz, int8_t sign)
{
    uint32_t periodTicks;

    if (frequencyHz < APP_BALL_STEP_MIN_HZ) {
        frequencyHz = APP_BALL_STEP_MIN_HZ;
    } else if (frequencyHz > APP_BALL_STEP_MAX_HZ) {
        frequencyHz = APP_BALL_STEP_MAX_HZ;
    }
    if (gStepRunning && (sign == gStepSign) &&
        (frequencyHz == gStepFrequencyHz)) {
        return;
    }
    if ((!gStepRunning) || (sign != gStepSign)) {
        set_direction_for_sign(sign);
    } else {
        DL_TimerG_stopCounter(PWM_BALL_STEP_INST);
    }

    periodTicks = APP_BALL_STEP_CLOCK_HZ / frequencyHz;
    DL_TimerG_setLoadValue(PWM_BALL_STEP_INST, periodTicks);
    DL_TimerG_setCaptureCompareValue(PWM_BALL_STEP_INST,
        periodTicks / 2U, DL_TIMER_CC_1_INDEX);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_BALL_STEP_C1_IOMUX,
        GPIO_PWM_BALL_STEP_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(
        GPIO_PWM_BALL_STEP_C1_PORT, GPIO_PWM_BALL_STEP_C1_PIN);
    DL_TimerG_startCounter(PWM_BALL_STEP_INST);
    gStepFrequencyHz = frequencyHz;
    gStepRunning = true;
}

static void move_toward_target(void)
{
    int32_t distance = gTargetSteps - gCurrentSteps;
    uint32_t magnitude = absolute_i32(distance);
    uint32_t requested;
    uint16_t nextFrequency;
    int8_t sign;

#if APP_BALL_CALIBRATION_MODE
    /*
     * Calibration jogs must finish on the requested multiple of 16.  The
     * normal controller's two-step settling tolerance would otherwise make
     * each button press stop at 14 steps and corrupt the measured limits.
     */
    if (magnitude == 0U) {
#else
    if (magnitude <= APP_BALL_POSITION_TOLERANCE_STEPS) {
#endif
        step_pin_gpio_low();
        return;
    }
    sign = (distance > 0) ? 1 : -1;
#if APP_BALL_CALIBRATION_MODE
    requested = APP_BALL_STEP_MIN_HZ +
        magnitude * APP_BALL_STEP_HZ_PER_ERROR;
#else
    requested = gCommandFrequencyHz;
#endif
    if (requested > APP_BALL_STEP_MAX_HZ) {
        requested = APP_BALL_STEP_MAX_HZ;
    }

    if (!gStepRunning || (sign != gStepSign)) {
        nextFrequency = APP_BALL_STEP_MIN_HZ;
    } else if (requested >
        (uint32_t) gStepFrequencyHz + APP_BALL_STEP_HZ_SLEW_PER_TICK) {
        nextFrequency =
            gStepFrequencyHz + APP_BALL_STEP_HZ_SLEW_PER_TICK;
    } else if (gStepFrequencyHz >
        requested + APP_BALL_STEP_HZ_SLEW_PER_TICK) {
        nextFrequency =
            gStepFrequencyHz - APP_BALL_STEP_HZ_SLEW_PER_TICK;
    } else {
        nextFrequency = (uint16_t) requested;
    }
    configure_and_start_step(nextFrequency, sign);
}

#if !APP_BALL_CALIBRATION_MODE
static void select_center_mode(uint32_t nowMs)
{
    stop_and_hold_current_position();
    reset_position_pid(nowMs);
    reset_center_hold_state();
    gMotionPhase = BALL_MOTION_HOLD_CENTER;
    gMotionTimerStarted = false;
    gSequenceTimedOut = false;
    gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
    gCenterLastForcedActionMs =
        nowMs - APP_BALL_CENTER_FORCED_ACTION_MS;
    gCenterRunId++;
    publish_status_event(BALL_STATUS_EVENT_CENTER_START);
}

static void start_motion_sequence(uint32_t nowMs)
{
    if (gMotionPhase == BALL_MOTION_HOLD_CENTER) {
        publish_status_event(BALL_STATUS_EVENT_DOUBLE_CLICK_OVERRIDE);
    }
    stop_and_hold_current_position();
    reset_position_pid(nowMs);
    reset_center_hold_state();
    gMotionPhase = BALL_MOTION_CENTER_BEFORE_SEQUENCE;
    gMotionTimerStarted = false;
    gSequenceTimedOut = false;
    gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
}

static void service_operation_button(uint32_t nowMs, bool pressed)
{
    if (gClickPending &&
        elapsed_ms(nowMs, gFirstClickMs, APP_BALL_DOUBLE_CLICK_MS)) {
        gClickPending = false;
    }

    if (pressed != gButtonRaw) {
        gButtonRaw = pressed;
        gButtonRawChangedMs = nowMs;
    }
    if ((gButtonStable == gButtonRaw) ||
        !elapsed_ms(nowMs, gButtonRawChangedMs, APP_BUTTON_DEBOUNCE_MS)) {
        return;
    }

    gButtonStable = gButtonRaw;
    if (gButtonStable) {
        gButtonPressedMs = nowMs;
        return;
    }

    if (!elapsed_ms(nowMs, gButtonPressedMs, APP_BALL_BUTTON_LONG_MS)) {
        if (gClickPending &&
            !elapsed_ms(nowMs, gFirstClickMs, APP_BALL_DOUBLE_CLICK_MS)) {
            gClickPending = false;
            start_motion_sequence(nowMs);
        } else {
            /*
             * A first click takes effect immediately for safety.  Keep a
             * one-second pending flag so a second click can override center
             * hold and launch the complete motion sequence.
             */
            select_center_mode(nowMs);
            gClickPending = true;
            gFirstClickMs = nowMs;
        }
    }
}

static int32_t center_absolute_target(int32_t relativeTarget)
{
    int32_t limitedRelative = clamp_i32(relativeTarget,
        -(int32_t) gCenterTiltLimit, (int32_t) gCenterTiltLimit);
    int32_t requested = gCenterReferenceSteps + limitedRelative;
    int32_t limited = clamp_i32(
        requested, APP_BALL_MIN_STEPS, APP_BALL_MAX_STEPS);

    gCenterAtLimit = (limited != requested);
    return limited;
}

static void command_center_relative(int32_t relativeTarget, bool immediate)
{
    int32_t desiredTarget = center_absolute_target(relativeTarget);

    if (immediate) {
        gTargetSteps = desiredTarget;
    } else {
        int32_t targetChange = clamp_i32(desiredTarget - gTargetSteps,
            -APP_BALL_TARGET_SLEW_STEPS_PER_FRAME,
            APP_BALL_TARGET_SLEW_STEPS_PER_FRAME);

        gTargetSteps = clamp_i32(gTargetSteps + targetChange,
            APP_BALL_MIN_STEPS, APP_BALL_MAX_STEPS);
    }
}

static void begin_center_recovery(uint32_t nowMs, int8_t sign)
{
    gCenterRecoveryPhase = BALL_CENTER_RECOVERY_BACKOFF;
    gCenterRecoverySign = sign;
    gCenterRecoveryStartMs = nowMs;
    gCenterLastForcedActionMs = nowMs;
    gCenterForcedActionSeen = true;
    command_center_relative(sign *
        ((int32_t) gCenterTiltLimit -
            APP_BALL_CENTER_RECOVERY_BACKOFF_STEPS), true);
    publish_status_event(BALL_STATUS_EVENT_RECOVERY_BACKOFF);
}

static void cancel_center_recovery(void)
{
    gCenterRecoveryPhase = BALL_CENTER_RECOVERY_NONE;
    gCenterRecoverySign = 0;
    gCenterForceCooldown = false;
}

static void service_center_recovery(uint32_t nowMs)
{
    bool rodAtTarget;

    if (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_NONE) {
        return;
    }
    if (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_LEVELING) {
        gCommandFrequencyHz = APP_BALL_CENTER_LEVEL_HZ;
        command_center_relative(0, true);
        if (absolute_i32(gCurrentSteps - gCenterReferenceSteps) <=
            APP_BALL_POSITION_TOLERANCE_STEPS) {
            gCenterRecoveryPhase = BALL_CENTER_RECOVERY_NONE;
            gCenterRecoverySign = 0;
            gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
        }
        return;
    }
    if (!gCenterRecoveryRealFramePermit) {
        return;
    }
    gCenterRecoveryRealFramePermit = false;
    rodAtTarget =
        absolute_i32(gTargetSteps - gCurrentSteps) <=
        APP_BALL_POSITION_TOLERANCE_STEPS;
    if (!rodAtTarget ||
        !elapsed_ms(nowMs, gCenterRecoveryStartMs,
            APP_BALL_CENTER_RECOVERY_INTERVAL_MS)) {
        return;
    }

    gCenterRecoveryStartMs = nowMs;
    gCenterLastForcedActionMs = nowMs;
    if (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_BACKOFF) {
        gCenterRecoveryPhase = BALL_CENTER_RECOVERY_REAPPLY;
        command_center_relative(
            gCenterRecoverySign * (int32_t) gCenterTiltLimit, true);
        publish_status_event(BALL_STATUS_EVENT_RECOVERY_REAPPLY);
    } else {
        gCenterRecoveryPhase = BALL_CENTER_RECOVERY_BACKOFF;
        command_center_relative(gCenterRecoverySign *
            ((int32_t) gCenterTiltLimit -
                APP_BALL_CENTER_RECOVERY_BACKOFF_STEPS), true);
        publish_status_event(BALL_STATUS_EVENT_RECOVERY_BACKOFF);
    }
}

static void update_center_hold_target(
    const BallVisionSample *vision, uint32_t nowMs)
{
    int32_t errorQ4 =
        APP_BALL_IMAGE_CENTER_Q4 - (int32_t) vision->xQ4;
    int32_t x = (int32_t) ((vision->xQ4 + 8U) >> 4U);
    uint32_t errorMagnitudeQ4 = absolute_i32(errorQ4);
    uint32_t errorMagnitudePx = errorMagnitudeQ4 >> 4U;
    bool predicted =
        (vision->flags & BALL_VISION_FLAG_PREDICTED) != 0U;
    bool realFrame = !predicted;
    bool rodAtTarget =
        absolute_i32(gTargetSteps - gCurrentSteps) <=
        APP_BALL_POSITION_TOLERANCE_STEPS;
    bool positionApproaching = false;
    bool velocityApproaching;
    int32_t proportional;
    int32_t damping;
    int32_t desiredRelative;
    int8_t correctionSign = (errorQ4 >= 0) ? 1 : -1;
    bool crossedCenter = false;

    if (realFrame) {
        gCenterRecoveryRealFramePermit = true;
    }
    gCenterFilteredVelocity +=
        ((int32_t) vision->velocityX - gCenterFilteredVelocity) /
        APP_BALL_CENTER_VELOCITY_FILTER_DIVISOR;
    velocityApproaching =
        (absolute_i32(gCenterFilteredVelocity) >=
            APP_BALL_CENTER_SETTLE_SPEED_PX_S) &&
        (((errorQ4 > 0) && (gCenterFilteredVelocity > 0)) ||
         ((errorQ4 < 0) && (gCenterFilteredVelocity < 0)));
    if (realFrame && gCenterHaveRealError &&
        (errorMagnitudeQ4 + APP_BALL_CENTER_PROGRESS_Q4 <=
            gCenterLastErrorMagnitudeQ4)) {
        positionApproaching = true;
    }
    gCenterApproaching =
        realFrame && (positionApproaching || velocityApproaching);

    /*
     * Only a real detector frame can enter or leave the settled state.
     * |errorQ4|=60 may continue holding, while 61 immediately resumes
     * correction.  Predicted frames never advance arming, settling, or stuck
     * recovery counters.
     */
    if (realFrame) {
        if (errorMagnitudeQ4 >= APP_BALL_CENTER_MUST_CORRECT_Q4) {
            crossedCenter = gCenterHaveCorrectionSign &&
                (correctionSign != gCenterLastCorrectionSign);
            gCenterLastCorrectionSign = correctionSign;
            gCenterHaveCorrectionSign = true;
            if ((gCenterSettled || gCenterCandidate) &&
                !crossedCenter) {
                publish_status_event(
                    BALL_STATUS_EVENT_CORRECTION_RESUME);
            }
            gCenterSettled = false;
            gCenterCandidate = false;
            gCenterStableFrames = 0U;
            gCenterMustCorrect = true;
        } else {
            gCenterMustCorrect = false;
            gCenterNoProgressFrames = 0U;
            if (gCenterRecoveryPhase !=
                BALL_CENTER_RECOVERY_LEVELING) {
                cancel_center_recovery();
            }
        }

        if (errorMagnitudeQ4 <= APP_BALL_CENTER_SETTLE_ERROR_Q4) {
            if (!gCenterCandidate) {
                stop_and_hold_current_position();
                gCenterCandidate = true;
                gCenterStableFrames = 0U;
            }
            if (gCenterStableFrames <
                APP_BALL_CENTER_STABLE_REAL_FRAMES) {
                gCenterStableFrames++;
            }
            gCenterNoProgressFrames = 0U;
            cancel_center_recovery();
            if (!gCenterSettled &&
                (gCenterStableFrames >=
                    APP_BALL_CENTER_STABLE_REAL_FRAMES)) {
                gCenterSettled = true;
                gCenterReferenceSteps = gCurrentSteps;
                gCenterBiasSteps = 0;
                gCenterFineMode = true;
                gCenterTiltLimit = APP_BALL_CENTER_FINE_TILT_STEPS;
                publish_status_event(
                    BALL_STATUS_EVENT_CENTER_SETTLED);
            }
        } else {
            gCenterCandidate = false;
            gCenterStableFrames = 0U;
        }
    }

    if (gCenterSettled && !gCenterMustCorrect) {
        stop_and_hold_current_position();
        gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
        goto center_sample_done;
    }
    if (gCenterCandidate) {
        stop_and_hold_current_position();
        gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
        goto center_sample_done;
    }

    /*
     * A real sign reversal means the ball crossed the red line.  Discard all
     * learned force from the old side and request the new braking direction
     * immediately instead of slewing through a stale +/-48..120-step target.
     */
    if (realFrame && crossedCenter) {
        cancel_center_recovery();
        gCenterBiasSteps = 0;
        gCenterNoProgressFrames = 0U;
        gCenterFineMode =
            errorMagnitudeQ4 <= APP_BALL_CENTER_FINE_ZONE_Q4;
        gCenterTiltLimit = gCenterFineMode ?
            APP_BALL_CENTER_FINE_TILT_STEPS :
            APP_BALL_CENTER_INITIAL_TILT_STEPS;
        gCommandFrequencyHz = APP_BALL_CENTER_CROSS_HZ;
        command_center_relative(
            correctionSign * APP_BALL_CENTER_CROSS_TILT_STEPS, true);
        gCenterLastForcedActionMs = nowMs;
        gCenterForcedActionSeen = true;
        gCenterForceCooldown = true;
        publish_status_event(BALL_STATUS_EVENT_CORRECTION_RESUME);
        goto center_sample_done;
    }

    /*
     * Once a distant ball reaches the conservative fine zone while moving
     * toward the line, remove the learned tilt and level the rod first.
     * The physical rod must reach the button-time reference before the
     * low-speed +/-24-step fine controller is allowed to resume.
     */
    if (realFrame && !gCenterFineMode &&
        (errorMagnitudeQ4 <= APP_BALL_CENTER_FINE_ZONE_Q4) &&
        gCenterApproaching) {
        cancel_center_recovery();
        gCenterFineMode = true;
        gCenterTiltLimit = APP_BALL_CENTER_FINE_TILT_STEPS;
        gCenterBiasSteps = 0;
        gCenterNoProgressFrames = 0U;
        gCenterRecoveryPhase = BALL_CENTER_RECOVERY_LEVELING;
        gCenterRecoveryStartMs = nowMs;
        gCommandFrequencyHz = APP_BALL_CENTER_LEVEL_HZ;
        command_center_relative(0, true);
        publish_status_event(BALL_STATUS_EVENT_CENTER_LEVELING);
        goto center_sample_done;
    }

    if (realFrame && gCenterFineMode &&
        (errorMagnitudeQ4 > APP_BALL_CENTER_FINE_ZONE_Q4)) {
        cancel_center_recovery();
        gCenterFineMode = false;
        gCenterTiltLimit = APP_BALL_CENTER_INITIAL_TILT_STEPS;
        gCenterBiasSteps = 0;
        gCenterNoProgressFrames = 0U;
        gCenterForcedActionSeen = false;
    }

    if (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_LEVELING) {
        gCommandFrequencyHz = APP_BALL_CENTER_LEVEL_HZ;
        command_center_relative(0, true);
        goto center_sample_done;
    }

    if (gCenterApproaching) {
        gCenterNoProgressFrames = 0U;
        if (gCenterBiasSteps > 0) {
            gCenterBiasSteps -= APP_BALL_CENTER_BIAS_DECAY_STEPS;
            if (gCenterBiasSteps < 0) {
                gCenterBiasSteps = 0;
            }
        } else if (gCenterBiasSteps < 0) {
            gCenterBiasSteps += APP_BALL_CENTER_BIAS_DECAY_STEPS;
            if (gCenterBiasSteps > 0) {
                gCenterBiasSteps = 0;
            }
        }
        if ((gCenterRecoveryPhase == BALL_CENTER_RECOVERY_BACKOFF) ||
            (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_REAPPLY)) {
            cancel_center_recovery();
        }
    } else if (realFrame && gCenterMustCorrect && rodAtTarget) {
        if (gCenterNoProgressFrames <
            APP_BALL_CENTER_NO_PROGRESS_FRAMES) {
            gCenterNoProgressFrames++;
        }
    } else if (realFrame) {
        gCenterNoProgressFrames = 0U;
    }
    if (gCenterForceCooldown &&
        elapsed_ms(nowMs, gCenterLastForcedActionMs,
            APP_BALL_CENTER_FORCED_ACTION_MS)) {
        gCenterForceCooldown = false;
    }

    if (gCenterFineMode) {
        gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
    } else if (errorMagnitudePx <= APP_BALL_SPEED_NEAR_ERROR_PX) {
        gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
    } else if (errorMagnitudePx <= APP_BALL_SPEED_MEDIUM_ERROR_PX) {
        gCommandFrequencyHz = APP_BALL_SPEED_MEDIUM_HZ;
    } else {
        gCommandFrequencyHz = APP_BALL_SPEED_FAR_HZ;
    }

    proportional =
        ((errorQ4 * APP_BALL_POSITION_KP_NUMERATOR) /
            APP_BALL_POSITION_KP_DIVISOR) / 16;
    damping =
        (-gCenterFilteredVelocity *
            APP_BALL_POSITION_KD_NUMERATOR) /
        APP_BALL_POSITION_KD_DIVISOR;
    desiredRelative = proportional + damping + gCenterBiasSteps;
    if (gCenterMustCorrect &&
        ((desiredRelative * correctionSign) <
            APP_BALL_MIN_EFFECTIVE_TILT_STEPS)) {
        desiredRelative =
            correctionSign * APP_BALL_MIN_EFFECTIVE_TILT_STEPS;
    }

    if (realFrame && gCenterMustCorrect &&
        (gCenterNoProgressFrames >=
            APP_BALL_CENTER_NO_PROGRESS_FRAMES) &&
        (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_NONE) &&
        (!gCenterForcedActionSeen ||
         elapsed_ms(nowMs, gCenterLastForcedActionMs,
             APP_BALL_CENTER_FORCED_ACTION_MS))) {
        bool alreadyAtMaximum;
        int32_t relativeCurrent =
            gCurrentSteps - gCenterReferenceSteps;

        gCenterNoProgressFrames = 0U;
        if (!gCenterFineMode &&
            (gCenterTiltLimit < APP_BALL_CENTER_MAX_TILT_STEPS)) {
            gCenterTiltLimit +=
                APP_BALL_CENTER_TILT_INCREMENT_STEPS;
            if (gCenterTiltLimit >
                APP_BALL_CENTER_MAX_TILT_STEPS) {
                gCenterTiltLimit =
                    APP_BALL_CENTER_MAX_TILT_STEPS;
            }
        }
        alreadyAtMaximum =
            absolute_i32(relativeCurrent) >=
                (gCenterTiltLimit -
                    APP_BALL_POSITION_TOLERANCE_STEPS);

        if (alreadyAtMaximum &&
            (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_NONE)) {
            begin_center_recovery(nowMs, correctionSign);
        } else {
            gCenterBiasSteps = clamp_i32(
                gCenterBiasSteps +
                    correctionSign * APP_BALL_CENTER_NUDGE_STEPS,
                -(int32_t) gCenterTiltLimit,
                (int32_t) gCenterTiltLimit);
            /*
             * This direct four-microstep request is the hard guarantee that a
             * stationary >=3 mm error cannot leave target==current.
             */
            command_center_relative(relativeCurrent +
                correctionSign * APP_BALL_CENTER_NUDGE_STEPS, true);
            gCenterLastForcedActionMs = nowMs;
            gCenterForcedActionSeen = true;
            gCenterForceCooldown = true;
            publish_status_event(
                BALL_STATUS_EVENT_RECOVERY_REAPPLY);
        }
    } else if (gCenterRecoveryPhase == BALL_CENTER_RECOVERY_NONE) {
        command_center_relative(desiredRelative, false);
    }

center_sample_done:
    if (realFrame) {
        gCenterLastErrorMagnitudeQ4 = errorMagnitudeQ4;
        gCenterHaveRealError = true;
    }
    gTelemetry.ballX = (int16_t) x;
    gTelemetry.ballError =
        (int16_t) (errorQ4 / 16);
    gTelemetry.ballErrorQ4 = (int16_t) errorQ4;
}

static void service_return_to_level(uint32_t nowMs)
{
    if (gMotionPhase != BALL_MOTION_RETURN_TO_LEVEL) {
        return;
    }
    gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;

    if (gTargetSteps > APP_BALL_LEVEL_SLEW_STEPS_PER_TICK) {
        gTargetSteps -= APP_BALL_LEVEL_SLEW_STEPS_PER_TICK;
    } else if (gTargetSteps < -APP_BALL_LEVEL_SLEW_STEPS_PER_TICK) {
        gTargetSteps += APP_BALL_LEVEL_SLEW_STEPS_PER_TICK;
    } else {
        gTargetSteps = 0;
    }

    /*
     * The requested angle reaches zero over about 150 ms.  Do not start the
     * negative leg until the physical step position has also caught up, so
     * the target cannot command an abrupt reversal while the rod is tilted.
     */
    if (elapsed_ms(nowMs, gReturnToLevelStartMs,
            APP_BALL_REVERSAL_LEVEL_MS) &&
        (absolute_i32(gTargetSteps) <=
            APP_BALL_POSITION_TOLERANCE_STEPS) &&
        (absolute_i32(gCurrentSteps) <=
            APP_BALL_POSITION_TOLERANCE_STEPS)) {
        stop_and_hold_current_position();
        reset_position_pid(nowMs);
        gMotionPhase = BALL_MOTION_TO_NEGATIVE_5CM;
    }
}

static void update_pid_target(const BallVisionSample *vision)
{
    int32_t x = (int32_t) ((vision->xQ4 + 8U) >> 4U);
    bool centerMode =
        gMotionPhase == BALL_MOTION_CENTER_BEFORE_SEQUENCE;
    int32_t imageTarget = centerMode ? APP_BALL_IMAGE_CENTER_X :
        ((gMotionPhase == BALL_MOTION_TO_POSITIVE_5CM) ?
            APP_BALL_POSITIVE_5CM_X : APP_BALL_NEGATIVE_5CM_X);
    int32_t tolerance = centerMode ? APP_BALL_CENTER_DEADBAND_PX :
        ((gMotionPhase == BALL_MOTION_TO_POSITIVE_5CM) ?
            APP_BALL_POSITIVE_TOLERANCE_PX :
            APP_BALL_FINAL_TOLERANCE_PX);
    int32_t error = imageTarget - x;
    int32_t proportional;
    int32_t derivative;
    int32_t rawDerivative = 0;
    int32_t desiredTarget;
    int32_t targetChange;
    uint32_t errorMagnitude = absolute_i32(error);

    if ((error >= -tolerance) && (error <= tolerance)) {
        stop_and_hold_current_position();
        reset_position_pid(vision->lastFrameMs);
        gTelemetry.ballX = (int16_t) x;
        gTelemetry.ballError = 0;

        if (gMotionPhase == BALL_MOTION_CENTER_BEFORE_SEQUENCE) {
            /*
             * A double-click sequence always establishes O first.  Start the
             * competition trajectory and its timer only after x=160 +/-5
             * has actually been reached.
             */
            gMotionPhase = BALL_MOTION_TO_POSITIVE_5CM;
            gMotionStartMs = vision->lastFrameMs;
            gMotionTimerStarted = gDriverEnabled;
        } else if (gMotionPhase == BALL_MOTION_TO_POSITIVE_5CM) {
            /*
             * The positive point has genuinely been reached.  Freeze the
             * current rod angle for this frame, then command -5 cm from the
             * next vision frame without carrying a derivative kick across
             * the reversal.
             */
            gMotionPhase = BALL_MOTION_RETURN_TO_LEVEL;
            gReturnToLevelStartMs = vision->lastFrameMs;
        } else if (!centerMode) {
            /*
             * At -5 cm, stop STEP immediately but leave PA24 high.  If the
             * ball later leaves the +/-1 cm band, the same PD loop resumes.
             */
            gMotionPhase = BALL_MOTION_HOLD_NEGATIVE_5CM;
        }
        gTelemetry.motionPhase = gMotionPhase;
        return;
    }

    if (errorMagnitude <= APP_BALL_SPEED_NEAR_ERROR_PX) {
        gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
    } else if (errorMagnitude <= APP_BALL_SPEED_MEDIUM_ERROR_PX) {
        gCommandFrequencyHz = APP_BALL_SPEED_MEDIUM_HZ;
    } else {
        gCommandFrequencyHz = APP_BALL_SPEED_FAR_HZ;
    }

    /*
     * Match the li position loop: derivative comes from error change over the
     * real outer-loop interval and is low-pass filtered before use.
     */
    if (gPositionPidInitialized) {
        uint32_t dtMs = (uint32_t)
            (vision->lastFrameMs - gPositionPidLastMs);

        if (dtMs < APP_BALL_OUTER_DT_MIN_MS) {
            dtMs = APP_BALL_OUTER_DT_MIN_MS;
        } else if (dtMs > APP_BALL_OUTER_DT_MAX_MS) {
            dtMs = APP_BALL_OUTER_DT_MAX_MS;
        }
        rawDerivative =
            ((error - gPositionPidLastError) * 1000) / (int32_t) dtMs;
        gPositionPidFilteredDerivative +=
            (rawDerivative - gPositionPidFilteredDerivative) /
            APP_BALL_D_FILTER_DIVISOR;
    } else {
        gPositionPidInitialized = true;
        gPositionPidFilteredDerivative = 0;
    }
    gPositionPidLastError = error;
    gPositionPidLastMs = vision->lastFrameMs;

    proportional = (error * APP_BALL_POSITION_KP_NUMERATOR) /
        APP_BALL_POSITION_KP_DIVISOR;
    derivative = (gPositionPidFilteredDerivative *
        APP_BALL_POSITION_KD_NUMERATOR) /
        APP_BALL_POSITION_KD_DIVISOR;
    desiredTarget = proportional + derivative;
    if (absolute_i32(desiredTarget) < APP_BALL_MIN_EFFECTIVE_TILT_STEPS) {
        desiredTarget = (error > 0) ?
            APP_BALL_MIN_EFFECTIVE_TILT_STEPS :
            -APP_BALL_MIN_EFFECTIVE_TILT_STEPS;
    }

    desiredTarget = clamp_i32(desiredTarget,
        -APP_BALL_ANGLE_TARGET_LIMIT_STEPS,
        APP_BALL_ANGLE_TARGET_LIMIT_STEPS);
    desiredTarget = clamp_i32(desiredTarget,
        APP_BALL_MIN_STEPS, APP_BALL_MAX_STEPS);

    /*
     * Slew the requested rod angle at the 30 Hz vision-frame rate.  This is
     * the no-angle-sensor equivalent of the li example's fast inner loop and
     * prevents noisy coordinates from commanding an instant large tilt.
     */
    targetChange = clamp_i32(desiredTarget - gTargetSteps,
        -APP_BALL_TARGET_SLEW_STEPS_PER_FRAME,
        APP_BALL_TARGET_SLEW_STEPS_PER_FRAME);
    gTargetSteps = clamp_i32(gTargetSteps + targetChange,
        APP_BALL_MIN_STEPS, APP_BALL_MAX_STEPS);
    gTelemetry.ballX = (int16_t) x;
    gTelemetry.ballError = (int16_t) error;
}
#endif

#if APP_BALL_CALIBRATION_MODE
static void service_calibration_button(uint32_t nowMs, bool pressed)
{
    if (pressed != gButtonRaw) {
        gButtonRaw = pressed;
        gButtonRawChangedMs = nowMs;
    }
    if ((gButtonStable != gButtonRaw) &&
        elapsed_ms(nowMs, gButtonRawChangedMs, APP_BUTTON_DEBOUNCE_MS)) {
        gButtonStable = gButtonRaw;
        if (gButtonStable) {
            gButtonPressedMs = nowMs;
            gButtonEmergencyHandled = false;
        } else if (!gButtonEmergencyHandled) {
            uint32_t heldMs = (uint32_t) (nowMs - gButtonPressedMs);

            if (heldMs >= APP_BALL_BUTTON_LONG_MS) {
                gCalibrationPositiveDirection =
                    !gCalibrationPositiveDirection;
                set_direction_for_sign(
                    gCalibrationPositiveDirection ? 1 : -1);
            } else {
                int32_t target = gCurrentSteps +
                    (gCalibrationPositiveDirection ?
                        APP_BALL_CALIBRATION_JOG_STEPS :
                        -APP_BALL_CALIBRATION_JOG_STEPS);

                gTargetSteps = clamp_i32(target,
                    -APP_BALL_CALIBRATION_HARD_LIMIT,
                    APP_BALL_CALIBRATION_HARD_LIMIT);
            }
        }
    }
    if (gButtonStable && !gButtonEmergencyHandled &&
        elapsed_ms(nowMs, gButtonPressedMs, APP_BALL_BUTTON_ESTOP_MS)) {
        gButtonEmergencyHandled = true;
        gTargetSteps = gCurrentSteps;
        step_pin_gpio_low();
        gTelemetry.state = BALL_ROD_SAFETY_FAULT;
    }
}
#endif

void ball_rod_init(uint32_t nowMs)
{
    gCurrentSteps = 0;
    gTargetSteps = 0;
    gStepRunning = false;
    gStepSign = 1;
    gStepFrequencyHz = 0U;
    gCommandFrequencyHz = APP_BALL_SPEED_NEAR_HZ;
    gLastVisionSequence = 0U;
    gHaveVisionSequence = false;
    gPositionPidInitialized = false;
    gPositionPidLastError = 0;
    gPositionPidFilteredDerivative = 0;
    gPositionPidLastMs = nowMs;
    gCenterRunId = 0U;
    gEventCounter = 0U;
    gLastEvent = BALL_STATUS_EVENT_NONE;
    reset_center_hold_state();
    gButtonRaw = false;
    gButtonStable = false;
    gButtonEmergencyHandled = false;
    gButtonRawChangedMs = nowMs;
    gButtonPressedMs = nowMs;
    gCalibrationPositiveDirection = true;
    gClickPending = false;
    gFirstClickMs = nowMs;
    gTelemetry = (BallRodTelemetry) {0};
    gTelemetry.minimumReached = 0;
    gTelemetry.maximumReached = 0;

    step_pin_gpio_low();
    DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    DL_GPIO_clearPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
    gDriverEnabled = false;
    gMotionPhase = BALL_MOTION_IDLE;
    gMotionStartMs = nowMs;
    gMotionTimerStarted = false;
    gSequenceTimedOut = false;
    gReturnToLevelStartMs = nowMs;
    gTelemetry.motionPhase = gMotionPhase;

#if APP_BALL_CALIBRATION_MODE
    set_direction_for_sign(1);
    DL_GPIO_setPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
    gDriverEnabled = true;
    gEnableStartMs = nowMs;
    gTelemetry.state = BALL_ROD_CALIBRATION;
#else
    gEnableStartMs = nowMs;
    gTelemetry.state = BALL_ROD_WAITING_VISION;
#endif
}

void ball_rod_tick_5ms(
    uint32_t nowMs, bool buttonPressed, const BallVisionSample *vision)
{
    bool predicted =
        (vision->flags & BALL_VISION_FLAG_PREDICTED) != 0U;
    bool newVisionFrame = vision->received &&
        (!gHaveVisionSequence ||
            (vision->sequence != gLastVisionSequence));
    bool centerMode = gMotionPhase == BALL_MOTION_HOLD_CENTER;

    gTelemetry.crcErrors = vision->crcErrors;
    gTelemetry.sequenceDrops = vision->sequenceDrops;
    gTelemetry.rxOverflows = vision->rxOverflows;

#if APP_BALL_CALIBRATION_MODE
    service_calibration_button(nowMs, buttonPressed);
    if (gTelemetry.state != BALL_ROD_SAFETY_FAULT) {
        gTelemetry.state = BALL_ROD_CALIBRATION;
        if (gDriverEnabled &&
            elapsed_ms(nowMs, gEnableStartMs, APP_D36A_WAKE_DELAY_MS)) {
            move_toward_target();
        }
    }
#else
    service_operation_button(nowMs, buttonPressed);
    centerMode = gMotionPhase == BALL_MOTION_HOLD_CENTER;
    if (newVisionFrame) {
        gLastVisionSequence = vision->sequence;
        gHaveVisionSequence = true;
        if (centerMode) {
            if (vision->valid && !predicted) {
                if (gCenterArmFrames <
                    APP_BALL_CENTER_ARM_REAL_FRAMES) {
                    gCenterArmFrames++;
                }
            } else if (!vision->valid) {
                gCenterArmFrames = 0U;
                gCenterStableFrames = 0U;
                gCenterCandidate = false;
                gCenterNoProgressFrames = 0U;
                gCenterRecoveryRealFramePermit = false;
            }
        }
        if (vision->valid &&
            (gMotionPhase != BALL_MOTION_IDLE) &&
            (gMotionPhase != BALL_MOTION_RETURN_TO_LEVEL) &&
            (gTelemetry.state != BALL_ROD_SAFETY_FAULT)) {
            if (gMotionPhase == BALL_MOTION_HOLD_CENTER) {
                if (gCenterArmFrames >=
                    APP_BALL_CENTER_ARM_REAL_FRAMES) {
                    update_center_hold_target(vision, nowMs);
                }
            } else {
                update_pid_target(vision);
            }
        }
    }

    if (!gDriverEnabled) {
        if ((gMotionPhase != BALL_MOTION_IDLE) &&
            vision->valid &&
            ((centerMode &&
                (gCenterArmFrames >=
                    APP_BALL_CENTER_ARM_REAL_FRAMES)) ||
             (!centerMode &&
                (vision->validStreak >=
                    APP_BALL_VALID_FRAMES_TO_ARM)))) {
            set_direction_for_sign(1);
            DL_GPIO_setPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
            gDriverEnabled = true;
            gEnableStartMs = nowMs;
            if ((gMotionPhase == BALL_MOTION_TO_POSITIVE_5CM) ||
                (gMotionPhase == BALL_MOTION_RETURN_TO_LEVEL) ||
                (gMotionPhase == BALL_MOTION_TO_NEGATIVE_5CM)) {
                gMotionStartMs = nowMs;
                gMotionTimerStarted = true;
            }
        } else {
            gTelemetry.state = BALL_ROD_WAITING_VISION;
        }
    } else if (gTelemetry.state == BALL_ROD_SAFETY_FAULT) {
        /* Keep STEP stopped and PA24 high so the rod remains locked. */
    } else if (centerMode) {
        if (!vision->received ||
            elapsed_ms(nowMs, vision->lastValidMs,
                APP_BALL_VISION_FAULT_MS)) {
            /*
             * With no angle encoder, returning to software step zero is not a
             * safe response to lost vision.  Stop STEP and retain the last
             * physical rod angle with PA24 still high.
             */
            stop_and_hold_current_position();
            gTelemetry.state = BALL_ROD_VISION_FAULT;
            if (!gVisionFaultReported) {
                gVisionFaultReported = true;
                publish_status_event(
                    BALL_STATUS_EVENT_VISION_FAULT);
            }
        } else if (elapsed_ms(nowMs, vision->lastValidMs,
                       APP_BALL_VISION_RETURN_MS)) {
            stop_and_hold_current_position();
            gTelemetry.state = BALL_ROD_RETURNING;
        } else {
            gTelemetry.state = BALL_ROD_ACTIVE;
            gVisionFaultReported = false;
        }
    } else if (!vision->received ||
        elapsed_ms(nowMs, vision->lastValidMs,
            APP_BALL_VISION_FAULT_MS)) {
        gTargetSteps = 0;
        gPositionPidInitialized = false;
        reset_center_hold_state();
        gTelemetry.state = BALL_ROD_VISION_FAULT;
    } else if (elapsed_ms(nowMs, vision->lastValidMs,
                   APP_BALL_VISION_RETURN_MS)) {
        gTargetSteps = 0;
        gPositionPidInitialized = false;
        reset_center_hold_state();
        gTelemetry.state = BALL_ROD_RETURNING;
    } else {
        gTelemetry.state = BALL_ROD_ACTIVE;
    }

    if (gMotionTimerStarted &&
        (gMotionPhase != BALL_MOTION_HOLD_NEGATIVE_5CM) &&
        elapsed_ms(nowMs, gMotionStartMs, APP_BALL_SEQUENCE_TIMEOUT_MS)) {
        /*
         * Record that the competition time target was missed, but keep the
         * controller running toward -5 cm.  A timing result must never turn
         * an intermediate ball coordinate into a false final position.
         */
        gSequenceTimedOut = true;
    }

    if (gTelemetry.state == BALL_ROD_ACTIVE) {
        service_return_to_level(nowMs);
        if (centerMode) {
            service_center_recovery(nowMs);
        }
    }

    if (gDriverEnabled &&
        elapsed_ms(nowMs, gEnableStartMs, APP_D36A_WAKE_DELAY_MS) &&
        (!centerMode ||
            (gCenterArmFrames >=
                APP_BALL_CENTER_ARM_REAL_FRAMES))) {
        move_toward_target();
    }
#endif

    if (gCurrentSteps < gTelemetry.minimumReached) {
        gTelemetry.minimumReached = gCurrentSteps;
    }
    if (gCurrentSteps > gTelemetry.maximumReached) {
        gTelemetry.maximumReached = gCurrentSteps;
    }
    gTelemetry.currentSteps = gCurrentSteps;
    gTelemetry.targetSteps = gTargetSteps;
    gTelemetry.motionPhase = gMotionPhase;
    gTelemetry.stepFrequencyHz = gStepFrequencyHz;
    gTelemetry.stepRunning = gStepRunning;
    gTelemetry.sequenceTimedOut = gSequenceTimedOut;
    gTelemetry.driverEnabled = gDriverEnabled;
    gTelemetry.visionFresh = vision->received &&
        !elapsed_ms(nowMs, vision->lastValidMs,
            APP_BALL_VISION_RETURN_MS);
    gTelemetry.centerSettled = gCenterSettled;
    gTelemetry.mustCorrect = gCenterMustCorrect;
    gTelemetry.approachingCenter = gCenterApproaching;
    gTelemetry.recoveryActive =
        (gCenterRecoveryPhase != BALL_CENTER_RECOVERY_NONE) ||
        gCenterForceCooldown;
    gTelemetry.limitReached = gCenterAtLimit ||
        ((gCenterTiltLimit != 0U) &&
            (absolute_i32(gCurrentSteps - gCenterReferenceSteps) >=
                (gCenterTiltLimit -
                    APP_BALL_POSITION_TOLERANCE_STEPS)));
    gTelemetry.filteredVelocity =
        (int16_t) gCenterFilteredVelocity;
    gTelemetry.runId = gCenterRunId;
    gTelemetry.tiltLimit = gCenterTiltLimit;
    gTelemetry.eventCounter = gEventCounter;
    gTelemetry.recoveryPhase = gCenterRecoveryPhase;
    gTelemetry.armFrames = gCenterArmFrames;
    gTelemetry.event = gLastEvent;
}

void ball_rod_step_isr(void)
{
    if (DL_TimerG_getPendingInterrupt(PWM_BALL_STEP_INST) ==
        DL_TIMERG_IIDX_CC1_DN) {
        gCurrentSteps += gStepSign;
        if (((gStepSign > 0) && (gCurrentSteps >= gTargetSteps)) ||
            ((gStepSign < 0) && (gCurrentSteps <= gTargetSteps))) {
            gCurrentSteps = gTargetSteps;
            step_pin_gpio_low();
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
