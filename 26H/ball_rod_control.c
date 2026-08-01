#include "ball_rod_control.h"

#include "app_config.h"
#include "ball_stepper.h"
#include "ti_msp_dl_config.h"

#define BALL_VISION_FLAG_PREDICTED (0x02U)

/*
 * Q4/Q5/Q6 balance law adapted from the H-problem open-source
 * BalanceController (AFL-3.0).  The original drives a ZDT absolute-angle
 * motor.  This derivative port retains its estimator/state logic and maps the
 * requested angle to bounded D36A relative microsteps.
 */

typedef enum {
    BALL_OPEN_MOTION_UNKNOWN = 0,
    BALL_OPEN_MOTION_STATIONARY,
    BALL_OPEN_MOTION_BREAKAWAY,
    BALL_OPEN_MOTION_ROLLING,
    BALL_OPEN_MOTION_SETTLED
} BallOpenMotionState;

typedef struct {
    uint32_t timeMs;
    int16_t x;
} BallVelocitySample;

typedef enum {
    BALL_PRACTICE_IDLE = 0,
    BALL_PRACTICE_TO_POS5,
    BALL_PRACTICE_SETTLED_POS5,
    BALL_PRACTICE_TO_NEG5,
    BALL_PRACTICE_SETTLED_NEG5,
    BALL_PRACTICE_COMPLETE,
    BALL_PRACTICE_TIMEOUT
} BallPracticePhase;

static BallRodTelemetry gTelemetry;
static uint16_t gTargetXQ4;
static uint16_t gLastVisionSequence;
static bool gHaveVisionSequence;
static bool gHaveRealVision;
static uint32_t gLastRealVisionMs;
static uint32_t gRunRequestMs;
static uint32_t gLastOuterUpdateMs;
static uint8_t gRealArmFrames;
static float gFilteredVelocity;
static float gContinuousTilt;
static float gTiltResidual;
static float gFrictionBoost;
static float gChassisAccelCompensation;
static float gDisturbanceTrim;
static float gSettledHoldTilt;
static float gMotionAnchorX;
static float gBreakawayAnchorX;
static BallVelocitySample
    gVelocitySamples[APP_Q456_BALL_VELOCITY_SAMPLE_COUNT];
static uint8_t gVelocitySampleCount;
static uint32_t gStationarySinceMs;
static uint32_t gSettledExitSinceMs;
static uint32_t gBreakawayStartedMs;
static uint32_t gTrimErrorSinceMs;
static uint32_t gEndpointSafeSinceMs;
static int8_t gTrimErrorDirection;
static int8_t gBreakawayErrorDirection;
static int8_t gEndpointSide;
static bool gLastOutputSaturated;
static bool gHoldLatched;
static BallOpenMotionState gOpenMotionState;
static bool gRunRequested;
static bool gSafetyLatched;
static int32_t gMinimumReached;
static int32_t gMaximumReached;

static bool gButtonRaw;
static bool gButtonStable;
static bool gButtonEmergencyHandled;
static uint32_t gButtonRawChangedMs;
static uint32_t gButtonPressedMs;
static BallPracticePhase gPracticePhase;
static uint32_t gPracticeStartMs;
static uint32_t gPracticeTargetSettledMs;
static float gPracticeBalanceTilt;
#if APP_BALL_CALIBRATION_MODE
static bool gCalibrationClickPending;
static uint32_t gCalibrationFirstClickMs;
#endif

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float move_toward(float current, float target, float maximumDelta)
{
    float delta = target - current;

    if (delta > maximumDelta) {
        delta = maximumDelta;
    } else if (delta < -maximumDelta) {
        delta = -maximumDelta;
    }
    return current + delta;
}

static int32_t round_float(float value)
{
    return (value >= 0.0f) ?
        (int32_t) (value + 0.5f) : (int32_t) (value - 0.5f);
}

static int16_t clamp_i16(int32_t value)
{
    if (value < -32768) {
        return -32768;
    }
    if (value > 32767) {
        return 32767;
    }
    return (int16_t) value;
}

static void publish_event(uint8_t event)
{
    gTelemetry.event = event;
    gTelemetry.eventCounter++;
}

static void reset_control_history(int32_t currentSteps, uint32_t nowMs)
{
    gLastOuterUpdateMs = nowMs;
    gFilteredVelocity = 0.0f;
    gContinuousTilt = (float) currentSteps;
    gTiltResidual = 0.0f;
    gFrictionBoost = 0.0f;
    gDisturbanceTrim = 0.0f;
    gSettledHoldTilt = (float) currentSteps;
    gMotionAnchorX = 0.0f;
    gBreakawayAnchorX = 0.0f;
    gVelocitySampleCount = 0U;
    gStationarySinceMs = nowMs;
    gSettledExitSinceMs = 0U;
    gBreakawayStartedMs = 0U;
    gTrimErrorSinceMs = 0U;
    gEndpointSafeSinceMs = 0U;
    gTrimErrorDirection = 0;
    gBreakawayErrorDirection = 0;
    gEndpointSide = 0;
    gLastOutputSaturated = false;
    gHoldLatched = false;
    gOpenMotionState = BALL_OPEN_MOTION_UNKNOWN;
}

static void enter_waiting_vision(uint32_t nowMs)
{
    BallStepperStatus stepper = ball_stepper_get_status();

    ball_stepper_enable(nowMs);
    ball_stepper_hold();
    reset_control_history(stepper.currentSteps, nowMs);
    gRealArmFrames = 0U;
    gHaveRealVision = false;
    gRunRequestMs = nowMs;
    gTelemetry.state = BALL_ROD_WAITING_VISION;
    gTelemetry.motionPhase = BALL_MOTION_IDLE;
    gTelemetry.faultReason = BALL_FAULT_NONE;
    gTelemetry.runId++;
    publish_event(BALL_STATUS_EVENT_CENTER_START);
}

static void pause_control(void)
{
    BallStepperStatus stepper;

    ball_stepper_hold();
    stepper = ball_stepper_get_status();
    reset_control_history(stepper.currentSteps, 0U);
    gChassisAccelCompensation = 0.0f;
    gRunRequested = false;
    gRealArmFrames = 0U;
    gTelemetry.state = BALL_ROD_DISARMED;
    gTelemetry.motionPhase = BALL_MOTION_IDLE;
    gTelemetry.faultReason = BALL_FAULT_NONE;
    publish_event(BALL_STATUS_EVENT_CENTER_END);
}

static void emergency_stop(void)
{
    ball_stepper_disarm();
    gRunRequested = false;
    gChassisAccelCompensation = 0.0f;
    gSafetyLatched = true;
    gRealArmFrames = 0U;
    gTelemetry.state = BALL_ROD_SAFETY_FAULT;
    gTelemetry.motionPhase = BALL_MOTION_IDLE;
    gTelemetry.faultReason = BALL_FAULT_EMERGENCY_STOP;
    publish_event(BALL_STATUS_EVENT_CENTER_END);
}

#if APP_BALL_CALIBRATION_MODE
static void calibration_jog(uint32_t nowMs, int8_t sign)
{
    BallStepperStatus stepper = ball_stepper_get_status();
    int32_t target = stepper.currentSteps +
        (int32_t) sign * APP_BALL_CALIBRATION_JOG_STEPS;

    ball_stepper_enable(nowMs);
    ball_stepper_set_requested_hz(APP_BALL_STEP_MIN_HZ);
    ball_stepper_set_target(target);
    gTelemetry.state = BALL_ROD_DISARMED;
    gTelemetry.motionPhase = BALL_MOTION_CALIBRATION;
}
#endif

static void handle_short_click(uint32_t nowMs)
{
    if (gSafetyLatched) {
        return;
    }
#if APP_BALL_CALIBRATION_MODE
    if (gCalibrationClickPending &&
        ((uint32_t) (nowMs - gCalibrationFirstClickMs) <=
            APP_BALL_DOUBLE_CLICK_MS)) {
        gCalibrationClickPending = false;
        calibration_jog(nowMs, -1);
    } else {
        gCalibrationClickPending = true;
        gCalibrationFirstClickMs = nowMs;
    }
#else
    if (gRunRequested) {
        pause_control();
    } else {
        gRunRequested = true;
        enter_waiting_vision(nowMs);
    }
#endif
}

static void service_button(uint32_t nowMs, bool pressed)
{
    if (pressed != gButtonRaw) {
        gButtonRaw = pressed;
        gButtonRawChangedMs = nowMs;
    }
    if ((gButtonStable != gButtonRaw) &&
        ((uint32_t) (nowMs - gButtonRawChangedMs) >=
            APP_BUTTON_DEBOUNCE_MS)) {
        gButtonStable = gButtonRaw;
        if (gButtonStable) {
            gButtonPressedMs = nowMs;
            gButtonEmergencyHandled = false;
        } else if (!gButtonEmergencyHandled) {
            handle_short_click(nowMs);
        }
    }

    /* 2 s emergency stop disabled in Q5 mode. */
    (void)emergency_stop;

#if APP_BALL_CALIBRATION_MODE
    if (gCalibrationClickPending &&
        ((uint32_t) (nowMs - gCalibrationFirstClickMs) >
            APP_BALL_DOUBLE_CLICK_MS)) {
        gCalibrationClickPending = false;
        calibration_jog(nowMs, 1);
    }
#endif
}

static void command_continuous_tilt(float desiredTilt, bool leveling);

static void enter_vision_fault(uint32_t nowMs)
{
    if (gTelemetry.state != BALL_ROD_VISION_FAULT) {
        BallStepperStatus stepper = ball_stepper_get_status();

        reset_control_history(stepper.currentSteps, nowMs);
        publish_event(BALL_STATUS_EVENT_VISION_FAULT);
    }
    gRealArmFrames = 0U;
    gTelemetry.state = BALL_ROD_VISION_FAULT;
    gTelemetry.motionPhase = BALL_MOTION_IDLE;
    gTelemetry.faultReason = BALL_FAULT_VISION_STALE;
    /* Open-source vision-loss policy: return toward level, never freeze a
     * stale corrective tilt.  Chassis acceleration feedforward remains valid. */
    command_continuous_tilt(
        APP_BALL_NEUTRAL_STEPS + gChassisAccelCompensation, true);
}

static void shift_velocity_samples_left(void)
{
    for (uint8_t i = 1U; i < gVelocitySampleCount; i++) {
        gVelocitySamples[i - 1U] = gVelocitySamples[i];
    }
    if (gVelocitySampleCount != 0U) {
        gVelocitySampleCount--;
    }
}

static void update_velocity_estimator(uint32_t nowMs, int16_t ballX)
{
    float meanTime = 0.0f;
    float meanX = 0.0f;
    float numerator = 0.0f;
    float denominator = 0.0f;
    float velocity;
    int16_t minimumX = ballX;
    int16_t maximumX = ballX;
    uint32_t firstTime;

    if ((gVelocitySampleCount != 0U) &&
        ((nowMs <= gVelocitySamples[gVelocitySampleCount - 1U].timeMs) ||
        ((uint32_t) (nowMs -
            gVelocitySamples[gVelocitySampleCount - 1U].timeMs) >
            APP_Q456_BALL_VELOCITY_WINDOW_MS))) {
        gVelocitySampleCount = 0U;
        gFilteredVelocity = 0.0f;
    }

    while ((gVelocitySampleCount > 2U) &&
        ((uint32_t) (nowMs - gVelocitySamples[0].timeMs) >
            APP_Q456_BALL_VELOCITY_WINDOW_MS)) {
        shift_velocity_samples_left();
    }
    if (gVelocitySampleCount >= APP_Q456_BALL_VELOCITY_SAMPLE_COUNT) {
        shift_velocity_samples_left();
    }
    gVelocitySamples[gVelocitySampleCount].timeMs = nowMs;
    gVelocitySamples[gVelocitySampleCount].x = ballX;
    gVelocitySampleCount++;

    if ((gVelocitySampleCount < 3U) ||
        ((uint32_t) (gVelocitySamples[gVelocitySampleCount - 1U].timeMs -
            gVelocitySamples[0].timeMs) <
            APP_Q456_BALL_VELOCITY_MIN_SPAN_MS)) {
        return;
    }

    firstTime = gVelocitySamples[0].timeMs;
    for (uint8_t i = 0U; i < gVelocitySampleCount; i++) {
        meanTime +=
            (float) (gVelocitySamples[i].timeMs - firstTime);
        meanX += (float) gVelocitySamples[i].x;
        if (gVelocitySamples[i].x < minimumX) {
            minimumX = gVelocitySamples[i].x;
        }
        if (gVelocitySamples[i].x > maximumX) {
            maximumX = gVelocitySamples[i].x;
        }
    }
    meanTime /= (float) gVelocitySampleCount;
    meanX /= (float) gVelocitySampleCount;
    for (uint8_t i = 0U; i < gVelocitySampleCount; i++) {
        float sampleTime =
            (float) (gVelocitySamples[i].timeMs - firstTime);
        float timeDelta = sampleTime - meanTime;

        numerator += timeDelta * ((float) gVelocitySamples[i].x - meanX);
        denominator += timeDelta * timeDelta;
    }
    if (denominator <= 0.001f) {
        return;
    }
    velocity = (numerator / denominator) * 1000.0f;
    if ((maximumX - minimumX) <= 1) {
        velocity = 0.0f;
    }
    gFilteredVelocity =
        APP_Q456_BALL_VELOCITY_FILTER_ALPHA * velocity +
        (1.0f - APP_Q456_BALL_VELOCITY_FILTER_ALPHA) *
            gFilteredVelocity;
}

static float derivative_brake_scale(float errorPixels)
{
    float distance;
    float fraction;

    if ((errorPixels * gFilteredVelocity) <= 0.0f) {
        return 1.0f;
    }
    distance = abs_float(errorPixels);
    if (distance >= APP_Q456_BALL_BRAKE_START_ERROR_PX) {
        return APP_Q456_BALL_BRAKE_FAR_SCALE;
    }
    if (distance <= APP_Q456_BALL_BRAKE_FULL_ERROR_PX) {
        return 1.0f;
    }
    fraction = (APP_Q456_BALL_BRAKE_START_ERROR_PX - distance) /
        (APP_Q456_BALL_BRAKE_START_ERROR_PX -
            APP_Q456_BALL_BRAKE_FULL_ERROR_PX);
    return APP_Q456_BALL_BRAKE_FAR_SCALE +
        (1.0f - APP_Q456_BALL_BRAKE_FAR_SCALE) * fraction;
}

static void latch_settled_hold(uint32_t nowMs)
{
    if (!gHoldLatched) {
        publish_event(BALL_STATUS_EVENT_CENTER_SETTLED);
    }
    gHoldLatched = true;
    gOpenMotionState = BALL_OPEN_MOTION_SETTLED;
    gSettledHoldTilt = gContinuousTilt - gChassisAccelCompensation;
    gSettledExitSinceMs = 0U;
    gFrictionBoost = 0.0f;
    gBreakawayStartedMs = 0U;
    gBreakawayErrorDirection = 0;
    gStationarySinceMs = nowMs;
}

static void release_settled_hold(uint32_t nowMs, int16_t ballX)
{
    gHoldLatched = false;
    gSettledExitSinceMs = 0U;
    gOpenMotionState =
        (abs_float(gFilteredVelocity) >=
            APP_Q456_BALL_ROLLING_SPEED_PX_S) ?
        BALL_OPEN_MOTION_ROLLING : BALL_OPEN_MOTION_STATIONARY;
    gMotionAnchorX = (float) ballX;
    gStationarySinceMs = nowMs;
    publish_event(BALL_STATUS_EVENT_CORRECTION_RESUME);
}

static void update_motion_state(
    uint32_t nowMs, int16_t ballX, float errorPixels)
{
    float displacement;
    bool rolling;
    bool fastOutward;

    if (gOpenMotionState == BALL_OPEN_MOTION_UNKNOWN) {
        gOpenMotionState = BALL_OPEN_MOTION_STATIONARY;
        gMotionAnchorX = (float) ballX;
        gStationarySinceMs = nowMs;
        return;
    }

    if (gHoldLatched) {
        fastOutward =
            (abs_float(gFilteredVelocity) >=
                APP_Q456_BALL_SETTLED_FAST_EXIT_SPEED_PX_S) &&
            ((errorPixels * gFilteredVelocity) < 0.0f);
        if ((abs_float(errorPixels) <=
                APP_Q456_BALL_SETTLED_EXIT_ERROR_PX) &&
            !fastOutward) {
            gSettledExitSinceMs = 0U;
            return;
        }
        if (fastOutward) {
            release_settled_hold(nowMs, ballX);
            return;
        }
        if (gSettledExitSinceMs == 0U) {
            gSettledExitSinceMs = nowMs;
            return;
        }
        if ((uint32_t) (nowMs - gSettledExitSinceMs) >=
            APP_Q456_BALL_SETTLED_EXIT_CONFIRM_MS) {
            release_settled_hold(nowMs, ballX);
        }
        return;
    }

    displacement = abs_float((float) ballX - gMotionAnchorX);
    rolling = displacement >=
        APP_Q456_BALL_ROLLING_DISPLACEMENT_PX;
    if (rolling) {
        gOpenMotionState = BALL_OPEN_MOTION_ROLLING;
        gMotionAnchorX = (float) ballX;
        gStationarySinceMs = nowMs;
        return;
    }
    if (gStationarySinceMs == 0U) {
        gStationarySinceMs = nowMs;
    }
    if (((uint32_t) (nowMs - gStationarySinceMs) >=
            APP_Q456_BALL_STATIONARY_CONFIRM_MS) &&
        (abs_float(gFilteredVelocity) <=
            APP_Q456_BALL_STATIONARY_SPEED_PX_S)) {
        if (abs_float(errorPixels) <= APP_Q456_BALL_DEADBAND_PX) {
            latch_settled_hold(nowMs);
        } else if (gOpenMotionState != BALL_OPEN_MOTION_BREAKAWAY) {
            gOpenMotionState = BALL_OPEN_MOTION_STATIONARY;
        }
    }
}

static float update_breakaway_offset(
    uint32_t nowMs, uint32_t dtMs, int16_t ballX, float errorPixels)
{
    float dtSeconds = (float) dtMs * 0.001f;
    int8_t errorDirection =
        (errorPixels > 0.0f) ? 1 : ((errorPixels < 0.0f) ? -1 : 0);
    float correctiveDirection =
        (float) (APP_BALL_POSITION_TO_TILT_SIGN * errorDirection);
    bool staticError =
        abs_float(errorPixels) > APP_Q456_BALL_BREAKAWAY_ERROR_MIN_PX;
    bool lowSpeed = abs_float(gFilteredVelocity) <
        APP_Q456_BALL_ROLLING_SPEED_PX_S;
    bool staticState =
        (gOpenMotionState == BALL_OPEN_MOTION_STATIONARY) ||
        (gOpenMotionState == BALL_OPEN_MOTION_BREAKAWAY);
    float desired = 0.0f;
    float rate = APP_Q456_BALL_BREAKAWAY_RELEASE_STEPS_PER_S;

    if (staticError && lowSpeed && staticState) {
        float elapsedSeconds;
        float magnitude;

        if (gOpenMotionState != BALL_OPEN_MOTION_BREAKAWAY ||
            gBreakawayErrorDirection != errorDirection) {
            gOpenMotionState = BALL_OPEN_MOTION_BREAKAWAY;
            gBreakawayStartedMs = nowMs;
            gBreakawayAnchorX = (float) ballX;
            gBreakawayErrorDirection = errorDirection;
            publish_event(BALL_STATUS_EVENT_RECOVERY_REAPPLY);
        }
        elapsedSeconds =
            (float) (nowMs - gBreakawayStartedMs) * 0.001f;
        magnitude = APP_Q456_BALL_BREAKAWAY_INITIAL_STEPS +
            APP_Q456_BALL_BREAKAWAY_ESCALATE_STEPS_PER_S * elapsedSeconds;
        magnitude = clamp_float(magnitude, 0.0f,
            APP_Q456_BALL_BREAKAWAY_MAX_STEPS);
        magnitude = clamp_float(magnitude, 0.0f,
            abs_float(errorPixels) *
                APP_Q456_BALL_BREAKAWAY_ERROR_GAIN_STEPS_PER_PX);
        desired = correctiveDirection * magnitude;
        rate = APP_Q456_BALL_BREAKAWAY_RAMP_STEPS_PER_S;
    } else {
        gBreakawayStartedMs = 0U;
        gBreakawayErrorDirection = 0;
    }

    if ((errorDirection == 0) ||
        (abs_float(errorPixels) <=
            APP_Q456_BALL_SETTLED_HOLD_ERROR_PX) ||
        ((gFrictionBoost * correctiveDirection) < 0.0f)) {
        gFrictionBoost = 0.0f;
    } else {
        bool targetProgress =
            (errorPixels * gFilteredVelocity > 0.0f) &&
            (abs_float((float) ballX - gBreakawayAnchorX) >=
                APP_Q456_BALL_ROLLING_DISPLACEMENT_PX);
        if (targetProgress) {
            desired = 0.0f;
            rate = APP_Q456_BALL_BREAKAWAY_RELEASE_STEPS_PER_S;
            gOpenMotionState = BALL_OPEN_MOTION_ROLLING;
        }
        gFrictionBoost = move_toward(
            gFrictionBoost, desired, rate * dtSeconds);
    }
    return gFrictionBoost;
}

static float update_disturbance_trim(
    uint32_t nowMs, uint32_t dtMs, float errorPixels)
{
    int8_t direction =
        (errorPixels > 0.0f) ? 1 : ((errorPixels < 0.0f) ? -1 : 0);
    float speed = abs_float(gFilteredVelocity);
    float dtSeconds = (float) dtMs * 0.001f;
    float rate;

    if (gHoldLatched || gLastOutputSaturated ||
        (abs_float(errorPixels) <= APP_Q456_BALL_TRIM_DEADBAND_PX) ||
        (speed >= APP_Q456_BALL_TRIM_VELOCITY_GATE_PX_S)) {
        gTrimErrorDirection = 0;
        gTrimErrorSinceMs = 0U;
        return gDisturbanceTrim;
    }
    if (direction != gTrimErrorDirection) {
        gTrimErrorDirection = direction;
        gTrimErrorSinceMs = nowMs;
        return gDisturbanceTrim;
    }
    if ((gTrimErrorSinceMs == 0U) ||
        ((uint32_t) (nowMs - gTrimErrorSinceMs) <
            APP_Q456_BALL_TRIM_ACTIVATION_MS)) {
        return gDisturbanceTrim;
    }
    rate = APP_Q456_BALL_TRIM_GAIN_STEPS_PER_PX_S *
        (abs_float(errorPixels) - APP_Q456_BALL_TRIM_DEADBAND_PX);
    rate = clamp_float(rate, 0.0f,
        APP_Q456_BALL_TRIM_MAX_RATE_STEPS_PER_S);
    gDisturbanceTrim +=
        (float) (APP_BALL_POSITION_TO_TILT_SIGN * direction) *
        rate * dtSeconds;
    gDisturbanceTrim = clamp_float(gDisturbanceTrim,
        -APP_Q456_BALL_TRIM_LIMIT_STEPS,
        APP_Q456_BALL_TRIM_LIMIT_STEPS);
    return gDisturbanceTrim;
}

static bool endpoint_guard_active(
    uint32_t nowMs, int16_t ballX)
{
    float endpoint = APP_Q456_BALL_RAIL_LENGTH_PX *
        APP_Q456_BALL_ENDPOINT_MARGIN_RATIO;
    float recovery = APP_Q456_BALL_RAIL_LENGTH_PX *
        APP_Q456_BALL_ENDPOINT_RECOVERY_MARGIN_RATIO;
    bool insideCore = ((float) ballX >= recovery) &&
        ((float) ballX <= APP_Q456_BALL_RAIL_LENGTH_PX - recovery);
    bool lowSpeed = abs_float(gFilteredVelocity) <=
        APP_Q456_BALL_ENDPOINT_RECOVERY_SPEED_PX_S;

    if (gEndpointSide == 0) {
        if ((float) ballX <= endpoint) {
            gEndpointSide = -1;
        } else if ((float) ballX >=
            APP_Q456_BALL_RAIL_LENGTH_PX - endpoint) {
            gEndpointSide = 1;
        }
        if (gEndpointSide != 0) {
            gEndpointSafeSinceMs = 0U;
            gHoldLatched = false;
            gFrictionBoost = 0.0f;
            gDisturbanceTrim = 0.0f;
            gTrimErrorDirection = 0;
            gTrimErrorSinceMs = 0U;
            publish_event(BALL_STATUS_EVENT_RECOVERY_BACKOFF);
        }
    }
    if (gEndpointSide == 0) {
        return false;
    }
    if (!insideCore || !lowSpeed) {
        gEndpointSafeSinceMs = 0U;
        return true;
    }
    if (gEndpointSafeSinceMs == 0U) {
        gEndpointSafeSinceMs = nowMs;
        return true;
    }
    if ((uint32_t) (nowMs - gEndpointSafeSinceMs) <
        APP_Q456_BALL_ENDPOINT_RECOVERY_CONFIRM_MS) {
        return true;
    }
    gEndpointSide = 0;
    gEndpointSafeSinceMs = 0U;
    gVelocitySampleCount = 0U;
    gFilteredVelocity = 0.0f;
    gOpenMotionState = BALL_OPEN_MOTION_UNKNOWN;
    publish_event(BALL_STATUS_EVENT_CORRECTION_RESUME);
    return false;
}

static void command_continuous_tilt(float desiredTilt, bool leveling)
{
    BallStepperStatus stepper = ball_stepper_get_status();
    float maximumDelta = leveling ?
        APP_BALL_LEVEL_SLEW_STEPS_PER_FRAME :
        APP_BALL_TARGET_SLEW_STEPS_PER_FRAME;
    float quantizedInput;
    int32_t integerTarget;
    uint32_t frequency;

    desiredTilt = clamp_float(desiredTilt,
        -(float) APP_BALL_WORK_TILT_LIMIT_STEPS,
        (float) APP_BALL_WORK_TILT_LIMIT_STEPS);
    gContinuousTilt =
        move_toward(gContinuousTilt, desiredTilt, maximumDelta);

    quantizedInput = gContinuousTilt + gTiltResidual;
    integerTarget = round_float(quantizedInput);
    gTiltResidual = quantizedInput - (float) integerTarget;
    ball_stepper_set_target(integerTarget);

    frequency = APP_BALL_STEP_MIN_HZ +
        APP_BALL_STEP_HZ_PER_ERROR *
        (uint32_t) abs_i32(integerTarget - stepper.currentSteps);
    if (frequency > APP_BALL_STEP_MAX_HZ) {
        frequency = APP_BALL_STEP_MAX_HZ;
    }
    ball_stepper_set_requested_hz((uint16_t) frequency);
}

#if 0
static void update_controller_from_real_frame(
    uint32_t nowMs, const BallVisionSample *vision)
{
    uint32_t dtMs = (uint32_t) (nowMs - gLastOuterUpdateMs);
    int16_t errorQ4 = (int16_t) ((int32_t) gTargetXQ4 -
        (int32_t) vision->xQ4);
    float errorPixels = (float) errorQ4 * (1.0f / 16.0f);
    float desiredTilt;
    bool enterHold;
    bool releaseHold;

    update_velocity_filter(vision->velocityX, dtMs);
    gLastOuterUpdateMs = nowMs;
    gLastRealVisionMs = nowMs;
    gHaveRealVision = true;

    gTelemetry.ballX = (int16_t) ((vision->xQ4 + 8U) / 16U);
    gTelemetry.ballErrorQ4 = errorQ4;
    gTelemetry.ballError =
        (int16_t) round_float((float) errorQ4 * (1.0f / 16.0f));

    if (!gRunRequested) {
        return;
    }
    if (gRealArmFrames < APP_BALL_VALID_FRAMES_TO_ARM) {
        gRealArmFrames++;
    }
    if (gRealArmFrames < APP_BALL_VALID_FRAMES_TO_ARM) {
        gTelemetry.state = BALL_ROD_WAITING_VISION;
        gTelemetry.motionPhase = BALL_MOTION_IDLE;
        return;
    }
    gTelemetry.faultReason = BALL_FAULT_NONE;

    enterHold =
        (abs_i32(errorQ4) <= APP_BALL_HOLD_ENTER_ERROR_Q4) &&
        (abs_float(gFilteredVelocity) <=
            APP_BALL_HOLD_ENTER_SPEED_PX_S);
    releaseHold =
        (abs_i32(errorQ4) >= APP_BALL_HOLD_RELEASE_ERROR_Q4) ||
        (abs_float(gFilteredVelocity) >=
            APP_BALL_HOLD_RELEASE_SPEED_PX_S);

    if (!gHoldLatched && enterHold) {
        gHoldLatched = true;
        gFrictionBoost = 0.0f;
        gIntegral = 0.0f;
        gTiltResidual = 0.0f;
        gStuckFrames = 0U;
        publish_event(BALL_STATUS_EVENT_CENTER_SETTLED);
    } else if (gHoldLatched && releaseHold) {
        gHoldLatched = false;
        publish_event(BALL_STATUS_EVENT_CORRECTION_RESUME);
    }

    if (gHoldLatched) {
        gTelemetry.state = BALL_ROD_HOLD;
        gTelemetry.motionPhase = BALL_MOTION_LEVELING;
        gFrictionBoost = 0.0f;
        gIntegral = 0.0f;
        gTiltResidual = 0.0f;
        gStuckFrames = 0U;
        command_continuous_tilt(
            APP_BALL_NEUTRAL_STEPS + gChassisAccelCompensation, true);
        return;
    }

    gTelemetry.state = BALL_ROD_ACTIVE;
    gTelemetry.motionPhase = BALL_MOTION_CORRECTING;

    /*
     * Continuous PD — 4 个锚点线性插值。
     *
     *   锚点  |E|:    0      5      15     40     (px)
     *          KP:  0.030  0.080  0.120  0.200
     *          KD:  0.015  0.044  0.066  0.120
     *     KD/KP:  0.50   0.55   0.55   0.60
     */
    {
        float absErrPx = abs_float(errorPixels);
        float kp, kd;

        if (absErrPx <= APP_BALL_GAIN_ERROR_1_PX) {
            float t = absErrPx / APP_BALL_GAIN_ERROR_1_PX;
            kp = APP_BALL_GAIN_KP_0 +
                t * (APP_BALL_GAIN_KP_1 - APP_BALL_GAIN_KP_0);
            kd = APP_BALL_GAIN_KD_0 +
                t * (APP_BALL_GAIN_KD_1 - APP_BALL_GAIN_KD_0);
        } else if (absErrPx <= APP_BALL_GAIN_ERROR_2_PX) {
            float t = (absErrPx - APP_BALL_GAIN_ERROR_1_PX) /
                (APP_BALL_GAIN_ERROR_2_PX - APP_BALL_GAIN_ERROR_1_PX);
            kp = APP_BALL_GAIN_KP_1 +
                t * (APP_BALL_GAIN_KP_2 - APP_BALL_GAIN_KP_1);
            kd = APP_BALL_GAIN_KD_1 +
                t * (APP_BALL_GAIN_KD_2 - APP_BALL_GAIN_KD_1);
        } else if (absErrPx <= APP_BALL_GAIN_ERROR_3_PX) {
            float t = (absErrPx - APP_BALL_GAIN_ERROR_2_PX) /
                (APP_BALL_GAIN_ERROR_3_PX - APP_BALL_GAIN_ERROR_2_PX);
            kp = APP_BALL_GAIN_KP_2 +
                t * (APP_BALL_GAIN_KP_3 - APP_BALL_GAIN_KP_2);
            kd = APP_BALL_GAIN_KD_2 +
                t * (APP_BALL_GAIN_KD_3 - APP_BALL_GAIN_KD_2);
        } else {
            kp = APP_BALL_GAIN_KP_3;
            kd = APP_BALL_GAIN_KD_3;
        }

        desiredTilt = (float) APP_BALL_POSITION_TO_TILT_SIGN *
            (kp * errorPixels - kd * gFilteredVelocity);
        desiredTilt = apply_rough_tube_compensation(
            desiredTilt, errorQ4);
        desiredTilt += gChassisAccelCompensation;
        command_continuous_tilt(desiredTilt, false);
    }
}

#endif

static void update_controller_from_real_frame(
    uint32_t nowMs, const BallVisionSample *vision)
{
    uint32_t dtMs = (uint32_t) (nowMs - gLastOuterUpdateMs);
    int16_t errorQ4 = (int16_t) ((int32_t) gTargetXQ4 -
        (int32_t) vision->xQ4);
    float errorPixels = (float) errorQ4 * (1.0f / 16.0f);
    int16_t ballX = (int16_t) ((vision->xQ4 + 8U) / 16U);
    float desiredTilt;
    float controlError;
    float kp;
    float pTerm;
    float dTerm;
    float correction;
    float trim;
    float breakaway;
    float brakeScale;

    if (dtMs < APP_BALL_OUTER_DT_MIN_MS) {
        dtMs = APP_BALL_OUTER_DT_MIN_MS;
    } else if (dtMs > APP_BALL_OUTER_DT_MAX_MS) {
        dtMs = APP_BALL_OUTER_DT_MAX_MS;
    }
    update_velocity_estimator(nowMs, ballX);
    gLastOuterUpdateMs = nowMs;
    gLastRealVisionMs = nowMs;
    gHaveRealVision = true;

    gTelemetry.ballX = ballX;
    gTelemetry.ballErrorQ4 = errorQ4;
    gTelemetry.ballError =
        (int16_t) round_float((float) errorQ4 * (1.0f / 16.0f));

    if (!gRunRequested) {
        return;
    }
    if (gRealArmFrames < APP_BALL_VALID_FRAMES_TO_ARM) {
        gRealArmFrames++;
    }
    if (gRealArmFrames < APP_BALL_VALID_FRAMES_TO_ARM) {
        gTelemetry.state = BALL_ROD_WAITING_VISION;
        gTelemetry.motionPhase = BALL_MOTION_IDLE;
        return;
    }
    gTelemetry.faultReason = BALL_FAULT_NONE;

    if (endpoint_guard_active(nowMs, ballX)) {
        int8_t recoveryDirection =
            (errorPixels > 0.0f) ? 1 :
            ((errorPixels < 0.0f) ? -1 : -gEndpointSide);

        gTelemetry.state = BALL_ROD_ACTIVE;
        gTelemetry.motionPhase = BALL_MOTION_CORRECTING;
        gOpenMotionState = BALL_OPEN_MOTION_ROLLING;
        gFrictionBoost = 0.0f;
        desiredTilt = APP_BALL_NEUTRAL_STEPS +
            gChassisAccelCompensation +
            (float) (APP_BALL_POSITION_TO_TILT_SIGN * recoveryDirection) *
                APP_Q456_BALL_ENDPOINT_RECOVERY_TILT_STEPS;
        command_continuous_tilt(desiredTilt, false);
        return;
    }

    update_motion_state(nowMs, ballX, errorPixels);
    if (gHoldLatched) {
        gTelemetry.state = BALL_ROD_HOLD;
        gTelemetry.motionPhase = BALL_MOTION_HOLD_RED_LINE;
        gFrictionBoost = 0.0f;
        gTiltResidual = 0.0f;
        command_continuous_tilt(
            gSettledHoldTilt + gChassisAccelCompensation, true);
        return;
    }

    gTelemetry.state = BALL_ROD_ACTIVE;
    gTelemetry.motionPhase = BALL_MOTION_CORRECTING;
    controlError =
        (abs_float(errorPixels) <= APP_Q456_BALL_DEADBAND_PX) ?
        0.0f : errorPixels;
    kp = ((gOpenMotionState == BALL_OPEN_MOTION_STATIONARY) ||
        (gOpenMotionState == BALL_OPEN_MOTION_BREAKAWAY)) ?
        APP_Q456_BALL_STATIONARY_KP_STEPS_PER_PX :
        APP_Q456_BALL_ROLLING_KP_STEPS_PER_PX;
    brakeScale = derivative_brake_scale(errorPixels);
    pTerm = clamp_float(
        (float) APP_BALL_POSITION_TO_TILT_SIGN * kp * controlError,
        -APP_Q456_BALL_P_LIMIT_STEPS,
        APP_Q456_BALL_P_LIMIT_STEPS);
    dTerm = clamp_float(
        -(float) APP_BALL_POSITION_TO_TILT_SIGN *
            APP_Q456_BALL_KD_STEPS_PER_PX_S *
            brakeScale * gFilteredVelocity,
        -APP_Q456_BALL_D_LIMIT_STEPS,
        APP_Q456_BALL_D_LIMIT_STEPS);
    trim = update_disturbance_trim(nowMs, dtMs, errorPixels);
    breakaway = update_breakaway_offset(
        nowMs, dtMs, ballX, errorPixels);
    correction = clamp_float(pTerm + dTerm + breakaway,
        -APP_Q456_BALL_DYNAMIC_LIMIT_STEPS,
        APP_Q456_BALL_DYNAMIC_LIMIT_STEPS);
    desiredTilt = APP_BALL_NEUTRAL_STEPS + trim +
        gChassisAccelCompensation + correction;
    gLastOutputSaturated =
        (abs_float(pTerm) >= APP_Q456_BALL_P_LIMIT_STEPS) ||
        (abs_float(dTerm) >= APP_Q456_BALL_D_LIMIT_STEPS) ||
        (abs_float(pTerm + dTerm + breakaway) >
            APP_Q456_BALL_DYNAMIC_LIMIT_STEPS) ||
        (abs_float(desiredTilt) >
            (float) APP_BALL_WORK_TILT_LIMIT_STEPS);
    command_continuous_tilt(desiredTilt, false);
}

static void update_telemetry(
    uint32_t nowMs, const BallVisionSample *vision)
{
    BallStepperStatus stepper = ball_stepper_get_status();
    int32_t velocityRounded = round_float(gFilteredVelocity);
    int32_t tiltQ8 = round_float(gContinuousTilt * 256.0f);
    int32_t frictionQ8 = round_float(gFrictionBoost * 256.0f);

    if (stepper.currentSteps < gMinimumReached) {
        gMinimumReached = stepper.currentSteps;
    }
    if (stepper.currentSteps > gMaximumReached) {
        gMaximumReached = stepper.currentSteps;
    }

    gTelemetry.currentSteps = stepper.currentSteps;
    gTelemetry.targetSteps = stepper.targetSteps;
    gTelemetry.minimumReached = gMinimumReached;
    gTelemetry.maximumReached = gMaximumReached;
    gTelemetry.targetXQ4 = gTargetXQ4;
    gTelemetry.filteredVelocity = clamp_i16(velocityRounded);
    gTelemetry.continuousTiltQ8 = clamp_i16(tiltQ8);
    gTelemetry.frictionBoostQ8 = clamp_i16(frictionQ8);
    gTelemetry.stepFrequencyHz = stepper.stepHz;
    gTelemetry.directionLevel = stepper.directionLevel;
    gTelemetry.stepRunning = stepper.running;
    gTelemetry.driverEnabled = stepper.enabled;
    gTelemetry.limitReached = stepper.atLimit;
    gTelemetry.tiltLimit = APP_BALL_WORK_TILT_LIMIT_STEPS;
    gTelemetry.armFrames = gRealArmFrames;
    gTelemetry.centerSettled = gHoldLatched;
    gTelemetry.practiceActive =
        (gPracticePhase != BALL_PRACTICE_IDLE);
    gTelemetry.mustCorrect =
        abs_i32(gTelemetry.ballErrorQ4) >
            APP_BALL_HOLD_ENTER_ERROR_Q4;
    gTelemetry.approachingCenter =
        ((float) gTelemetry.ballErrorQ4 * gFilteredVelocity) > 0.0f;
    gTelemetry.recoveryActive =
        (gEndpointSide != 0) || (abs_float(gFrictionBoost) > 0.0f);
    gTelemetry.recoveryPhase = (gEndpointSide != 0) ?
        BALL_RECOVERY_ENDPOINT :
        ((abs_float(gFrictionBoost) > 0.0f) ?
            BALL_RECOVERY_STATIC_FRICTION : BALL_RECOVERY_NONE);
    gTelemetry.crcErrors = vision->crcErrors;
    gTelemetry.sequenceDrops = vision->sequenceDrops;
    gTelemetry.rxOverflows = vision->rxOverflows;

    if (gHaveRealVision) {
        gTelemetry.visionAgeMs =
            (uint32_t) (nowMs - gLastRealVisionMs);
    } else {
        gTelemetry.visionAgeMs = UINT32_MAX;
    }
    gTelemetry.visionFresh =
        gHaveRealVision &&
        (gTelemetry.visionAgeMs <= APP_BALL_VISION_STALE_MS);
    gTelemetry.sequenceTimedOut = !gTelemetry.visionFresh;
}

void ball_rod_init(uint32_t nowMs)
{
    gTargetXQ4 = APP_BALL_DEFAULT_TARGET_X_Q4;
    gLastVisionSequence = 0U;
    gHaveVisionSequence = false;
    gHaveRealVision = false;
    gLastRealVisionMs = nowMs;
    gRunRequestMs = nowMs;
    gRealArmFrames = 0U;
    gChassisAccelCompensation = 0.0f;
    reset_control_history(0, nowMs);
    gRunRequested = false;
    gSafetyLatched = false;
    gMinimumReached = 0;
    gMaximumReached = 0;
    gButtonRaw = false;
    gButtonStable = false;
    gButtonEmergencyHandled = false;
    gButtonRawChangedMs = nowMs;
    gButtonPressedMs = nowMs;
    gPracticePhase = BALL_PRACTICE_IDLE;
    gPracticeStartMs = nowMs;
    gPracticeTargetSettledMs = 0U;
    gPracticeBalanceTilt = 0.0f;
#if APP_BALL_CALIBRATION_MODE
    gCalibrationClickPending = false;
    gCalibrationFirstClickMs = nowMs;
#endif

    gTelemetry = (BallRodTelemetry) {0};
    gTelemetry.state = BALL_ROD_DISARMED;
    gTelemetry.motionPhase = BALL_MOTION_IDLE;
    gTelemetry.targetXQ4 = gTargetXQ4;
    gTelemetry.tiltLimit = APP_BALL_WORK_TILT_LIMIT_STEPS;
    gTelemetry.faultReason = BALL_FAULT_NONE;
    ball_stepper_init(nowMs);
}

void ball_rod_set_target_x_q4(uint16_t targetXQ4)
{
    if (targetXQ4 > APP_BALL_TARGET_MAX_X_Q4) {
        targetXQ4 = APP_BALL_TARGET_MAX_X_Q4;
    }
    if (targetXQ4 == gTargetXQ4) {
        return;
    }
    gTargetXQ4 = targetXQ4;
    gTelemetry.targetXQ4 = gTargetXQ4;
    if (gTelemetry.state == BALL_ROD_HOLD) {
        gTelemetry.state = BALL_ROD_ACTIVE;
        gTelemetry.motionPhase = BALL_MOTION_CORRECTING;
        publish_event(BALL_STATUS_EVENT_CORRECTION_RESUME);
    }
    gHoldLatched = false;
    gFrictionBoost = 0.0f;
    gTiltResidual = 0.0f;
    gVelocitySampleCount = 0U;
    gFilteredVelocity = 0.0f;
    gOpenMotionState = BALL_OPEN_MOTION_UNKNOWN;
    gStationarySinceMs = 0U;
    gBreakawayStartedMs = 0U;
    gBreakawayErrorDirection = 0;
}

bool ball_rod_enable_driver(uint32_t nowMs)
{
    if (gSafetyLatched) {
        return false;
    }

    ball_stepper_enable(nowMs);
    ball_stepper_hold();
    return true;
}

bool ball_rod_start(uint32_t nowMs)
{
    if (gSafetyLatched) {
        return false;
    }

    gRunRequested = true;
    enter_waiting_vision(nowMs);
    return true;
}

void ball_rod_start_practice(uint32_t nowMs)
{
    if (gSafetyLatched) {
        return;
    }
    if (!gRunRequested) {
        gRunRequested = true;
        enter_waiting_vision(nowMs);
    }
    gPracticePhase = BALL_PRACTICE_TO_POS5;
    gPracticeStartMs = nowMs;
    gPracticeTargetSettledMs = nowMs;
    gPracticeBalanceTilt = gContinuousTilt;
    gHoldLatched = false;
    gFrictionBoost = 0.0f;
    gOpenMotionState = BALL_OPEN_MOTION_UNKNOWN;
    ball_rod_set_target_x_q4(APP_BALL_PRACTICE_TARGET_POS5_Q4);
    publish_event(BALL_STATUS_EVENT_CORRECTION_RESUME);
}

void ball_rod_set_chassis_accel_compensation_steps(float steps)
{
    gChassisAccelCompensation = clamp_float(steps,
        -APP_Q456_BALL_ACCEL_FF_LIMIT_STEPS,
        APP_Q456_BALL_ACCEL_FF_LIMIT_STEPS);
}

void ball_rod_pause(void)
{
    if (!gSafetyLatched) {
        pause_control();
    }
}

void ball_rod_emergency_stop(void)
{
    emergency_stop();
}

void ball_rod_tick_5ms(
    uint32_t nowMs, bool buttonPressed, const BallVisionSample *vision)
{
    bool newFrame;
    bool realValidFrame;

    service_button(nowMs, buttonPressed);
    newFrame = vision->received &&
        (!gHaveVisionSequence ||
            (vision->sequence != gLastVisionSequence));
    if (newFrame) {
        gHaveVisionSequence = true;
        gLastVisionSequence = vision->sequence;
    }
    realValidFrame = newFrame && vision->valid &&
        ((vision->flags & BALL_VISION_FLAG_PREDICTED) == 0U);

#if !APP_BALL_CALIBRATION_MODE
    if (realValidFrame) {
        update_controller_from_real_frame(nowMs, vision);
    }

    /* Practice mode: timed open-loop sequence. */
    if (gRunRequested && gPracticePhase != BALL_PRACTICE_IDLE) {
        float practiceTilt;
        uint32_t phaseElapsed =
            (uint32_t)(nowMs - gPracticeTargetSettledMs);

        if (gPracticePhase == BALL_PRACTICE_TO_POS5) {
            practiceTilt = gPracticeBalanceTilt + 50.0f;
            if (phaseElapsed >= 1000U) {
                gPracticePhase = BALL_PRACTICE_TO_NEG5;
                gPracticeTargetSettledMs = nowMs;
            }
        } else if (gPracticePhase == BALL_PRACTICE_TO_NEG5) {
            practiceTilt = gPracticeBalanceTilt - 33.0f;
            if (phaseElapsed >= 2000U) {
                gPracticePhase = BALL_PRACTICE_SETTLED_NEG5;
                gPracticeTargetSettledMs = nowMs;
            }
        } else {
            practiceTilt = gPracticeBalanceTilt;
        }
        command_continuous_tilt(practiceTilt, false);
    }

    if (gRunRequested &&
        ((!gHaveRealVision &&
            ((uint32_t) (nowMs - gRunRequestMs) >
                APP_BALL_VISION_STALE_MS)) ||
        (gHaveRealVision &&
            ((uint32_t) (nowMs - gLastRealVisionMs) >
                APP_BALL_VISION_STALE_MS)))) {
        enter_vision_fault(nowMs);
    }
#endif

    ball_stepper_tick_5ms(nowMs);
    update_telemetry(nowMs, vision);
}

void ball_rod_step_isr(void)
{
    ball_stepper_step_isr();
}

BallRodTelemetry ball_rod_get_telemetry(void)
{
    BallRodTelemetry telemetry;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    telemetry = gTelemetry;
    if (primask == 0U) {
        __enable_irq();
    }
    return telemetry;
}
