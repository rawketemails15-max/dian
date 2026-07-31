#include "ball_rod_control.h"

#include "app_config.h"
#include "ball_stepper.h"
#include "ti_msp_dl_config.h"

#define BALL_VISION_FLAG_PREDICTED (0x02U)
#define FLOAT_TWO_PI                (6.2831853f)

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
static uint8_t gStuckFrames;
static bool gHoldLatched;
static bool gRunRequested;
static bool gSafetyLatched;
static int32_t gMinimumReached;
static int32_t gMaximumReached;

static bool gButtonRaw;
static bool gButtonStable;
static bool gButtonEmergencyHandled;
static uint32_t gButtonRawChangedMs;
static uint32_t gButtonPressedMs;
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
    gStuckFrames = 0U;
    gHoldLatched = false;
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

    if (gButtonStable && !gButtonEmergencyHandled &&
        ((uint32_t) (nowMs - gButtonPressedMs) >=
            APP_BALL_BUTTON_ESTOP_MS)) {
        gButtonEmergencyHandled = true;
        emergency_stop();
    }

#if APP_BALL_CALIBRATION_MODE
    if (gCalibrationClickPending &&
        ((uint32_t) (nowMs - gCalibrationFirstClickMs) >
            APP_BALL_DOUBLE_CLICK_MS)) {
        gCalibrationClickPending = false;
        calibration_jog(nowMs, 1);
    }
#endif
}

static void enter_vision_fault(void)
{
    BallStepperStatus stepper;

    ball_stepper_hold();
    stepper = ball_stepper_get_status();
    reset_control_history(stepper.currentSteps, 0U);
    gRealArmFrames = 0U;
    if (gTelemetry.state != BALL_ROD_VISION_FAULT) {
        publish_event(BALL_STATUS_EVENT_VISION_FAULT);
    }
    gTelemetry.state = BALL_ROD_VISION_FAULT;
    gTelemetry.motionPhase = BALL_MOTION_IDLE;
    gTelemetry.faultReason = BALL_FAULT_VISION_STALE;
}

static void update_velocity_filter(int16_t measuredVelocity, uint32_t dtMs)
{
    float dtSeconds;
    float coefficient;

    if (dtMs < APP_BALL_OUTER_DT_MIN_MS) {
        dtMs = APP_BALL_OUTER_DT_MIN_MS;
    } else if (dtMs > APP_BALL_OUTER_DT_MAX_MS) {
        dtMs = APP_BALL_OUTER_DT_MAX_MS;
    }
    dtSeconds = (float) dtMs * 0.001f;
    coefficient = FLOAT_TWO_PI * APP_BALL_VELOCITY_FILTER_HZ * dtSeconds;
    coefficient = coefficient / (1.0f + coefficient);
    gFilteredVelocity +=
        coefficient * ((float) measuredVelocity - gFilteredVelocity);
}

static float apply_rough_tube_compensation(float baseTilt, int16_t errorQ4)
{
    float mappedError = (float) (APP_BALL_POSITION_TO_TILT_SIGN * errorQ4);
    float errorDirection = (mappedError >= 0.0f) ? 1.0f : -1.0f;
    float breakaway = (errorDirection > 0.0f) ?
        APP_BALL_BREAKAWAY_STEPS_POS : APP_BALL_BREAKAWAY_STEPS_NEG;
    bool nearlyStopped =
        abs_float(gFilteredVelocity) <= APP_BALL_STUCK_SPEED_PX_S;
    bool movingTowardTarget =
        ((float) errorQ4 * gFilteredVelocity) > 0.0f;

    if ((gFrictionBoost * errorDirection) < 0.0f) {
        gFrictionBoost = 0.0f;
    }
    if (nearlyStopped) {
        if (gStuckFrames < 255U) {
            gStuckFrames++;
        }
    } else {
        gStuckFrames = 0U;
    }

    if ((gStuckFrames >= APP_BALL_STUCK_REAL_FRAMES) &&
        (abs_float(gFrictionBoost) < APP_BALL_FRICTION_MAX_STEPS)) {
        gFrictionBoost +=
            errorDirection * APP_BALL_FRICTION_INCREMENT_STEPS;
        gFrictionBoost = clamp_float(gFrictionBoost,
            -APP_BALL_FRICTION_MAX_STEPS,
            APP_BALL_FRICTION_MAX_STEPS);
        gStuckFrames = 0U;
        publish_event(BALL_STATUS_EVENT_RECOVERY_REAPPLY);
    } else if (movingTowardTarget && !nearlyStopped) {
        gFrictionBoost = move_toward(
            gFrictionBoost, 0.0f, APP_BALL_FRICTION_DECAY_STEPS);
    }

    /*
     * Minimum effective tilt applies only while almost stationary.  When the
     * ball is moving, the signed D term remains free to brake before crossing
     * the red line.
     */
    if (nearlyStopped && (abs_float(baseTilt) < breakaway)) {
        baseTilt = errorDirection * breakaway;
    }
    return baseTilt + gFrictionBoost;
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
        gTiltResidual = 0.0f;
        gStuckFrames = 0U;
        command_continuous_tilt(APP_BALL_NEUTRAL_STEPS, true);
        return;
    }

    gTelemetry.state = BALL_ROD_ACTIVE;
    gTelemetry.motionPhase = BALL_MOTION_CORRECTING;
    desiredTilt = (float) APP_BALL_POSITION_TO_TILT_SIGN *
        (APP_BALL_POSITION_KP * errorPixels -
            APP_BALL_POSITION_KD * gFilteredVelocity);
    desiredTilt = apply_rough_tube_compensation(desiredTilt, errorQ4);
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
    gTelemetry.mustCorrect =
        abs_i32(gTelemetry.ballErrorQ4) >
            APP_BALL_HOLD_ENTER_ERROR_Q4;
    gTelemetry.approachingCenter =
        ((float) gTelemetry.ballErrorQ4 * gFilteredVelocity) > 0.0f;
    gTelemetry.recoveryActive = abs_float(gFrictionBoost) > 0.0f;
    gTelemetry.recoveryPhase = gTelemetry.recoveryActive ?
        BALL_RECOVERY_STATIC_FRICTION : BALL_RECOVERY_NONE;
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
    gLastOuterUpdateMs = nowMs;
    gRealArmFrames = 0U;
    gFilteredVelocity = 0.0f;
    gContinuousTilt = 0.0f;
    gTiltResidual = 0.0f;
    gFrictionBoost = 0.0f;
    gStuckFrames = 0U;
    gHoldLatched = false;
    gRunRequested = false;
    gSafetyLatched = false;
    gMinimumReached = 0;
    gMaximumReached = 0;
    gButtonRaw = false;
    gButtonStable = false;
    gButtonEmergencyHandled = false;
    gButtonRawChangedMs = nowMs;
    gButtonPressedMs = nowMs;
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

    if (gRunRequested &&
        ((!gHaveRealVision &&
            ((uint32_t) (nowMs - gRunRequestMs) >
                APP_BALL_VISION_STALE_MS)) ||
        (gHaveRealVision &&
            ((uint32_t) (nowMs - gLastRealVisionMs) >
                APP_BALL_VISION_STALE_MS)))) {
        enter_vision_fault();
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
