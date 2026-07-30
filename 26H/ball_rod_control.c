#include "ball_rod_control.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

#include <limits.h>

static volatile int32_t gCurrentSteps;
static volatile int32_t gTargetSteps;
static volatile bool gStepRunning;
static volatile int8_t gStepSign;

static BallRodTelemetry gTelemetry;
static uint16_t gStepFrequencyHz;
static uint16_t gCommandFrequencyHz;
static uint16_t gLastVisionSequence;
static bool gHaveVisionSequence;
static uint32_t gEnableStartMs;
static bool gDriverEnabled;

static bool gButtonRaw;
static bool gButtonStable;
static bool gButtonEmergencyHandled;
static uint32_t gButtonRawChangedMs;
static uint32_t gButtonPressedMs;
static bool gCalibrationPositiveDirection = true;

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
static void update_pid_target(const BallVisionSample *vision)
{
    int32_t x = (int32_t) ((vision->xQ4 + 8U) >> 4U);
    int32_t error = APP_BALL_IMAGE_CENTER_X - x;
    uint32_t errorMagnitude;
    int32_t increment;
    int32_t desiredTarget;
    int32_t targetChange;

    /*
     * Treat 150..170 (inclusive) as the accepted center window.  Suppress
     * both position and velocity correction there so vision jitter cannot
     * keep rocking the rod; the slewed target then returns to level.
     */
    if ((error >= -APP_BALL_CENTER_DEADBAND_PX) &&
        (error <= APP_BALL_CENTER_DEADBAND_PX)) {
        error = 0;
    }
    errorMagnitude = absolute_i32(error);

    /*
     * Continue changing the rod target on every off-center vision frame.
     * Error magnitude selects both the target increment and pulse frequency;
     * ball velocity is deliberately not used for early braking.
     */
    if (error == 0) {
        desiredTarget = 0;
        gCommandFrequencyHz = APP_BALL_FREQUENCY_NEAR_HZ;
    } else {
        if (errorMagnitude > APP_BALL_ERROR_LARGE_PX) {
            increment = APP_BALL_INCREMENT_FAR_STEPS;
            gCommandFrequencyHz = APP_BALL_FREQUENCY_FAR_HZ;
        } else if (errorMagnitude > APP_BALL_ERROR_MEDIUM_PX) {
            increment = APP_BALL_INCREMENT_MEDIUM_STEPS;
            gCommandFrequencyHz = APP_BALL_FREQUENCY_MEDIUM_HZ;
        } else {
            increment = APP_BALL_INCREMENT_NEAR_STEPS;
            gCommandFrequencyHz = APP_BALL_FREQUENCY_NEAR_HZ;
        }
        desiredTarget = gTargetSteps +
            ((error > 0) ? increment : -increment);
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
    gCommandFrequencyHz = APP_BALL_FREQUENCY_NEAR_HZ;
    gLastVisionSequence = 0U;
    gHaveVisionSequence = false;
    gButtonRaw = false;
    gButtonStable = false;
    gButtonEmergencyHandled = false;
    gButtonRawChangedMs = nowMs;
    gButtonPressedMs = nowMs;
    gCalibrationPositiveDirection = true;
    gTelemetry = (BallRodTelemetry) {0};
    gTelemetry.minimumReached = 0;
    gTelemetry.maximumReached = 0;

    step_pin_gpio_low();
    DL_GPIO_clearPins(GPIO_BALL_DIR_PORT, GPIO_BALL_DIR_DIR_PIN);
    DL_GPIO_clearPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
    gDriverEnabled = false;

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
    (void) buttonPressed;
    if (vision->received &&
        (!gHaveVisionSequence ||
         (vision->sequence != gLastVisionSequence))) {
        gLastVisionSequence = vision->sequence;
        gHaveVisionSequence = true;
        if (vision->valid) {
            update_pid_target(vision);
        }
    }

    if (!gDriverEnabled) {
        if (vision->valid &&
            (vision->validStreak >= APP_BALL_VALID_FRAMES_TO_ARM)) {
            set_direction_for_sign(1);
            DL_GPIO_setPins(GPIO_D36A_EN_PORT, GPIO_D36A_EN_EN_PIN);
            gDriverEnabled = true;
            gEnableStartMs = nowMs;
        } else {
            gTelemetry.state = BALL_ROD_WAITING_VISION;
        }
    } else if (!vision->received ||
        elapsed_ms(nowMs, vision->lastValidMs,
            APP_BALL_VISION_FAULT_MS)) {
        gTargetSteps = 0;
        gCommandFrequencyHz = APP_BALL_FREQUENCY_NEAR_HZ;
        gTelemetry.state = BALL_ROD_VISION_FAULT;
    } else if (elapsed_ms(nowMs, vision->lastValidMs,
                   APP_BALL_VISION_RETURN_MS)) {
        gTargetSteps = 0;
        gCommandFrequencyHz = APP_BALL_FREQUENCY_NEAR_HZ;
        gTelemetry.state = BALL_ROD_RETURNING;
    } else {
        gTelemetry.state = BALL_ROD_ACTIVE;
    }

    if (gDriverEnabled &&
        elapsed_ms(nowMs, gEnableStartMs, APP_D36A_WAKE_DELAY_MS)) {
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
    gTelemetry.stepFrequencyHz = gStepFrequencyHz;
    gTelemetry.stepRunning = gStepRunning;
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
