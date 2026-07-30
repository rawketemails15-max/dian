#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/*
 * 26H requirement 2 tuning constants.
 *
 * Encoder targets are counts measured over each 20 ms PI update.  The four
 * track constants below are the intended on-car tuning surface; the state
 * machine and its safety gates do not need to change during normal tuning.
 */
#define APP_CONTROL_TICK_MS                    (5U)
#define APP_MOTOR_UPDATE_MS                    (20U)
#define APP_OLED_UPDATE_MS                     (100U)

#define APP_BUTTON_ACTIVE_LEVEL                (1U)
#define APP_BUTTON_DEBOUNCE_MS                 (30U)
#define APP_BALL_DOUBLE_CLICK_MS               (1000U)

/*
 * Grayscale position PID, executed every 5 ms.  Gains are expressed directly
 * in PWM ticks per error unit/tick so the controller remains integer-only.
 * The I term is deliberately small and active only near the line center.
 */
#define APP_TRACK_BASE_PWM                      (3000U)
#define APP_TRACK_FINAL_PWM                     (1800U)
#define APP_TRACK_CENTER_DEADBAND                (60)
#define APP_TRACK_ERROR_FILTER_DIVISOR           (4)
#define APP_TRACK_PWM_SLEW_PER_TICK              (200)
#define APP_LINE_PID_KP                          (5)
#define APP_LINE_PID_KI_NUMERATOR                (1)
#define APP_LINE_PID_KI_DIVISOR                  (64)
#define APP_LINE_PID_KD                          (10)
#define APP_LINE_PID_D_FILTER_DIVISOR            (4)
#define APP_LINE_PID_INTEGRAL_ACTIVE_ERROR       (150)
#define APP_LINE_PID_INTEGRAL_LIMIT              (12800)
#define APP_LINE_PID_OUTPUT_LIMIT                (3000)

/* Retained for the optional encoder-speed PI interface. */
#define APP_MOTOR_TARGET_MAX_COUNTS            (68U)

#define APP_LINE_LOST_TIMEOUT_MS               (100U)
#define APP_ENCODER_STALL_TIMEOUT_MS           (300U)
#define APP_RUN_TIMEOUT_MS                     (20000U)

#define APP_START_MARKER_CLEAR_MS              (50U)
#define APP_START_MARKER_CLEAR_COUNTS          (300U)
#define APP_FINISH_ARM_COUNTS                  (38000U)
#define APP_FINISH_ARM_MS                      (10000U)
#define APP_FINAL_APPROACH_COUNTS              (45500U)
#define APP_MARKER_MIN_CONTIGUOUS_CHANNELS     (4U)
#define APP_MARKER_CONFIRM_SCANS               (2U)

#define APP_LINE_CHANNEL_COUNT                 (8U)
#define APP_LINE_MUX_SETTLE_US                 (100U)
#define APP_LINE_BLACK_LEVEL                   (1U)
#define APP_LINE_REVERSE_ORDER                 (0U)

/* PI output uses a logical 1600-tick domain and is scaled to TIMG6/8000. */
#define APP_PWM_PERIOD_TICKS                   (1600U)
#define APP_PWM_FEEDFORWARD_MIN_TICKS          (160U)
#define APP_PWM_FEEDFORWARD_TICKS_PER_COUNT    (8U)
#define APP_PWM_MAX_TICKS                      (1440U)
#define APP_PI_KP_TICKS_PER_COUNT              (8)
#define APP_PI_KI_TICKS_PER_COUNT_STEP         (1)
#define APP_PI_INTEGRAL_MIN_TICKS              (-320)
#define APP_PI_INTEGRAL_MAX_TICKS              (640)

/* Motor A is the right wheel; Motor B is the left wheel. */
/* Forward polarity is confirmed on the installed C07A + S28A chassis. */
#define APP_MOTOR_A_FORWARD_D1                 (0U)
#define APP_MOTOR_A_FORWARD_D2                 (1U)
#define APP_MOTOR_B_FORWARD_D1                 (0U)
#define APP_MOTOR_B_FORWARD_D2                 (1U)

#define APP_FAULT_LED_TOGGLE_MS                (100U)

/*
 * Application mode.
 *
 * The ball-static mode deliberately leaves the completed line-following
 * implementation in this project, but keeps both chassis motors braked.
 */
#define APP_OPERATION_MODE_LINE_FOLLOW         (0U)
#define APP_OPERATION_MODE_BALL_STATIC         (1U)
#define APP_OPERATION_MODE                     APP_OPERATION_MODE_BALL_STATIC

/*
 * First installation must remain in calibration mode.  Remove the ball and
 * manually level the rod before power-on.  Record the safe travel displayed
 * on the OLED, subtract 16 steps at each end, update MIN/MAX and DIR_INVERT,
 * then set APP_BALL_CALIBRATION_MODE to 0 and rebuild.
 */
#define APP_BALL_CALIBRATION_MODE              (0U)
#define APP_BALL_MIN_STEPS                     (-238)
#define APP_BALL_MAX_STEPS                     (238)
#define APP_BALL_DIR_INVERT                     (0U)
#define APP_BALL_CALIBRATION_HARD_LIMIT         (256)
#define APP_BALL_CALIBRATION_JOG_STEPS          (16)
#define APP_BALL_BUTTON_LONG_MS                 (600U)
#define APP_BALL_BUTTON_ESTOP_MS                (2000U)

#define APP_BALL_IMAGE_CENTER_X                 (160)
#define APP_BALL_CENTER_DEADBAND_PX             (5)
#define APP_BALL_CENTER_RELEASE_PX              (8)
#define APP_BALL_CENTER_SETTLE_SPEED_PX_S       (5)
#define APP_BALL_CENTER_RELEASE_SPEED_PX_S      (12)
#define APP_BALL_CENTER_STABLE_REAL_FRAMES      (15U)
#define APP_BALL_CENTER_VELOCITY_FILTER_DIVISOR (4)
#define APP_BALL_CENTER_STUCK_REAL_FRAMES       (5U)
#define APP_BALL_CENTER_INTEGRAL_DIVISOR        (64)
#define APP_BALL_CENTER_INTEGRAL_LIMIT          (2560)
#define APP_BALL_CENTER_INTEGRAL_DECAY_DIVISOR  (4)
#define APP_BALL_VALID_FRAMES_TO_ARM            (5U)
#define APP_BALL_VISION_RETURN_MS               (150U)
#define APP_BALL_VISION_FAULT_MS                (1000U)

/*
 * Static-car requirement-2 trajectory.  The 25 cm rod spans the 320-pixel
 * calibrated image axis, so 5 cm is 64 pixels.  The positive turnaround uses
 * a 12-pixel band; the final -5 cm hold uses the requested tighter +/-6-pixel
 * band.  Image X increases rightward and defines the positive direction.
 */
#define APP_BALL_POSITIVE_5CM_X                 (224)
#define APP_BALL_NEGATIVE_5CM_X                 (96)
#define APP_BALL_POSITIVE_TOLERANCE_PX          (12)
#define APP_BALL_FINAL_TOLERANCE_PX             (6)
#define APP_BALL_SEQUENCE_TIMEOUT_MS            (5000U)
#define APP_BALL_REVERSAL_LEVEL_MS              (150U)
#define APP_BALL_LEVEL_SLEW_STEPS_PER_TICK      (2)

/*
 * li-style cascade controller.
 * Outer position PD -> target rod angle (relative microsteps).
 * Inner step-position loop -> STEP frequency and direction.
 * KI remains zero as in the li ball-and-beam position loop.
 */
#define APP_BALL_POSITION_KP_NUMERATOR          (1)
#define APP_BALL_POSITION_KP_DIVISOR            (2)
#define APP_BALL_POSITION_KD_NUMERATOR          (1)
#define APP_BALL_POSITION_KD_DIVISOR            (25)
#define APP_BALL_D_FILTER_DIVISOR                (4)
#define APP_BALL_OUTER_DT_MIN_MS                 (10U)
#define APP_BALL_OUTER_DT_MAX_MS                 (100U)
#define APP_BALL_ANGLE_TARGET_LIMIT_STEPS       (60)
#define APP_BALL_TARGET_SLEW_STEPS_PER_FRAME    (6)
#define APP_BALL_MIN_EFFECTIVE_TILT_STEPS       (8)
#define APP_BALL_SPEED_NEAR_ERROR_PX            (15)
#define APP_BALL_SPEED_MEDIUM_ERROR_PX          (40)
#define APP_BALL_SPEED_NEAR_HZ                  (80U)
#define APP_BALL_SPEED_MEDIUM_HZ                (160U)
#define APP_BALL_SPEED_FAR_HZ                   (260U)

#define APP_BALL_STEP_CLOCK_HZ                  (4000000U)
#define APP_BALL_STEP_MIN_HZ                    (80U)
#define APP_BALL_STEP_MAX_HZ                    (300U)
#define APP_BALL_STEP_HZ_PER_ERROR              (5U)
#define APP_BALL_STEP_HZ_SLEW_PER_TICK          (3U)
#define APP_BALL_POSITION_TOLERANCE_STEPS       (2)
#define APP_D36A_WAKE_DELAY_MS                  (1U)

#define APP_BALL_LED_WAIT_TOGGLE_MS             (500U)
#define APP_BALL_LED_FAULT_TOGGLE_MS            (100U)

#endif
