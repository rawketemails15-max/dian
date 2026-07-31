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
 * Ball rod installation.
 *
 * The rod has no angle sensor, encoder or limit switch.  Level it by hand
 * before every power-up; software then calls that pose zero.  The +/-238
 * limits are unconditional actuator limits, independent of controller state.
 */
#define APP_BALL_CALIBRATION_MODE               (0U)
#define APP_BALL_MIN_STEPS                      (-238)
#define APP_BALL_MAX_STEPS                      (238)
#define APP_BALL_DIR_INVERT                     (0U)
#define APP_BALL_POSITION_TO_TILT_SIGN          (1)
#define APP_BALL_NEUTRAL_STEPS                  (0.0f)
#define APP_BALL_WORK_TILT_LIMIT_STEPS          (64)
#define APP_BALL_CALIBRATION_JOG_STEPS          (16)
#define APP_BALL_BUTTON_ESTOP_MS                (2000U)

/* Draggable red-line target: AI x=0..319 px, default x=171 px. */
#define APP_BALL_TARGET_MIN_X_Q4                (0U)
#define APP_BALL_TARGET_MAX_X_Q4                (5104U)
#define APP_BALL_DEFAULT_TARGET_X_Q4            (2736U)
#define APP_BALL_VALID_FRAMES_TO_ARM            (3U)
#define APP_BALL_VISION_STALE_MS                (150U)
#define APP_BALL_STATUS_PERIOD_MS               (50U)

/*
 * Position PD.  Output is a continuous relative rod angle in microsteps.
 * Ki is intentionally zero because there is no measured rod-angle inner loop.
 * These are conservative bench defaults and are the main real-car tuning
 * surface: first KP, then KD.
 */
#define APP_BALL_POSITION_KP                    (0.50f)
#define APP_BALL_POSITION_KD                    (0.040f)
#define APP_BALL_VELOCITY_FILTER_HZ             (3.0f)
#define APP_BALL_OUTER_DT_MIN_MS                (15U)
#define APP_BALL_OUTER_DT_MAX_MS                (100U)
#define APP_BALL_TARGET_SLEW_STEPS_PER_FRAME    (8.0f)
#define APP_BALL_LEVEL_SLEW_STEPS_PER_FRAME     (4.0f)

/* Hold-zone hysteresis prevents STEP chatter at the red line. */
#define APP_BALL_HOLD_ENTER_ERROR_Q4            (96)
#define APP_BALL_HOLD_RELEASE_ERROR_Q4          (160)
#define APP_BALL_HOLD_ENTER_SPEED_PX_S          (6.0f)
#define APP_BALL_HOLD_RELEASE_SPEED_PX_S        (14.0f)

/*
 * Rough-tube static-friction compensation.  Positive and negative breakaway
 * values are deliberately independent because the tube is not uniform.
 */
#define APP_BALL_BREAKAWAY_STEPS_POS            (64.0f)
#define APP_BALL_BREAKAWAY_STEPS_NEG            (64.0f)
#define APP_BALL_STUCK_SPEED_PX_S               (8.0f)
#define APP_BALL_STUCK_REAL_FRAMES              (4U)
#define APP_BALL_FRICTION_INCREMENT_STEPS       (16.0f)
#define APP_BALL_FRICTION_MAX_STEPS             (32.0f)
#define APP_BALL_FRICTION_DECAY_STEPS           (4.0f)

/* D36A STEP generator: arbitrary finite pulse packets with frequency ramp. */
#define APP_BALL_STEP_CLOCK_HZ                  (4000000U)
#define APP_BALL_STEP_MIN_HZ                    (80U)
#define APP_BALL_STEP_MAX_HZ                    (300U)
#define APP_BALL_STEP_HZ_PER_ERROR              (8U)
#define APP_BALL_STEP_HZ_SLEW_PER_TICK          (10U)
#define APP_D36A_WAKE_DELAY_MS                  (1U)

#define APP_BALL_LED_WAIT_TOGGLE_MS             (500U)
#define APP_BALL_LED_FAULT_TOGGLE_MS            (100U)

#endif
