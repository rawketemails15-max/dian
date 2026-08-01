#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/*
 * 26H line-following tuning constants.
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
#define APP_MENU_DOUBLE_CLICK_MS               (300U)
#define APP_Q56_DRIVE_DOUBLE_CLICK_MS          (1000U)
#define APP_BUTTON_IGNORE_LONG_MS              (1000U)

/*
 * Grayscale position PID, executed every 5 ms.  Gains are expressed directly
 * in PWM ticks per error unit/tick so the controller remains integer-only.
 * The I term is deliberately small and active only near the line center.
 */
#define APP_Q2_TRACK_BASE_PWM                    (3500U)
#define APP_Q2_TRACK_FINAL_PWM                   (1800U)
#define APP_Q2_TRACK_PWM_SLEW_PER_TICK            (200)
#define APP_Q2_LINE_PID_KP                        (5)
#define APP_Q2_LINE_PID_KD                        (10)

#define APP_Q456_TRACK_BASE_PWM                  (3050U)
#define APP_Q456_TRACK_FINAL_PWM                 (1000U)
#define APP_Q456_TRACK_PWM_SLEW_PER_TICK          (100)
#define APP_Q456_LINE_PID_KP                      (11)
#define APP_Q456_LINE_PID_KD                      (22)

/* Questions 5/6 use a slower chassis profile to target a 27 s lap. */
#define APP_Q56_TRACK_BASE_PWM                   (2550U)
#define APP_Q56_TRACK_FINAL_PWM                  (1000U)
#define APP_Q56_TRACK_PWM_SLEW_PER_TICK           (100)
#define APP_Q56_LINE_PID_KP                        (9)
#define APP_Q56_LINE_PID_KD                       (18)

#define APP_TRACK_CENTER_DEADBAND                  (60)
#define APP_TRACK_ERROR_FILTER_DIVISOR             (4)
#define APP_LINE_PID_KI_NUMERATOR                  (1)
#define APP_LINE_PID_KI_DIVISOR                    (64)
#define APP_LINE_PID_D_FILTER_DIVISOR              (4)
#define APP_LINE_PID_INTEGRAL_ACTIVE_ERROR         (150)
#define APP_LINE_PID_INTEGRAL_LIMIT                (12800)
#define APP_LINE_PID_OUTPUT_LIMIT                  (3000)

/* Retained for the optional encoder-speed PI interface. */
#define APP_MOTOR_TARGET_MAX_COUNTS            (68U)

#define APP_ENCODER_STALL_TIMEOUT_MS           (300U)
#define APP_Q2_RUN_TIMEOUT_MS                  (20000U)

#define APP_START_MARKER_CLEAR_MS              (50U)
#define APP_START_MARKER_CLEAR_COUNTS          (300U)
#define APP_FINISH_ARM_COUNTS                  (38000U)
#define APP_FINISH_ARM_MS                      (10000U)
#define APP_FINAL_APPROACH_COUNTS              (45500U)
#define APP_Q2_FINISH_RUNOUT_MS                (1000U)
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
 * QUESTION5 runs the completed line-following and ball-centering controllers
 * concurrently.  BALL_STATIC is retained as a bench-test mode.
 */
#define APP_OPERATION_MODE_LINE_FOLLOW         (0U)
#define APP_OPERATION_MODE_BALL_STATIC         (1U)
#define APP_OPERATION_MODE_QUESTION5           (2U)
#define APP_OPERATION_MODE                     APP_OPERATION_MODE_QUESTION5

/*
 * Stable chassis supervisor used only by questions 4, 5 and 6.  These modes
 * keep tracking until reset or power-off; only the startup ramp applies.
 * The common-mode PWM slope estimates forward chassis acceleration.  Positive
 * compensation requests the configured front-down direction.
 */
#define APP_Q456_START_PRELOAD_MS                 (100U)
#define APP_Q456_START_ACCEL_MS                   (2700U)
#define APP_Q56_START_ACCEL_MS                    (2250U)
#define APP_Q456_BALL_ACCEL_FF_DEADBAND_PWM_PER_TICK (2.0f)
#define APP_Q456_BALL_ACCEL_FF_GAIN_STEPS_PER_PWM_TICK (6.40f)
#define APP_Q456_BALL_ACCEL_FF_LIMIT_STEPS          (40.0f)
#define APP_Q456_BALL_ACCEL_FF_SLEW_STEPS_PER_TICK  (4.0f)

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
#define APP_BALL_PRACTICE_TARGET_POS5_Q4        (3776U)
#define APP_BALL_PRACTICE_TARGET_NEG5_Q4        (1696U)
#define APP_BALL_PRACTICE_TIMEOUT_MS            (5000U)
#define APP_BALL_PRACTICE_TOLERANCE_Q4          (240)

/* Red-line target: AI x=0..319 px, default x=171 px. */
#define APP_BALL_TARGET_MIN_X_Q4                (0U)
#define APP_BALL_TARGET_MAX_X_Q4                (5104U)
#define APP_BALL_DEFAULT_TARGET_X_Q4            (2736U)
#define APP_BALL_VALID_FRAMES_TO_ARM            (3U)
#define APP_BALL_VISION_STALE_MS                (150U)
#define APP_BALL_STATUS_PERIOD_MS               (50U)
#define APP_BALL_PROTOCOL_INTERBYTE_TIMEOUT_MS  (20U)

/*
 * Questions 4/5/6 ball controller, adapted from the open-source H-problem
 * BalanceController.  The source mechanism uses a ZDT absolute-angle motor;
 * this port keeps the same control state logic but expresses every output in
 * D36A relative microsteps around the manually levelled power-on zero.
 *
 * Rolling control is P plus short-window ball-velocity damping.  D action is
 * deliberately weak far from the target while the ball is approaching, then
 * fades to full strength before the red line.  Motion away from the target
 * always receives full damping.
 */
#define APP_Q456_BALL_ROLLING_KP_STEPS_PER_PX       (0.35f)
#define APP_Q456_BALL_STATIONARY_KP_STEPS_PER_PX    (0.50f)
#define APP_Q456_BALL_KD_STEPS_PER_PX_S             (0.12f)
#define APP_Q456_BALL_P_LIMIT_STEPS                  (32.0f)
#define APP_Q456_BALL_D_LIMIT_STEPS                  (24.0f)
#define APP_Q456_BALL_DYNAMIC_LIMIT_STEPS            (48.0f)
#define APP_Q456_BALL_BRAKE_START_ERROR_PX           (30.0f)
#define APP_Q456_BALL_BRAKE_FULL_ERROR_PX             (8.0f)
#define APP_Q456_BALL_BRAKE_FAR_SCALE                  (0.20f)

/* Integer-position least-squares velocity estimator (open-source: 150 ms). */
#define APP_Q456_BALL_VELOCITY_WINDOW_MS              (150U)
#define APP_Q456_BALL_VELOCITY_MIN_SPAN_MS              (80U)
#define APP_Q456_BALL_VELOCITY_FILTER_ALPHA             (0.30f)
#define APP_Q456_BALL_VELOCITY_SAMPLE_COUNT             (12U)

/* Motion classification, settled-angle latch and hysteretic release. */
#define APP_Q456_BALL_DEADBAND_PX                        (3.0f)
#define APP_Q456_BALL_SETTLED_HOLD_ERROR_PX              (5.0f)
#define APP_Q456_BALL_SETTLED_EXIT_ERROR_PX              (7.0f)
#define APP_Q456_BALL_STATIONARY_CONFIRM_MS             (350U)
#define APP_Q456_BALL_STATIONARY_SPEED_PX_S              (2.0f)
#define APP_Q456_BALL_ROLLING_SPEED_PX_S                 (6.0f)
#define APP_Q456_BALL_ROLLING_DISPLACEMENT_PX            (2.0f)
#define APP_Q456_BALL_SETTLED_EXIT_CONFIRM_MS            (150U)
#define APP_Q456_BALL_SETTLED_FAST_EXIT_SPEED_PX_S      (15.0f)

/* Progressive static-friction breakaway; cleared on target entry/crossing. */
#define APP_Q456_BALL_BREAKAWAY_ERROR_MIN_PX              (5.0f)
#define APP_Q456_BALL_BREAKAWAY_INITIAL_STEPS             (12.0f)
#define APP_Q456_BALL_BREAKAWAY_MAX_STEPS                 (28.0f)
#define APP_Q456_BALL_BREAKAWAY_ERROR_GAIN_STEPS_PER_PX    (1.0f)
#define APP_Q456_BALL_BREAKAWAY_RAMP_STEPS_PER_S          (40.0f)
#define APP_Q456_BALL_BREAKAWAY_ESCALATE_STEPS_PER_S       (4.0f)
#define APP_Q456_BALL_BREAKAWAY_RELEASE_STEPS_PER_S       (80.0f)

/* Slow neutral trim compensates tube/floor slope without PID integral windup. */
#define APP_Q456_BALL_TRIM_DEADBAND_PX                     (5.0f)
#define APP_Q456_BALL_TRIM_ACTIVATION_MS                  (200U)
#define APP_Q456_BALL_TRIM_VELOCITY_GATE_PX_S             (12.0f)
#define APP_Q456_BALL_TRIM_GAIN_STEPS_PER_PX_S             (0.08f)
#define APP_Q456_BALL_TRIM_MAX_RATE_STEPS_PER_S             (4.0f)
#define APP_Q456_BALL_TRIM_LIMIT_STEPS                      (8.0f)

/* Outer 10% endpoint guard; recover into the inner 80% before normal PID. */
#define APP_Q456_BALL_RAIL_LENGTH_PX                       (320.0f)
#define APP_Q456_BALL_ENDPOINT_MARGIN_RATIO                  (0.10f)
#define APP_Q456_BALL_ENDPOINT_RECOVERY_MARGIN_RATIO         (0.20f)
#define APP_Q456_BALL_ENDPOINT_RECOVERY_SPEED_PX_S          (20.0f)
#define APP_Q456_BALL_ENDPOINT_RECOVERY_CONFIRM_MS          (200U)
#define APP_Q456_BALL_ENDPOINT_RECOVERY_TILT_STEPS           (36.0f)

#define APP_BALL_OUTER_DT_MIN_MS                (15U)
#define APP_BALL_OUTER_DT_MAX_MS                (100U)
#define APP_BALL_TARGET_SLEW_STEPS_PER_FRAME    (4.0f)
#define APP_BALL_LEVEL_SLEW_STEPS_PER_FRAME     (4.0f)

/* Hold-zone hysteresis prevents STEP chatter at the red line. */
#define APP_BALL_HOLD_ENTER_ERROR_Q4            (96)
#define APP_BALL_HOLD_RELEASE_ERROR_Q4          (160)
#define APP_BALL_HOLD_ENTER_SPEED_PX_S          (6.0f)
#define APP_BALL_HOLD_RELEASE_SPEED_PX_S        (14.0f)

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
