#ifndef Q3_APP_CONFIG_H_
#define Q3_APP_CONFIG_H_

/*
 * ================================================================
 * OPEN LOOP USER TUNING
 * ================================================================
 *
 * Electrically trim the pipe horizontal before every double-click start,
 * then put the ball at O.  Change only one parameter at a time.
 */
#define APP_BALL_DIR_INVERT                    (0U)
#define APP_OL_LAUNCH_STEPS                    (112)
#define APP_OL_LAUNCH_HOLD_MS                  (0U)
#define APP_OL_REVERSE_STEPS                   (-48)
#define APP_OL_RETURN_HOLD_MS                  (1000U)
#define APP_OL_LAUNCH_HZ                       (240U)
#define APP_OL_REVERSE_HZ                      (300U)
#define APP_OL_LEVEL_HZ                        (240U)
#define APP_OL_FINAL_SETTLE_MS                 (1000U)
#define APP_OL_RUN_TIMEOUT_MS                  (4500U)
#define APP_TRIM_STEP_SIZE                     (4)
#define APP_TRIM_HZ                            (120U)
#define APP_BUTTON_LONG_PRESS_MS               (1000U)
#define APP_BUTTON_DOUBLE_CLICK_MS             (350U)

/* Fixed control, hardware and safety constants. */
#define APP_CONTROL_TICK_MS                    (5U)
#define APP_OLED_UPDATE_MS                     (100U)
#define APP_BUTTON_ACTIVE_LEVEL                (1U)
#define APP_BUTTON_DEBOUNCE_MS                 (30U)
#define APP_FAULT_LED_TOGGLE_MS                (100U)

#define APP_BALL_MIN_STEPS                     (-238)
#define APP_BALL_MAX_STEPS                     (238)
#define APP_BALL_STEP_CLOCK_HZ                 (4000000U)
#define APP_BALL_STEP_MIN_HZ                   (80U)
#define APP_BALL_STEP_MAX_HZ                   (300U)
#define APP_D36A_WAKE_DELAY_MS                 (2U)

/*
 * Theoretical motion time.  Holds start only after the commanded step
 * position has actually been reached.
 */
#define APP_OL_LAUNCH_MOVE_MS \
    ((((APP_OL_LAUNCH_STEPS) * 1000U) + APP_OL_LAUNCH_HZ - 1U) / \
        APP_OL_LAUNCH_HZ)
#define APP_OL_REVERSE_MOVE_STEPS \
    ((APP_OL_LAUNCH_STEPS) - (APP_OL_REVERSE_STEPS))
#define APP_OL_REVERSE_MOVE_MS \
    (((APP_OL_REVERSE_MOVE_STEPS * 1000U) + APP_OL_REVERSE_HZ - 1U) / \
        APP_OL_REVERSE_HZ)
#define APP_OL_LEVEL_MOVE_STEPS                (-(APP_OL_REVERSE_STEPS))
#define APP_OL_LEVEL_MOVE_MS \
    (((APP_OL_LEVEL_MOVE_STEPS * 1000U) + APP_OL_LEVEL_HZ - 1U) / \
        APP_OL_LEVEL_HZ)
#define APP_OL_THEORETICAL_DURATION_MS \
    (APP_D36A_WAKE_DELAY_MS + APP_OL_LAUNCH_MOVE_MS + \
        APP_OL_LAUNCH_HOLD_MS + APP_OL_REVERSE_MOVE_MS + \
        APP_OL_RETURN_HOLD_MS + APP_OL_LEVEL_MOVE_MS + \
        APP_OL_FINAL_SETTLE_MS)

/* Reject unsafe or contradictory manual settings at compile time. */
#if (APP_BALL_DIR_INVERT > 1U)
#error "APP_BALL_DIR_INVERT must be 0U or 1U"
#endif

#if (APP_OL_LAUNCH_STEPS <= 0)
#error "APP_OL_LAUNCH_STEPS must be positive"
#endif

#if (APP_OL_REVERSE_STEPS >= 0)
#error "APP_OL_REVERSE_STEPS must be negative"
#endif

#if ((APP_OL_LAUNCH_STEPS > APP_BALL_MAX_STEPS) || \
     (APP_OL_LAUNCH_STEPS < APP_BALL_MIN_STEPS))
#error "APP_OL_LAUNCH_STEPS exceeds the calibrated soft travel"
#endif

#if ((APP_OL_REVERSE_STEPS > APP_BALL_MAX_STEPS) || \
     (APP_OL_REVERSE_STEPS < APP_BALL_MIN_STEPS))
#error "APP_OL_REVERSE_STEPS exceeds the calibrated soft travel"
#endif

#if ((APP_OL_LAUNCH_HZ < APP_BALL_STEP_MIN_HZ) || \
     (APP_OL_LAUNCH_HZ > APP_BALL_STEP_MAX_HZ))
#error "APP_OL_LAUNCH_HZ must be between 80 and 300 Hz"
#endif

#if ((APP_OL_REVERSE_HZ < APP_BALL_STEP_MIN_HZ) || \
     (APP_OL_REVERSE_HZ > APP_BALL_STEP_MAX_HZ))
#error "APP_OL_REVERSE_HZ must be between 80 and 300 Hz"
#endif

#if ((APP_OL_LEVEL_HZ < APP_BALL_STEP_MIN_HZ) || \
     (APP_OL_LEVEL_HZ > APP_BALL_STEP_MAX_HZ))
#error "APP_OL_LEVEL_HZ must be between 80 and 300 Hz"
#endif

#if (APP_TRIM_STEP_SIZE <= 0)
#error "APP_TRIM_STEP_SIZE must be positive"
#endif

#if ((APP_TRIM_HZ < APP_BALL_STEP_MIN_HZ) || \
     (APP_TRIM_HZ > APP_BALL_STEP_MAX_HZ))
#error "APP_TRIM_HZ must be between 80 and 300 Hz"
#endif

#if ((APP_BUTTON_DOUBLE_CLICK_MS <= APP_BUTTON_DEBOUNCE_MS) || \
     (APP_BUTTON_LONG_PRESS_MS <= APP_BUTTON_DOUBLE_CLICK_MS))
#error "Button timing must satisfy debounce < double-click < long-press"
#endif

#if (APP_OL_RUN_TIMEOUT_MS >= 5000U)
#error "APP_OL_RUN_TIMEOUT_MS must remain below the five-second limit"
#endif

#if (APP_OL_THEORETICAL_DURATION_MS >= APP_OL_RUN_TIMEOUT_MS)
#error "Open-loop schedule does not fit inside APP_OL_RUN_TIMEOUT_MS"
#endif

#endif
