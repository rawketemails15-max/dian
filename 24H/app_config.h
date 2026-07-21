#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <stdint.h>

#define APP_BUTTON_ACTIVE_LEVEL              (1U)
#define APP_BUTTON_DEBOUNCE_MS               (30U)
#define APP_BUTTON_LONG_PRESS_MS             (800U)

#define APP_BUZZER_PULSE_MS                  (200U)

#define APP_CONTROL_PERIOD_MS                (20U)
#define APP_ENCODER_STALL_TIMEOUT_MS         (300U)
#define APP_MOTOR_TARGET_MAX_COUNTS          (20U)

#define APP_PWM_PERIOD_TICKS                 (1600U)
#define APP_PWM_FEEDFORWARD_MIN_TICKS        (160U)
#define APP_PWM_FEEDFORWARD_TICKS_PER_COUNT  (8U)
#define APP_PWM_MAX_TICKS                    (960U)
#define APP_PI_KP_TICKS_PER_COUNT            (8)
#define APP_PI_KI_TICKS_PER_COUNT_STEP       (1)
#define APP_PI_INTEGRAL_MIN_TICKS            (-320)
#define APP_PI_INTEGRAL_MAX_TICKS            (640)

/* Motor A is the right wheel; Motor B is the left wheel. */
/* Forward polarity confirmed on the installed chassis. */
#define APP_MOTOR_A_FORWARD_D1               (0U)
#define APP_MOTOR_A_FORWARD_D2               (1U)
#define APP_MOTOR_B_FORWARD_D1               (0U)
#define APP_MOTOR_B_FORWARD_D2               (1U)

#define APP_LINE_CHANNEL_COUNT               (8U)
#define APP_LINE_ADDRESS_SETTLE_MS           (1U)
#define APP_LINE_BASELINE_SCAN_COUNT         (5U)
#define APP_LINE_BASELINE_REQUIRED_MATCHES   (4U)
#define APP_LINE_CONFIRM_SCAN_COUNT          (2U)
#define APP_LINE_STOP_SCAN_COUNT             (2U)

#define APP_TRACK_POSITION_SCALE             (256)
#define APP_TRACK_BASE_TARGET_COUNTS         (36U)
#define APP_TRACK_BASE_RAMP_COUNTS           (2U)
#define APP_TRACK_MAX_CORRECTION_COUNTS      (8)
#define APP_TRACK_SEARCH_BASE_COUNTS         (10U)
#define APP_TRACK_SEARCH_CORRECTION_COUNTS   (6)
#define APP_C_WHITE_CONFIRM_MS               (1000U)
/* Keep searching longer than the C-point white confirmation window. */
#define APP_TRACK_LOST_TIMEOUT_MS            (1200U)
#define APP_TRACK_KP_NUMERATOR               (1)
#define APP_TRACK_KD_NUMERATOR               (1)

#define APP_FAULT_LED_TOGGLE_MS              (100U)

#endif
