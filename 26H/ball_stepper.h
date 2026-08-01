#ifndef BALL_STEPPER_H_
#define BALL_STEPPER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t currentSteps;
    int32_t targetSteps;
    uint16_t stepHz;
    uint8_t directionLevel;
    bool enabled;
    bool running;
    bool atLimit;
} BallStepperStatus;

void ball_stepper_init(uint32_t nowMs);
void ball_stepper_enable(uint32_t nowMs);
void ball_stepper_disarm(void);
void ball_stepper_hold(void);
void ball_stepper_set_target(int32_t targetSteps);
void ball_stepper_set_requested_hz(uint16_t stepHz);
void ball_stepper_tick_5ms(uint32_t nowMs);
void ball_stepper_step_isr(void);
BallStepperStatus ball_stepper_get_status(void);

#endif
