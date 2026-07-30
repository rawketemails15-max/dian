#ifndef BALL_ROD_CONTROL_H_
#define BALL_ROD_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_ROD_IDLE = 0,
    BALL_ROD_WAKE,
    BALL_ROD_LAUNCH,
    BALL_ROD_LAUNCH_HOLD,
    BALL_ROD_REVERSE,
    BALL_ROD_RETURN_HOLD,
    BALL_ROD_LEVEL,
    BALL_ROD_SETTLE,
    BALL_ROD_COMPLETE,
    BALL_ROD_TIMEOUT_LEVEL,
    BALL_ROD_FAULT
} BallRodState;

typedef struct {
    BallRodState state;
    int32_t currentSteps;
    int32_t targetSteps;
    uint32_t runElapsedMs;
    uint16_t stepFrequencyHz;
    uint8_t directionLevel;
    bool driverEnabled;
    bool stepRunning;
    bool sequenceTimedOut;
} BallRodTelemetry;

void ball_rod_init(uint32_t nowMs);
void ball_rod_tick_5ms(uint32_t nowMs, bool buttonPressed);
void ball_rod_step_isr(void);
BallRodTelemetry ball_rod_get_telemetry(void);

#endif
