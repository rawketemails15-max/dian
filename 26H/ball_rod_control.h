#ifndef BALL_ROD_CONTROL_H_
#define BALL_ROD_CONTROL_H_

#include "ball_protocol.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_ROD_CALIBRATION = 0,
    BALL_ROD_WAITING_VISION,
    BALL_ROD_ACTIVE,
    BALL_ROD_RETURNING,
    BALL_ROD_VISION_FAULT,
    BALL_ROD_SAFETY_FAULT
} BallRodState;

typedef struct {
    BallRodState state;
    int32_t currentSteps;
    int32_t targetSteps;
    int32_t minimumReached;
    int32_t maximumReached;
    int16_t ballX;
    int16_t ballError;
    uint16_t stepFrequencyHz;
    uint8_t directionLevel;
    bool stepRunning;
    uint32_t crcErrors;
    uint32_t sequenceDrops;
    uint32_t rxOverflows;
} BallRodTelemetry;

void ball_rod_init(uint32_t nowMs);
void ball_rod_tick_5ms(
    uint32_t nowMs, bool buttonPressed, const BallVisionSample *vision);
void ball_rod_step_isr(void);
BallRodTelemetry ball_rod_get_telemetry(void);

#endif
