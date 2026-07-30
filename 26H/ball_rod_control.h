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

typedef enum {
    BALL_MOTION_IDLE = 0,
    BALL_MOTION_HOLD_CENTER,
    BALL_MOTION_CENTER_BEFORE_SEQUENCE,
    BALL_MOTION_TO_POSITIVE_5CM,
    BALL_MOTION_RETURN_TO_LEVEL,
    BALL_MOTION_TO_NEGATIVE_5CM,
    BALL_MOTION_HOLD_NEGATIVE_5CM
} BallMotionPhase;

typedef enum {
    BALL_CENTER_RECOVERY_NONE = 0,
    BALL_CENTER_RECOVERY_BACKOFF,
    BALL_CENTER_RECOVERY_REAPPLY,
    BALL_CENTER_RECOVERY_LEVELING
} BallCenterRecoveryPhase;

typedef struct {
    BallRodState state;
    BallMotionPhase motionPhase;
    int32_t currentSteps;
    int32_t targetSteps;
    int32_t minimumReached;
    int32_t maximumReached;
    int16_t ballX;
    int16_t ballError;
    uint16_t stepFrequencyHz;
    uint8_t directionLevel;
    bool stepRunning;
    bool sequenceTimedOut;
    bool driverEnabled;
    bool visionFresh;
    bool centerSettled;
    bool mustCorrect;
    bool approachingCenter;
    bool recoveryActive;
    bool limitReached;
    int16_t ballErrorQ4;
    int16_t filteredVelocity;
    uint16_t runId;
    uint16_t tiltLimit;
    uint16_t eventCounter;
    uint8_t recoveryPhase;
    uint8_t armFrames;
    uint8_t event;
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
