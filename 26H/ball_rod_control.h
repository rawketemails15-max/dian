#ifndef BALL_ROD_CONTROL_H_
#define BALL_ROD_CONTROL_H_

#include "ball_protocol.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_ROD_DISARMED = 0,
    BALL_ROD_WAITING_VISION,
    BALL_ROD_ACTIVE,
    BALL_ROD_HOLD,
    BALL_ROD_VISION_FAULT,
    BALL_ROD_SAFETY_FAULT
} BallRodState;

typedef enum {
    BALL_MOTION_IDLE = 0,
    BALL_MOTION_CORRECTING,
    BALL_MOTION_HOLD_RED_LINE,
    BALL_MOTION_LEVELING,
    BALL_MOTION_CALIBRATION
} BallMotionPhase;

typedef enum {
    BALL_RECOVERY_NONE = 0,
    BALL_RECOVERY_STATIC_FRICTION,
    BALL_RECOVERY_ENDPOINT
} BallRecoveryPhase;

typedef enum {
    BALL_FAULT_NONE = 0,
    BALL_FAULT_VISION_STALE,
    BALL_FAULT_HARD_LIMIT,
    BALL_FAULT_EMERGENCY_STOP
} BallFaultReason;

typedef struct {
    BallRodState state;
    BallMotionPhase motionPhase;
    int32_t currentSteps;
    int32_t targetSteps;
    int32_t minimumReached;
    int32_t maximumReached;
    int16_t ballX;
    int16_t ballError;
    uint16_t targetXQ4;
    int16_t ballErrorQ4;
    int16_t filteredVelocity;
    int16_t continuousTiltQ8;
    int16_t frictionBoostQ8;
    uint32_t visionAgeMs;
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
    uint16_t runId;
    uint16_t tiltLimit;
    uint16_t eventCounter;
    uint8_t recoveryPhase;
    uint8_t armFrames;
    uint8_t event;
    uint8_t faultReason;
    bool practiceActive;
    uint32_t crcErrors;
    uint32_t sequenceDrops;
    uint32_t rxOverflows;
} BallRodTelemetry;

void ball_rod_init(uint32_t nowMs);
void ball_rod_set_target_x_q4(uint16_t targetXQ4);
bool ball_rod_enable_driver(uint32_t nowMs);
bool ball_rod_start(uint32_t nowMs);
void ball_rod_start_practice(uint32_t nowMs);
void ball_rod_set_chassis_accel_compensation_steps(float steps);
void ball_rod_pause(void);
void ball_rod_emergency_stop(void);
void ball_rod_tick_5ms(
    uint32_t nowMs, bool buttonPressed, const BallVisionSample *vision);
void ball_rod_step_isr(void);
BallRodTelemetry ball_rod_get_telemetry(void);

#endif
