#ifndef BALL_PROTOCOL_H_
#define BALL_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool received;
    bool valid;
    uint16_t sequence;
    uint8_t flags;
    uint8_t confidence;
    uint16_t xQ4;
    int16_t velocityX;
    uint32_t lastFrameMs;
    uint32_t lastValidMs;
    uint8_t validStreak;
    uint32_t crcErrors;
    uint32_t sequenceDrops;
    uint32_t rxOverflows;
} BallVisionSample;

typedef struct {
    bool received;
    uint16_t sequence;
    uint16_t targetXQ4;
    uint32_t lastCommandMs;
    uint32_t updateCounter;
    uint32_t crcErrors;
    uint32_t sequenceDrops;
    uint32_t formatErrors;
    uint32_t rxOverflows;
} BallTargetCommand;

typedef enum {
    BALL_STATUS_EVENT_NONE = 0,
    BALL_STATUS_EVENT_CENTER_START = 1,
    BALL_STATUS_EVENT_CENTER_SETTLED = 2,
    BALL_STATUS_EVENT_CORRECTION_RESUME = 3,
    BALL_STATUS_EVENT_RECOVERY_BACKOFF = 4,
    BALL_STATUS_EVENT_RECOVERY_REAPPLY = 5,
    BALL_STATUS_EVENT_VISION_FAULT = 6,
    BALL_STATUS_EVENT_CENTER_END = 7,
    BALL_STATUS_EVENT_DOUBLE_CLICK_OVERRIDE = 8,
    BALL_STATUS_EVENT_CENTER_LEVELING = 9
} BallStatusEvent;

#define BALL_STATUS_FLAG_DRIVER_ENABLED  (1U << 0)
#define BALL_STATUS_FLAG_STEP_RUNNING    (1U << 1)
#define BALL_STATUS_FLAG_VISION_FRESH    (1U << 2)
#define BALL_STATUS_FLAG_SETTLED         (1U << 3)
#define BALL_STATUS_FLAG_MUST_CORRECT    (1U << 4)
#define BALL_STATUS_FLAG_APPROACHING     (1U << 5)
#define BALL_STATUS_FLAG_RECOVERY        (1U << 6)
#define BALL_STATUS_FLAG_AT_LIMIT        (1U << 7)
#define BALL_STATUS_FLAG_SEQUENCE_LATE   (1U << 8)

typedef struct {
    uint16_t runId;
    uint32_t mspMs;
    uint8_t state;
    uint8_t motionPhase;
    uint16_t flags;
    int16_t currentSteps;
    int16_t targetSteps;
    int16_t errorQ4;
    int16_t filteredVelocity;
    uint16_t stepHz;
    uint16_t tiltLimit;
    uint8_t recoveryPhase;
    uint8_t armFrames;
    uint16_t targetXQ4;
    int16_t continuousTiltQ8;
    int16_t frictionBoostQ8;
    uint16_t visionAgeMs;
    uint8_t faultReason;
    uint16_t crcErrors;
    uint16_t sequenceDrops;
    uint16_t rxOverflows;
    uint8_t event;
} BallStatusFrame;

void ball_protocol_init(void);
void ball_protocol_uart_isr(void);
void ball_protocol_process_5ms(uint32_t nowMs);
BallVisionSample ball_protocol_get_sample(void);
BallTargetCommand ball_protocol_get_target_command(void);
bool ball_protocol_queue_status(const BallStatusFrame *status);
uint16_t ball_protocol_get_tx_drops(void);

#endif
