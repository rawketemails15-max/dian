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

void ball_protocol_init(void);
void ball_protocol_uart_isr(void);
void ball_protocol_process_5ms(uint32_t nowMs);
BallVisionSample ball_protocol_get_sample(void);

#endif
