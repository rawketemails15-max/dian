#include "ball_protocol.h"

#include "ti_msp_dl_config.h"

#define BALL_RX_RING_SIZE  (128U)
#define BALL_FRAME_SIZE    (15U)
#define BALL_PAYLOAD_SIZE  (8U)

static volatile uint8_t gRxRing[BALL_RX_RING_SIZE];
static volatile uint8_t gRxWrite;
static volatile uint8_t gRxRead;
static volatile uint32_t gRxOverflows;

static uint8_t gFrame[BALL_FRAME_SIZE];
static uint8_t gFrameIndex;
static BallVisionSample gSample;
static bool gHaveSequence;

static uint16_t crc16_ccitt_false(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;

    while (length-- != 0U) {
        crc ^= (uint16_t) *data++ << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t) ((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

static bool ring_pop(uint8_t *value)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (gRxRead == gRxWrite) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }
    *value = gRxRing[gRxRead];
    gRxRead = (uint8_t) ((gRxRead + 1U) & (BALL_RX_RING_SIZE - 1U));
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

static void accept_frame(uint32_t nowMs)
{
    uint16_t receivedCrc =
        (uint16_t) gFrame[13] | ((uint16_t) gFrame[14] << 8U);
    uint16_t calculatedCrc = crc16_ccitt_false(&gFrame[2], 11U);
    uint16_t sequence;

    if (receivedCrc != calculatedCrc) {
        gSample.crcErrors++;
        return;
    }

    sequence = (uint16_t) gFrame[5] | ((uint16_t) gFrame[6] << 8U);
    if (gHaveSequence) {
        uint16_t expected = (uint16_t) (gSample.sequence + 1U);
        if (sequence != expected) {
            gSample.sequenceDrops += (uint16_t) (sequence - expected);
        }
    }
    gHaveSequence = true;

    gSample.received = true;
    gSample.sequence = sequence;
    gSample.flags = gFrame[7];
    gSample.confidence = gFrame[8];
    gSample.xQ4 =
        (uint16_t) gFrame[9] | ((uint16_t) gFrame[10] << 8U);
    gSample.velocityX = (int16_t) ((uint16_t) gFrame[11] |
        ((uint16_t) gFrame[12] << 8U));
    gSample.lastFrameMs = nowMs;
    gSample.valid = (gSample.flags & 0x01U) != 0U;
    if (gSample.valid) {
        gSample.lastValidMs = nowMs;
        if (gSample.validStreak < 255U) {
            gSample.validStreak++;
        }
    } else {
        gSample.validStreak = 0U;
    }
}

static void parse_byte(uint8_t value, uint32_t nowMs)
{
    if (gFrameIndex == 0U) {
        if (value == 0xA5U) {
            gFrame[gFrameIndex++] = value;
        }
        return;
    }
    if (gFrameIndex == 1U) {
        if (value == 0x5AU) {
            gFrame[gFrameIndex++] = value;
        } else {
            gFrameIndex = (value == 0xA5U) ? 1U : 0U;
        }
        return;
    }

    gFrame[gFrameIndex++] = value;
    if (gFrameIndex == 5U) {
        if ((gFrame[2] != 0x01U) || (gFrame[3] != 0x03U) ||
            (gFrame[4] != BALL_PAYLOAD_SIZE)) {
            gFrameIndex = (value == 0xA5U) ? 1U : 0U;
        }
    } else if (gFrameIndex == BALL_FRAME_SIZE) {
        accept_frame(nowMs);
        gFrameIndex = 0U;
    }
}

void ball_protocol_init(void)
{
    gRxWrite = 0U;
    gRxRead = 0U;
    gRxOverflows = 0U;
    gFrameIndex = 0U;
    gHaveSequence = false;
    gSample = (BallVisionSample) {0};
}

void ball_protocol_uart_isr(void)
{
    (void) DL_UART_Main_getPendingInterrupt(UART_K230_INST);
    while (!DL_UART_Main_isRXFIFOEmpty(UART_K230_INST)) {
        uint8_t next =
            (uint8_t) ((gRxWrite + 1U) & (BALL_RX_RING_SIZE - 1U));
        uint8_t value = (uint8_t)
            DL_UART_Main_receiveData(UART_K230_INST);

        if (next == gRxRead) {
            gRxOverflows++;
        } else {
            gRxRing[gRxWrite] = value;
            gRxWrite = next;
        }
    }
}

void ball_protocol_process_5ms(uint32_t nowMs)
{
    uint8_t value;

    while (ring_pop(&value)) {
        parse_byte(value, nowMs);
    }
    gSample.rxOverflows = gRxOverflows;
}

BallVisionSample ball_protocol_get_sample(void)
{
    BallVisionSample sample;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    sample = gSample;
    if (primask == 0U) {
        __enable_irq();
    }
    return sample;
}
