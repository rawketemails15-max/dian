#include "ball_protocol.h"

#include "ti_msp_dl_config.h"

#define BALL_RX_RING_SIZE  (128U)
#define BALL_FRAME_SIZE    (15U)
#define BALL_PAYLOAD_SIZE  (8U)
#define BALL_STATUS_TYPE          (0x83U)
#define BALL_STATUS_PAYLOAD_SIZE  (36U)
#define BALL_STATUS_FRAME_SIZE    (43U)
#define BALL_TX_QUEUE_SIZE        (4U)
#define BALL_TX_PERIODIC_LIMIT    (3U)

typedef struct {
    uint8_t data[BALL_STATUS_FRAME_SIZE];
    bool critical;
} BallTxFrame;

static volatile uint8_t gRxRing[BALL_RX_RING_SIZE];
static volatile uint8_t gRxWrite;
static volatile uint8_t gRxRead;
static volatile uint32_t gRxOverflows;

static uint8_t gFrame[BALL_FRAME_SIZE];
static uint8_t gFrameIndex;
static BallVisionSample gSample;
static bool gHaveSequence;
static BallTxFrame gTxQueue[BALL_TX_QUEUE_SIZE];
static volatile uint8_t gTxRead;
static volatile uint8_t gTxWrite;
static volatile uint8_t gTxCount;
static volatile uint8_t gTxByteIndex;
static volatile uint16_t gTxDrops;
static uint16_t gTxSequence;

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

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t) value;
    destination[1] = (uint8_t) (value >> 8U);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t) value;
    destination[1] = (uint8_t) (value >> 8U);
    destination[2] = (uint8_t) (value >> 16U);
    destination[3] = (uint8_t) (value >> 24U);
}

static void service_tx_fifo(void)
{
    while ((gTxCount != 0U) &&
        !DL_UART_Main_isTXFIFOFull(UART_K230_INST)) {
        BallTxFrame *frame = &gTxQueue[gTxRead];

        DL_UART_Main_transmitData(
            UART_K230_INST, frame->data[gTxByteIndex]);
        gTxByteIndex++;
        if (gTxByteIndex >= BALL_STATUS_FRAME_SIZE) {
            gTxByteIndex = 0U;
            gTxRead =
                (uint8_t) ((gTxRead + 1U) % BALL_TX_QUEUE_SIZE);
            gTxCount--;
        }
    }

    if (gTxCount == 0U) {
        DL_UART_Main_disableInterrupt(
            UART_K230_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

static void copy_tx_frame(
    uint8_t queueIndex, const uint8_t *frame, bool critical)
{
    for (uint8_t i = 0U; i < BALL_STATUS_FRAME_SIZE; i++) {
        gTxQueue[queueIndex].data[i] = frame[i];
    }
    gTxQueue[queueIndex].critical = critical;
}

static bool replace_tail_periodic(const uint8_t *frame)
{
    uint8_t index;

    if (gTxCount == 0U) {
        return false;
    }
    index = (uint8_t) ((gTxWrite + BALL_TX_QUEUE_SIZE - 1U) %
        BALL_TX_QUEUE_SIZE);
    if (gTxQueue[index].critical ||
        ((index == gTxRead) && (gTxByteIndex != 0U))) {
        return false;
    }
    copy_tx_frame(index, frame, false);
    return true;
}

static bool remove_newest_queued_periodic(void)
{
    uint8_t index = gTxWrite;

    for (uint8_t checked = 0U; checked < gTxCount; checked++) {
        uint8_t next;

        index = (uint8_t)
            ((index + BALL_TX_QUEUE_SIZE - 1U) %
                BALL_TX_QUEUE_SIZE);
        if (gTxQueue[index].critical ||
            ((index == gTxRead) && (gTxByteIndex != 0U))) {
            continue;
        }

        /*
         * Remove the periodic frame and compact every newer queued frame
         * toward the read side.  This preserves FIFO order for all critical
         * events; the new critical frame is appended at the tail below.
         */
        next = (uint8_t) ((index + 1U) % BALL_TX_QUEUE_SIZE);
        while (next != gTxWrite) {
            gTxQueue[index] = gTxQueue[next];
            index = next;
            next = (uint8_t) ((next + 1U) % BALL_TX_QUEUE_SIZE);
        }
        gTxWrite = (uint8_t)
            ((gTxWrite + BALL_TX_QUEUE_SIZE - 1U) %
                BALL_TX_QUEUE_SIZE);
        gTxCount--;
        return true;
    }
    return false;
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
    gTxRead = 0U;
    gTxWrite = 0U;
    gTxCount = 0U;
    gTxByteIndex = 0U;
    gTxDrops = 0U;
    gTxSequence = 0U;
    DL_UART_Main_disableInterrupt(
        UART_K230_INST, DL_UART_MAIN_INTERRUPT_TX);
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
    service_tx_fifo();
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

bool ball_protocol_queue_status(const BallStatusFrame *status)
{
    uint8_t frame[BALL_STATUS_FRAME_SIZE];
    uint16_t crc;
    uint32_t primask;
    bool critical = status->event != BALL_STATUS_EVENT_NONE;

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = 0x01U;
    frame[3] = BALL_STATUS_TYPE;
    frame[4] = BALL_STATUS_PAYLOAD_SIZE;
    put_u16_le(&frame[5], gTxSequence++);
    put_u16_le(&frame[7], status->runId);
    put_u32_le(&frame[9], status->mspMs);
    frame[13] = status->state;
    frame[14] = status->motionPhase;
    put_u16_le(&frame[15], status->flags);
    put_u16_le(&frame[17], (uint16_t) status->currentSteps);
    put_u16_le(&frame[19], (uint16_t) status->targetSteps);
    put_u16_le(&frame[21], (uint16_t) status->errorQ4);
    put_u16_le(&frame[23], (uint16_t) status->filteredVelocity);
    put_u16_le(&frame[25], status->stepHz);
    put_u16_le(&frame[27], status->tiltLimit);
    frame[29] = status->recoveryPhase;
    frame[30] = status->armFrames;
    put_u16_le(&frame[31], status->crcErrors);
    put_u16_le(&frame[33], status->sequenceDrops);
    put_u16_le(&frame[35], status->rxOverflows);
    put_u16_le(&frame[37], gTxDrops);
    frame[39] = status->event;
    frame[40] = 0U;
    crc = crc16_ccitt_false(&frame[2], 39U);
    put_u16_le(&frame[41], crc);

    primask = __get_PRIMASK();
    __disable_irq();
    /*
     * A periodic update can only replace the newest queued periodic update.
     * It may never overwrite an older slot across a critical event, because
     * that would make transmitted status sequences move backward.
     */
    if (!critical && (gTxCount >= BALL_TX_PERIODIC_LIMIT)) {
        if (replace_tail_periodic(frame)) {
            if (primask == 0U) {
                __enable_irq();
            }
            return true;
        }
        gTxDrops++;
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }
    if (gTxCount >= BALL_TX_QUEUE_SIZE) {
        if (!critical || !remove_newest_queued_periodic()) {
            gTxDrops++;
            if (primask == 0U) {
                __enable_irq();
            }
            return false;
        }
    }
    copy_tx_frame(gTxWrite, frame, critical);
    gTxWrite = (uint8_t) ((gTxWrite + 1U) % BALL_TX_QUEUE_SIZE);
    gTxCount++;
    DL_UART_Main_enableInterrupt(
        UART_K230_INST, DL_UART_MAIN_INTERRUPT_TX);
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

uint16_t ball_protocol_get_tx_drops(void)
{
    uint16_t drops;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    drops = gTxDrops;
    if (primask == 0U) {
        __enable_irq();
    }
    return drops;
}
