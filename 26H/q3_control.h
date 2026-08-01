#ifndef Q3_CONTROL_H_
#define Q3_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    Q3_IDLE = 0,
    Q3_TRIM,
    Q3_WAKE,
    Q3_LAUNCH,
    Q3_LAUNCH_HOLD,
    Q3_REVERSE,
    Q3_RETURN_HOLD,
    Q3_LEVEL,
    Q3_SETTLE,
    Q3_COMPLETE,
    Q3_TIMEOUT_LEVEL,
    Q3_FAULT
} Q3State;

typedef struct {
    Q3State state;
    /* Physical microstep position relative to MCU power-on. */
    int32_t currentSteps;
    int32_t targetSteps;
    int32_t runZeroSteps;
    uint32_t runElapsedMs;
    uint16_t stepFrequencyHz;
    int8_t trimDirection;
    uint8_t directionLevel;
    bool driverEnabled;
    bool stepRunning;
    bool sequenceTimedOut;
    bool adjustmentUiActive;
} Q3Telemetry;

void Q3_init(uint32_t nowMs);
void Q3_tick_5ms(uint32_t nowMs, bool buttonPressed);
void Q3_step_isr(void);
Q3Telemetry Q3_get_telemetry(void);

#endif
