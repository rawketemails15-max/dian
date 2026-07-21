#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t encoderCountA;
    int32_t encoderCountB;
    uint32_t speedCountA;
    uint32_t speedCountB;
    uint16_t targetCountA;
    uint16_t targetCountB;
    uint16_t pwmTicksA;
    uint16_t pwmTicksB;
    bool running;
    bool stalled;
} MotorTelemetry;

void motor_control_init(void);
void motor_control_start(uint32_t nowMs);
void motor_control_brake(void);
void motor_control_update_20ms(uint32_t nowMs, uint16_t targetCountA,
    uint16_t targetCountB);

void motor_control_handle_encoder_a_irq(void);
void motor_control_handle_encoder_b_irq(void);

bool motor_control_stalled(void);
MotorTelemetry motor_control_get_telemetry(void);

#endif
