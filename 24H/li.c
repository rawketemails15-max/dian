/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Reference-only source. Its app_config/motor_control/line_tracker modules
 * are not part of this CCS project; the straight-control pattern is integrated
 * into empty.c. Keep this file out of the firmware translation units.
 */
#if 0

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "buzzer.h"
#include "button_debounce.h"
#include "grayscale.h"
#include "line_tracker.h"
#include "motor_control.h"
#include "ti_msp_dl_config.h"
#include "track_app.h"

static volatile uint32_t gSystemMs;
volatile TrackDiagnostics gTrackDiagnostics;

static TrackState gState;
static TrackFault gFault;
static TrackMission gMission;
static TrackPhase gPhase;
static ButtonDebounce gBlsButton;
static ButtonEvent gLastButtonEvent;
static uint32_t gLastControlMs;
static uint32_t gLastLedToggleMs;
static uint16_t gStraightTargetCount;
static bool gFaultLedOn;
static bool gIgnoreNextRelease;

void SysTick_Handler(void)
{
    ++gSystemMs;
}

void GROUP1_IRQHandler(void)
{
    uint32_t pendingA = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_A_PORT, GPIO_ENCODER_A_MOTOR_A_ENC_A_PIN |
            GPIO_ENCODER_A_MOTOR_A_ENC_B_PIN);
    uint32_t pendingB = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_B_PORT, GPIO_ENCODER_B_MOTOR_B_ENC_A_PIN |
            GPIO_ENCODER_B_MOTOR_B_ENC_B_PIN);

    if (pendingA != 0U) {
        motor_control_handle_encoder_a_irq();
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_A_PORT, pendingA);
    }
    if (pendingB != 0U) {
        motor_control_handle_encoder_b_irq();
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_B_PORT, pendingB);
    }
}

static bool read_bls(void)
{
    bool level = (DL_GPIO_readPins(GPIO_BUTTON_PORT,
        GPIO_BUTTON_BLS_PA18_PIN) != 0U);

    return (APP_BUTTON_ACTIVE_LEVEL != 0U) ? level : !level;
}

static void set_status_led(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_STATUS_PORT, GPIO_STATUS_LED_PB9_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_STATUS_PORT, GPIO_STATUS_LED_PB9_PIN);
    }
}

static bool is_running(void)
{
    return (gState == TRACK_STATE_RUNNING_STRAIGHT) ||
        (gState == TRACK_STATE_RUNNING_TRACKING);
}

static void enter_idle(TrackFault fault)
{
    motor_control_brake();
    line_tracker_stop();
    gStraightTargetCount = 0U;
    gState = TRACK_STATE_IDLE;
    gFault = fault;
    gPhase = (fault == TRACK_FAULT_NONE) ? TRACK_PHASE_IDLE :
        TRACK_PHASE_FAULT;
    if (fault == TRACK_FAULT_NONE) {
        gMission = TRACK_MISSION_NONE;
    }
}

static void enter_encoder_fault(void)
{
    motor_control_brake();
    line_tracker_stop();
    gStraightTargetCount = 0U;
    gState = TRACK_STATE_FAULT_ENCODER;
    gFault = TRACK_FAULT_ENCODER;
    gPhase = TRACK_PHASE_FAULT;
}

static void start_straight(uint32_t nowMs)
{
    line_tracker_stop();
    motor_control_start(nowMs);
    gStraightTargetCount = 0U;
    gLastControlMs = nowMs;
    gState = TRACK_STATE_RUNNING_STRAIGHT;
    gFault = TRACK_FAULT_NONE;
    gPhase = TRACK_PHASE_STRAIGHT_TO_B;
}

static void begin_mission(TrackMission mission, uint32_t nowMs)
{
    motor_control_brake();
    line_tracker_stop();
    gStraightTargetCount = 0U;
    gMission = mission;
    gPhase = TRACK_PHASE_CALIBRATING_WHITE;
    gState = TRACK_STATE_CALIBRATING;
    gFault = TRACK_FAULT_NONE;
    grayscale_start_calibration(nowMs);
}

static void service_calibration(uint32_t nowMs)
{
    if (!grayscale_calibration_done()) {
        return;
    }

    if (grayscale_calibration_failed()) {
        enter_idle(TRACK_FAULT_LINE_CALIBRATION);
    } else {
        start_straight(nowMs);
    }
}

static void signal_point(uint32_t nowMs)
{
    buzzer_pulse(nowMs);
}

static void complete_at_b(uint32_t nowMs)
{
    motor_control_brake();
    line_tracker_stop();
    gStraightTargetCount = 0U;
    gState = TRACK_STATE_STOPPED_B;
    gFault = TRACK_FAULT_NONE;
    gPhase = TRACK_PHASE_COMPLETE_B;
    signal_point(nowMs);
}

static void start_tracking_from_b(uint32_t nowMs)
{
    uint16_t initialTarget = gStraightTargetCount;

    signal_point(nowMs);
    line_tracker_start(nowMs, initialTarget);
    gStraightTargetCount = 0U;
    gLastControlMs = nowMs;
    gState = TRACK_STATE_RUNNING_TRACKING;
    gPhase = TRACK_PHASE_TRACKING_B_TO_C;
}

static void complete_at_c(uint32_t nowMs)
{
    motor_control_brake();
    line_tracker_stop();
    gStraightTargetCount = 0U;
    gState = TRACK_STATE_STOPPED_C;
    gFault = TRACK_FAULT_NONE;
    gPhase = TRACK_PHASE_COMPLETE_C;
    signal_point(nowMs);
}

static void update_straight_target(void)
{
    uint16_t nextTarget = (uint16_t) (gStraightTargetCount +
        APP_TRACK_BASE_RAMP_COUNTS);

    if (nextTarget > APP_TRACK_BASE_TARGET_COUNTS) {
        nextTarget = APP_TRACK_BASE_TARGET_COUNTS;
    }
    gStraightTargetCount = nextTarget;
}

static void service_straight(uint32_t nowMs)
{
    GrayscaleStatus line = grayscale_get_status();

    if (line.lineMask != 0U) {
        if (gMission == TRACK_MISSION_TASK_1) {
            complete_at_b(nowMs);
        } else if (gMission == TRACK_MISSION_TASK_2) {
            start_tracking_from_b(nowMs);
        } else {
            enter_idle(TRACK_FAULT_NONE);
        }
        return;
    }

    if ((uint32_t) (nowMs - gLastControlMs) >= APP_CONTROL_PERIOD_MS) {
        gLastControlMs += APP_CONTROL_PERIOD_MS;
        update_straight_target();
        motor_control_update_20ms(nowMs, gStraightTargetCount,
            gStraightTargetCount);
        if (motor_control_stalled()) {
            enter_encoder_fault();
        }
    }
}

static void service_tracking(uint32_t nowMs)
{
    GrayscaleStatus line = grayscale_get_status();
    LineTrackerStatus tracker;

    if (line.lineMask == 0U) {
        complete_at_c(nowMs);
        return;
    }

    if ((uint32_t) (nowMs - gLastControlMs) >= APP_CONTROL_PERIOD_MS) {
        gLastControlMs += APP_CONTROL_PERIOD_MS;
        line_tracker_update_20ms(nowMs, &line);
        tracker = line_tracker_get_status();
        motor_control_update_20ms(nowMs, tracker.targetCountA,
            tracker.targetCountB);
        if (motor_control_stalled()) {
            enter_encoder_fault();
        }
    }
}

static void handle_button_event(ButtonEvent event, uint32_t nowMs)
{
    if (event == BUTTON_EVENT_NONE) {
        return;
    }
    gLastButtonEvent = event;

    if (event == BUTTON_EVENT_PRESSED) {
        if (is_running() || (gState == TRACK_STATE_CALIBRATING)) {
            enter_idle(TRACK_FAULT_NONE);
            gIgnoreNextRelease = true;
        }
        return;
    }

    if (gIgnoreNextRelease) {
        gIgnoreNextRelease = false;
        return;
    }

    if (event == BUTTON_EVENT_SHORT_RELEASE) {
        begin_mission(TRACK_MISSION_TASK_1, nowMs);
    } else if (event == BUTTON_EVENT_LONG_RELEASE) {
        begin_mission(TRACK_MISSION_TASK_2, nowMs);
    }
}

static void update_status_led(uint32_t nowMs)
{
    if (buzzer_active()) {
        gFaultLedOn = false;
        set_status_led(!is_running());
        return;
    }

    if (is_running()) {
        gFaultLedOn = true;
        set_status_led(true);
        return;
    }

    if (gFault == TRACK_FAULT_NONE) {
        gFaultLedOn = false;
        set_status_led(false);
        return;
    }

    if ((uint32_t) (nowMs - gLastLedToggleMs) >=
        APP_FAULT_LED_TOGGLE_MS) {
        gLastLedToggleMs = nowMs;
        gFaultLedOn = !gFaultLedOn;
        set_status_led(gFaultLedOn);
    }
}

static void update_diagnostics(uint32_t nowMs)
{
    GrayscaleStatus line = grayscale_get_status();
    LineTrackerStatus tracker = line_tracker_get_status();
    MotorTelemetry motor = motor_control_get_telemetry();

    gTrackDiagnostics.systemMs = nowMs;
    gTrackDiagnostics.state = gState;
    gTrackDiagnostics.fault = gFault;
    gTrackDiagnostics.mission = gMission;
    gTrackDiagnostics.phase = gPhase;
    gTrackDiagnostics.lastButtonEvent = (uint8_t) gLastButtonEvent;
    gTrackDiagnostics.buttonPressMs = gBlsButton.lastPressDurationMs;
    gTrackDiagnostics.buzzerActive = buzzer_active();
    gTrackDiagnostics.buzzerPulseCount = buzzer_pulse_count();
    gTrackDiagnostics.lineRawMask = line.rawMask;
    gTrackDiagnostics.lineBaselineMask = line.baselineMask;
    gTrackDiagnostics.lineRawLineMask = line.rawLineMask;
    gTrackDiagnostics.lineMask = line.lineMask;
    gTrackDiagnostics.lineAllBlackScanCount = line.allBlackScanCount;
    gTrackDiagnostics.lineScanSequence = line.scanSequence;
    gTrackDiagnostics.trackerMode = tracker.mode;
    gTrackDiagnostics.linePositionQ8 = tracker.positionQ8;
    gTrackDiagnostics.lineErrorQ8 = tracker.errorQ8;
    gTrackDiagnostics.correctionCount = tracker.correctionCount;
    gTrackDiagnostics.lineLostMs = tracker.lostMs;
    gTrackDiagnostics.encoderCountA = motor.encoderCountA;
    gTrackDiagnostics.encoderCountB = motor.encoderCountB;
    gTrackDiagnostics.speedCountA = motor.speedCountA;
    gTrackDiagnostics.speedCountB = motor.speedCountB;
    gTrackDiagnostics.targetCountA = motor.targetCountA;
    gTrackDiagnostics.targetCountB = motor.targetCountB;
    gTrackDiagnostics.pwmTicksA = motor.pwmTicksA;
    gTrackDiagnostics.pwmTicksB = motor.pwmTicksB;
    gTrackDiagnostics.straightTargetCount = gStraightTargetCount;
}

int main(void)
{
    uint32_t nowMs;
    ButtonEvent buttonEvent;

    SYSCFG_DL_init();
    gSystemMs = 0U;
    gState = TRACK_STATE_IDLE;
    gFault = TRACK_FAULT_NONE;
    gMission = TRACK_MISSION_NONE;
    gPhase = TRACK_PHASE_IDLE;
    gLastButtonEvent = BUTTON_EVENT_NONE;
    gLastControlMs = 0U;
    gLastLedToggleMs = 0U;
    gFaultLedOn = false;
    gStraightTargetCount = 0U;
    gIgnoreNextRelease = false;

    motor_control_init();
    grayscale_init(0U);
    line_tracker_init();
    buzzer_init();
    button_debounce_init(&gBlsButton, read_bls(), 0U);
    set_status_led(false);

    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_A_PORT,
        GPIO_ENCODER_A_MOTOR_A_ENC_A_PIN |
        GPIO_ENCODER_A_MOTOR_A_ENC_B_PIN);
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_B_PORT,
        GPIO_ENCODER_B_MOTOR_B_ENC_A_PIN |
        GPIO_ENCODER_B_MOTOR_B_ENC_B_PIN);
    NVIC_EnableIRQ(GPIO_ENCODER_A_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_B_INT_IRQN);

    while (1) {
        nowMs = gSystemMs;
        grayscale_service(nowMs);
        buttonEvent = button_debounce_update(&gBlsButton, read_bls(),
            nowMs);
        buzzer_service(nowMs);
        handle_button_event(buttonEvent, nowMs);

        if (gState == TRACK_STATE_CALIBRATING) {
            service_calibration(nowMs);
        } else if (gState == TRACK_STATE_RUNNING_STRAIGHT) {
            service_straight(nowMs);
        } else if (gState == TRACK_STATE_RUNNING_TRACKING) {
            service_tracking(nowMs);
        }

        update_status_led(nowMs);
        update_diagnostics(nowMs);
        __WFE();
    }
}

#endif
