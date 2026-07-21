# K230 UART and gimbal PWM bring-up playbook

Use this playbook when the K230 vision overlay works but the gimbal does not move, MSPM0 status does not return to the K230, PB16/PB17 have no PWM, the servos do not recenter at boot, or behavior changes after merging the stand-alone servo test with UART/vision code.

Repository-root `硬件.md` remains authoritative for wiring and power. Locate it through the workflow in `SKILL.md`; do not assume a fixed relative depth. Re-read the active `.syscfg`, generated `ti_msp_dl_config.h/.c`, SDK headers, and deployed sources before reusing any historical instance names below.

## Contents

- [Previously verified real-hardware baseline](#previously-verified-real-hardware-baseline)
- [Known-good MSPM0 startup invariant](#known-good-mspm0-startup-invariant)
- [No-recenter or constant-low PWM diagnosis](#diagnose-no-recenter-or-pb16pb17-constant-low)
- [One-way UART diagnosis](#diagnose-one-way-k230mspm0-uart)
- [Known-tested protocol and control invariants](#known-tested-protocol-and-control-invariants)
- [Vision and LCD lessons](#vision-and-lcd-lessons-from-the-integrated-tracker)
- [Validation boundary](#validation-boundary)

## Previously verified real-hardware baseline

The following baseline was observed on a prior integrated build; the currently checked-in active project does not contain its complete PWM/UART configuration or generated macros.

- K230: Hiwonder board running `CanMV v1.4-19-ga7de1c8` based on MicroPython `e00a144`, build date 2025-11-06, board string `k230_canmv_hiwonder`.
- Camera/display: GC2093 CSI2 camera and 800x480 ST7701 LCD ran together with the black-object tracker at roughly 42 FPS in the observed scene.
- K230 UART route: 40Pin physical pin 8 TX1(IO3)/UART1_TXD -> MSPM0 PB7/UART1_RX; MSPM0 PB6/UART1_TX -> 40Pin physical pin 10 RX1(IO4)/UART1_RXD; common GND.
- Do not substitute the separate debug-header TX3/RX3, the former GPIO11/GPIO12 UART2 route, or an example's unrelated GPIO/LED pins.
- PAN: PB16/TIMG7_C1, LD-3015MG. TILT: PB17/TIMA1_C0, LDX-227.
- That integrated firmware was observed to recenter both servos at boot, track a black target in both axes, and return valid MSPM0 status to the K230 LCD.

These observations prove the integrated hardware path only for that exact historical build and wiring. They do not prove the current repository project, a future firmware image, connector, supply, or mechanical installation.

## Known-good MSPM0 startup invariant

K230 absence must never suppress PWM. The historical integrated build used this order; port the sequence only after obtaining the active instances and APIs from regenerated output and the installed SDK:

1. Run `SYSCFG_DL_init()`.
2. Initialize the protocol parser and gimbal state.
3. Load both safe 1500 us midpoint compare values.
4. Explicitly start both PWM counters with the active generated instances and SDK-confirmed APIs. The historical build used `DL_TimerG_startCounter(PWM_PAN_INST)` and `DL_TimerA_startCounter(PWM_TILT_INST)`; these tokens are not present in the currently checked-in active generated header and must not be copied blindly.
5. Clear, enable, and start the 1 ms control timebase.
6. Drain the UART RX FIFO, clear the UART RX peripheral status, and clear its pending NVIC state.
7. Enable the UART IRQ last.

Do not treat a SysConfig `timerStartTimer = true` field, generated initialization code, a successful build, or a valid `.hex` as proof that a physical PWM waveform exists. The explicit start calls were intentionally retained even with automatic start configured; the historical hardware succeeded with this idempotent pattern.

For disconnected K230 operation, keep PB7/UART1_RX pulled to the UART idle-high state. Give the 1 ms control tick higher interrupt priority than UART RX; the historical generated configuration used control priority 1 and UART priority 2. Reconfirm both priorities in the active generated files. This arrangement prevents floating-RX traffic from starving later initialization or control work.

## Diagnose “no recenter” or PB16/PB17 constant low

Do not adjust PID gains or PAN/TILT signs when there is no PWM carrier. Those settings act only after timer output exists.

1. Disconnect servo signal leads and K230, then scope PB16 and PB17 relative to the common ground.
2. Expect continuous 50 Hz output: 20 ms period and 1500 us high pulse at boot.
3. Confirm that the flashed image is the intended active project's image, not merely a same-named stale `total.hex`.
4. Read the active generated header. The verified historical build routed PB16=`TIMG7_C1` through `PWM_PAN_INST` and PB17=`TIMA1_C0` through `PWM_TILT_INST`; never recreate those tokens from memory after regeneration.
5. Confirm a 1 MHz PWM timer clock, 20000-count period, and initial compare 1500 in the active generated output.
6. Confirm that midpoint compare writes and both explicit counter-start calls occur before the UART IRQ is enabled. Inspect final disassembly if the source/build provenance is uncertain.
7. Verify PB7 is idle high with the K230 disconnected. A floating or noisy RX line is a startup/IRQ fault candidate, not evidence of bad servo direction.
8. Scope PB6 for the 10 Hz status burst. It is an independent liveness marker for the main loop and UART TX path.

If the vendor stand-alone servo example works but the integrated build has no PWM, regard the hardware as provisionally capable and compare, in order: the actual flashed artifact, active `.syscfg`, generated instances/pin mux, counter start calls, and startup/IRQ order. Do not copy the example's entire `.syscfg` or generated files.

If only one servo responds, distinguish moving only the PWM signal from moving the whole three-wire connector. Check direct-pin contact, connector pin order, signal continuity, common GND at each servo, and rail sag before changing timer configuration. This project encountered poor Dupont contact, and direct connection to PB16/PB17 reproduced the vendor example.

The 1500 us electronic midpoint does not guarantee that an assembled horn looks geometrically horizontal or vertical. Set the electronic midpoint first, then install/trim the horn and linkage without exceeding mechanical limits.

## Diagnose one-way K230/MSPM0 UART

Treat the two directions independently:

- If black-target motion changes the gimbal, K230 IO3 -> MSPM0 PB7 reception, the vision frame parser, and the control path are already proven for that run.
- If the LCD still says `MSP RX:0`, no byte has reached CanMV's UART read path. CRC and status-payload parsing have not yet participated.
- `MSP BAD RX:n OK:0 CRC:n FMT:n` means raw bytes arrived but no valid status frame was accepted; then inspect baud, framing, byte stream, length, CRC, and resynchronization.
- `MSP ACK:... PAN:... TILT:...` proves a valid status frame and exposes the commanded pulse widths.

### Scope the return path before editing protocol code

1. Leave only MSPM0 PB6 -> K230 IO4 and common GND if necessary. The historical MSPM0 build sent status every 100 ms even without vision input; confirm the active build retains that behavior.
2. Scope PB6: expect idle near 3.3 V and a burst about every 100 ms when that status service is active.
3. Scope the K230's actual 40Pin physical pin 10/IO4 pad, not only the MSPM0 end of the jumper.
4. PB6 activity with no IO4 activity is wiring, pin-position, or connector continuity. Do not change CRC or PID.
5. PB6 and IO4 activity with `MSP RX:0` points to K230 pin mux/electrical configuration or CanMV UART reception.
6. No PB6 activity points back to the active MSPM0 image, application liveness, UART TX pin mux, or TX service path.

With no received vision frame and both axes enabled at midpoint, the historical protocol produced this representative 17-byte status frame:

```text
A5 5A 01 81 0A 00 00 0E 00 DC 05 DC 05 00 00 56 D8
```

Decode it only as a representative known-tested sample: header `A5 5A`, version 1, type `0x81`, payload length 10, ACK sequence 0, flags `0x0E`, PAN/TILT 1500 us, CRC-error count 0, and little-endian CRC16 bytes `56 D8`.

### Preserve and revalidate the K230 RX configuration that passed on hardware

An earlier program selected `UART1_RXD` with only `ie=1` and used a zero read timeout. Tracking worked in the transmit direction, but the K230 raw RX count remained zero. The tested historical hardware began receiving valid MSPM0 status after this combined change:

```python
fpioa.set_function(
    3, FPIOA.UART1_TXD,
    ie=0, oe=1, pu=0, pd=0)
fpioa.set_function(
    4, FPIOA.UART1_RXD,
    ie=1, oe=0, pu=1, pd=0, st=1)

uart = UART(
    UART.UART1,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
    timeout=2)

fpioa.help(4)
```

The 2 ms timeout is long enough to cover a 17-byte 8N1 frame at 115200 baud, about 1.48 ms, while remaining short relative to the camera loop. Explicitly setting IO4 input enable, output disable, pull-up, pull-down disable, and Schmitt input avoids retaining unsuitable electrical attributes from an earlier/default pin function. Print `fpioa.help(4)` at startup and confirm the actual mapping/configuration rather than assuming the call succeeded.

The combined configuration above is real-hardware evidence from the historical build. The individual change responsible was not isolated, so do not claim that `timeout=2`, the pull-up, or Schmitt input alone was the root cause. Revalidate all tokens and electrical attributes on the active CanMV firmware.

If normal camera mode still reports `MSP RX:0`, use the tracker file's RX-only diagnostic mode. It must skip camera/LCD initialization, map only IO4 as UART1 RX, call `uart.read()`, print raw byte blocks, and count raw bytes, valid frames, CRC errors, and format errors. If IO4 has a valid 3.3 V UART waveform and RX-only mode still receives nothing while `fpioa.help(4)` is correct, use a controlled UART1 TX-to-RX loopback to separate board/firmware reception from the MSPM0 link before changing the wire protocol.

## Known-tested protocol and control invariants

- UART: 115200, 8N1. Historical K230 vision publish interval 34 ms, approximately 29.4 Hz and no more than 30 Hz. Historical MSPM0 status interval 100 ms.
- Frame: `A5 5A`, version, type, payload length, payload, CRC16-CCITT-FALSE over version through payload; little-endian multibyte values.
- Vision type `0x01`: 12-byte payload with sequence, flags, reserved, X, Y, width, and height. Invalid/LOST frames zero the geometry.
- Status type `0x81`: 10-byte payload with ACK sequence, status flags, reserved, PAN us, TILT us, and CRC-error count.
- Parser must handle fragments, concatenated frames, invalid length/version, bad CRC, interbyte timeout, and resynchronization.
- Coordinates use top-left origin. `error_x = target_x - 400`; `error_y = target_y - 240`.
- Historical calibrated signs were PAN `-1` and TILT `+1`. Keep signs configurable for a changed linkage.
- Historical software travel was 600-2400 us, midpoint 1500 us, deadzone 20 px, maximum update 5 us, initial `Kp=0.05 us/px`, `Ki=0`, `Kd=0`. Manufacturer signal capability remains 500-2500 us; mechanical limits may require a narrower range.
- LOST, invalid target, or UART timeout must hold the current pulse, reset PID history/integral, keep PWM running, and never auto-search.
- PA18 is the active-high BLS button; PB8 is not the project button. Keep the narrow 1450-1550 us button test mode available but disabled during closed-loop tracking.
- K230 performs vision and coordinate transmission; MSPM0 owns both PID states and PWM. Do not add chassis motion to the gimbal tracker.

## Vision and LCD lessons from the integrated tracker

- Prefer classical LAB/grayscale blob detection for a black object on a light background before adding a neural model.
- Centralize threshold, ROI, area, aspect ratio, edge margin, fill ratio, smoothing, and lost-history parameters. Historical starting points included fill-ratio preferred/fallback thresholds 0.70/0.45 and coordinate smoothing alpha 0.35; recalibrate them for lighting and mounting.
- With history, choose the nearest reasonable candidate; without history, choose the largest. Reject edge interference, noise, implausible area/aspect ratio, and insufficient fill.
- Hide the old rectangle immediately on a LOST frame, but retain association history only for the configured short loss window so reacquisition remains stable.
- Display raw UART/valid/CRC/format counters in addition to TRACK/LOST, target geometry, error, fill, and FPS. This prevents a valid outbound control path from being mistaken for a valid return path.
- Keep firmware-matched cleanup in `finally`: stop the sensor, deinitialize UART and Display, enter the supported exitpoint state, then deinitialize MediaManager in the proven order.

## Validation boundary

Report each layer separately: static source inspection, SysConfig generation, compile/link, flashed artifact identity, scope waveform at the source pin, waveform at the receiving pad, raw UART bytes, valid protocol frame, actuator response, and closed-loop behavior. Never turn “compiled,” “PB6 toggles,” or “one direction tracks” into “bidirectional UART and gimbal are fully verified.”
