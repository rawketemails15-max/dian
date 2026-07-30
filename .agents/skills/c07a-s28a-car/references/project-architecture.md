# Project architecture and firmware conventions

These are project software and behavior conventions moved out of repository-root `硬件.md`. They do not override the physical wiring or electrical facts in that file.

## System roles

- Run the MSPM0G3507 as the real-time chassis and ball-rod actuator controller.
- Run the K230 as the camera-based ball-position sensor, line-vision coprocessor, and advanced HMI.
- For the H problem, derive ball position from the camera; do not reintroduce an ultrasonic ranging path.
- Keep the final K230 application capable of offline operation.
- Keep chassis and ball-rod control parameters owned and persisted by the MSPM0G3507.
- Use an image coordinate origin at the upper-left `(0, 0)`, with X increasing rightward and Y increasing downward.

## K230 UART behavior

- Use bidirectional UART at 115200 baud, 8 data bits, no parity, and 1 stop bit.
- Target a 30 Hz K230 vision-result transmission rate.
- Do not require the K230 and MSPM0 UART peripheral numbers to match; follow each device's actual pinmux and generated configuration.
- Keep K230 40Pin pin 8 TX1(IO3)/UART1_TXD and pin 10 RX1(IO4)/UART1_RXD assigned to the project UART route. Cross TX/RX to MSPM0 PB7/PB6 respectively and keep all grounds common. The former GPIO11/GPIO12 UART2 route is superseded.
- A known-tested CanMV runtime confirmed `FPIOA.UART1_TXD`, `FPIOA.UART1_RXD`, and `UART.UART1` for a prior integrated build. Re-run runtime introspection on the active device and after any firmware change; the bundled `UART.py` alone is not version proof, and its LED52 behavior is unrelated.
- Separate UART receive/transmit buffering from the camera loop. Define framing, length/checking, timeouts, recovery, and command/result ownership before treating the suggested command names below as a wire protocol.

## Control conventions

- Interpret `speed > 0` as vehicle forward and `speed < 0` as vehicle reverse.
- Use the eight-channel grayscale sensor for ordinary black-line tracking.
- Drive PA22/PA8/PA12 as the AD0/AD1/AD2 channel address and read PA27 as a digital GPIO input. Do not configure PA27 as an ADC input for this module.
- Follow the CH1~CH8 truth table in `硬件.md`; AD0 is the least-significant address bit and AD2 is the most-significant bit.
- Keep the address-to-read settling delay configurable. Bundled examples disagree between 50 us, 100 us, and 1 ms, so establish the final value on the installed hardware.
- Calibrate whether black line and white background read as `0` or `1` for the installed illumination variant and track surface.

## Interrupt and ball-rod bring-up conventions

- Let MPU6050 Data Ready trigger PA7.
- Keep the PA7 ISR short: set a readiness flag such as `imu_data_ready = true`, then perform I2C reads in the main loop or control task.
- Do not perform blocking I2C transactions inside the GPIO ISR.
- Treat PB16/TIMG7_C1 as D36A `STEP1`, PB17 as `DIR1`, and PA24 as `EN1`/ATD5984 `SLEEP#`. Do not restore the former servo PWM assignments.
- Initialize PA24 low so the driver sleeps, keep STEP stopped at a known level, set DIR, then drive PA24 high and wait at least 1 ms before enabling STEP pulses.
- Keep every STEP high and low interval at least 1 us. Hold DIR stable for at least 200 ns before and after each STEP rising edge; for simple and safe direction reversal, stop STEP before changing DIR.
- Generate STEP with a hardware timer where practical. Add an acceleration/deceleration ramp and software travel limits before closed-loop operation; an abrupt high pulse rate can cause loss of synchronism.
- Begin with DIP 1/2/3 all OFF (16 microsteps) and DIP 4/5/6 all ON (about 0.55 A). Do not raise the current setting until the installed motor's rated phase current is known.
- The motor has no documented encoder or limit switch in the supplied assembly. Mechanical center, safe travel, direction polarity, steps per output angle, backlash, and homing behavior remain calibration items.
- Disabling EN removes holding torque. Define a deliberate fault/stop policy that considers beam motion and ball escape rather than automatically sleeping the driver during ordinary balance control.

Recommended startup order:

1. Run the generated system initialization.
2. Hold D36A EN low, STEP inactive, and establish a safe DIR level.
3. Initialize protocol, estimator, control state, and software travel limits.
4. Start the control timebase and enable its IRQ.
5. Drain the UART RX FIFO, clear peripheral and NVIC pending state, then enable the UART IRQ.
6. When motion is authorized, raise EN, wait at least 1 ms, and start the STEP timer at the ramp's initial frequency.

A SysConfig-generated timer, successful build, or toggling GPIO is configuration evidence, not proof of usable torque or safe motion. Check STEP/DIR/EN waveforms, D36A VIN under load, the 5 V fan rail, driver temperature, mechanical travel, torque, and missed steps on the installed assembly. Use [the D36A and ball-rod index](module-docs/d36a-ballrod/index.md) for the electrical and mechanical source facts.

For the historical K230 IO4 receive configuration, RX-only diagnostic, and bidirectional oscilloscope decision tree, read [K230 UART bring-up](k230-uart-debugging.md).

## Local display and controls

- Use the OLED for chassis-only debugging when K230 is absent and for encoder, speed, PID, grayscale, IMU, ball-rod, and fault information.
- Suggested PA18 BLS behavior: short press starts/pauses; long press requests emergency stop or mode switching. Treat it as active high because the C07A schematic shows a 47 kOhm pull-down to GND and a press connection to 3V3.
- Suggested PB9 behavior: slow blink means healthy, fast blink means fault, and steady on means running.

## K230 LCD/HMI

Use the K230 3.5-inch 800×480 MIPI LCD for camera preview, ball position, line-vision results, virtual buttons, MSPM0 status, online parameter tuning, UART diagnostics, and offline operation. Bundled examples commonly select `Display.ST7701`, but verify that token and panel support against the installed CanMV firmware before reuse.

Keep camera/display/media setup and teardown version-matched. Prefer examples that stop the sensor and deinitialize Display and MediaManager in `finally`; repair examples such as `line_patrol.py` that omit complete cleanup before using them as an application base.

Treat bundled vision thresholds and ROIs as starting values only. Calibrate them for the installed camera, lens, exposure, illumination, mounting height, ball appearance, beam geometry, and competition surface. The bundled examples do not implement the MSPM0 UART result protocol.

Suggested application protocol commands:

```text
GET_STATUS
GET_CONTROL_PARAMS
SET_CONTROL_PARAMS
SAVE_CONTROL_PARAMS
LOAD_DEFAULT
SET_VISION_MODE
VISION_RESULT
HEARTBEAT
```

Treat these command names as a suggested application protocol, not as generated firmware identifiers or a completed wire-format specification.
