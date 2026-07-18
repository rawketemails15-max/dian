# Project architecture and firmware conventions

These are project software and behavior conventions moved out of repository-root `硬件.md`. They do not override the physical wiring or electrical facts in that file.

## System roles

- Run the MSPM0G3507 as the real-time chassis controller.
- Run the K230 as the vision coprocessor and advanced HMI.
- Keep the final K230 application capable of offline operation.
- Keep PID parameters owned and persisted by the MSPM0G3507.
- Use an image coordinate origin at the upper-left `(0, 0)`, with X increasing rightward and Y increasing downward.

## K230 UART behavior

- Use bidirectional UART at 115200 baud, 8 data bits, no parity, and 1 stop bit.
- Target a 30 Hz K230 vision-result transmission rate.
- Do not require the K230 and MSPM0 UART peripheral numbers to match; follow each device's actual pinmux and generated configuration.
- Keep K230 GPIO11/GPIO12 assigned to the project's intended UART2 TX/RX route. The bundled `UART.py` uses UART1 on GPIO3/GPIO4 and controls LED52; use it only as API-pattern evidence for its unknown firmware version.
- Verify exact UART2/FPIOA names against the installed CanMV firmware before writing code. Do not infer them by changing `UART1` to `UART2` in the example.
- Separate UART receive/transmit buffering from the camera loop. Define framing, length/checking, timeouts, recovery, and command/result ownership before treating the suggested command names below as a wire protocol.

## Control conventions

- Interpret `speed > 0` as vehicle forward and `speed < 0` as vehicle reverse.
- Use the eight-channel grayscale sensor for ordinary black-line tracking.
- Drive PA22/PA8/PA12 as the AD0/AD1/AD2 channel address and read PA27 as a digital GPIO input. Do not configure PA27 as an ADC input for this module.
- Follow the CH1~CH8 truth table in `硬件.md`; AD0 is the least-significant address bit and AD2 is the most-significant bit.
- Keep the address-to-read settling delay configurable. Bundled examples disagree between 50 us, 100 us, and 1 ms, so establish the final value on the installed hardware instead of treating one example as authoritative.
- Calibrate whether black line and white background read as `0` or `1` for the installed illumination variant and track surface. Do not infer final polarity from a sample `ACTIVE_VALUE` setting.

## Interrupt and bring-up conventions

- Let MPU6050 Data Ready trigger PA7.
- Keep the PA7 ISR short: set a readiness flag such as `imu_data_ready = true`, then perform I2C reads in the main loop or control task.
- Do not perform blocking I2C transactions inside the GPIO ISR.
- Keep both installed gimbal servos within their documented 50~330 Hz and 500~2500 us capability. Preserve an existing valid project frequency; if none exists, make the frequency an explicit configuration decision rather than inferring the active timer setup from a datasheet.
- Begin both axes at the 1500 us midpoint and expand the commanded range gradually while checking mechanical interference. Both manufacturer documents say that increasing pulse width commands counterclockwise shaft rotation, but verify each installed linkage's resulting PAN/TILT direction independently. Read [the gimbal servo reference](module-docs/gimbal-servos.md) for model and connector differences.

## Local display and controls

- Use the OLED for chassis-only debugging when K230 is absent and for encoder, speed, PID, grayscale, IMU, and fault information.
- Suggested PA18 BLS behavior: short press starts/pauses; long press requests emergency stop or mode switching. Treat it as active high because the C07A schematic shows a 47 kOhm pull-down to GND and a press connection to 3V3.
- Suggested PB9 behavior: slow blink means healthy, fast blink means fault, and steady on means running.

## K230 LCD/HMI

Use the K230 3.5-inch 800×480 MIPI LCD for camera preview, vision results, virtual buttons, MSPM0 status, online PID tuning, UART diagnostics, and offline operation. Bundled examples commonly select `Display.ST7701`, but verify that token and panel support against the installed CanMV firmware before reuse.

Keep camera/display/media setup and teardown version-matched. Prefer examples that stop the sensor and deinitialize Display and MediaManager in `finally`; repair examples such as `line_patrol.py` that omit complete cleanup before using them as an application base.

Treat the bundled line-tracking LAB threshold and ROIs as starting values only. Calibrate them for the installed camera, lens, exposure, illumination, mounting height, and competition surface. The reference line-tracking example does not implement the MSPM0 UART result protocol.

Suggested application protocol commands:

```text
GET_STATUS
GET_PID
SET_PID
SAVE_PID
LOAD_DEFAULT
SET_VISION_MODE
VISION_RESULT
HEARTBEAT
```

Treat these command names as a suggested application protocol, not as generated firmware identifiers or a completed wire-format specification.
