# S28A board map for this project

Read this together with repository-root [`硬件.md`](../../../../硬件.md). `硬件.md` wins for the intended project assignment; the S28A documents establish physical bottom-board routing and shared-resource warnings.

Bundled sources: [`S28A schematic`](board-docs/s28a/4.C07A适配S28A底板原理图.pdf), [`resource map`](board-docs/s28a/3.C07A搭配S28A底板资源分配表25.7.29.pdf), and the complete [S28A document index](board-docs/index.md).

## Fixed project routing

| Function | MSPM0 pins | S28A/C07A routing fact | Project constraint |
|---|---|---|---|
| Motor A encoder/direction/PWM | PA25, PA26; PA13, PA14; PB2 | Routed through the Motor A encoder/output and TB6612 interfaces | Keep PB2 as TIMG6_C0; do not swap motor A/B pins to fix code |
| Motor B encoder/direction/PWM | PB20, PB24; PA17, PA16; PB3 | Routed through the Motor B encoder/output and TB6612 interfaces | Keep PB3 as TIMG6_C1; motor side/polarity remain calibration facts |
| MPU6050 | PA0 SDA, PA1 SCL, PA7 INT | Routed to the S28A MPU6050 module header | Use I2C0 pin capability from TI docs; confirm pull-ups from actual hardware/project before diagnosing I2C |
| K230 UART | PB6 TX, PB7 RX, common GND | Reuses the S28A Bluetooth UART signal group; original Bluetooth is not used | MSPM0 PB6 → K230 GPIO12/UART2_RXD; K230 GPIO11/UART2_TXD → PB7; UART numbers need not match; do not copy K230 example UART1/GPIO3/GPIO4 or 40Pin TX1/RX1 |
| PAN servo | PB16 / TIMG7_C1 | User-confirmed direct-pin wiring reproduces the example behavior | Installed servo is LD-3015MG; use independent rated servo power and common ground for the formal design |
| TILT servo | PB17 / TIMA1_C0 | User-confirmed direct-pin wiring reproduces the example behavior | Installed servo is LDX-227; use independent rated servo power and common ground for the formal design |
| Eight-channel grayscale address/output | PA22 AD0, PA8 AD1, PA12 AD2, PA27 OUT | AD0/AD1 were moved to release PB17/PB16 for the direct servo connectors | Supply the module from 5 V; OUT is a digital signal with a maximum of 3.3 V; configure PA27 as GPIO input, not ADC |
| OLED | PA28, PA31, PB14, PB15 | C07A/S28A reserve the four GPIOs for the existing OLED | Preserve the existing driver and discover exact signal roles from the actual project |
| BLS button / LED | PA18 / PB9 | The C07A schematic shows BLS on PA18 with an external 47 kOhm pull-down; PB9 is the status LED | Treat BLS as active high; PB8 is not this project's BLS input |
| Buzzer | PA9 | User-confirmed installed buzzer control | GPIO output, active high; initialize low so the buzzer stays silent at startup |
| Ultrasonic reservation | PA24 | Remaining project reservation after PA9 was confirmed as the buzzer control | Do not allocate PA24 to another permanent module |

## Power routing facts

- The repository records a 7.4 V battery and P03B nominal 5 V/3 A logic rail.
- The installed LD-3015MG PAN and LDX-227 TILT servos require a separate **6~8.4 V Servo Power Rail**. Their two theoretical stall currents total about **4.8~6 A**.
- The user has observed recenter and button movement at 5 V. Treat it as bench evidence only; do not use P03B 5 V/3 A as the manufacturer-rated formal two-servo supply because the voltage is outside the specified range and theoretical-stall current margin is insufficient.
- Connect all control-system grounds together.
- Do not select direct battery feed or an independent BEC until the user confirms the final power design.

## Shared-resource conflicts inherited from S28A

- PB6/PB7 are the original Bluetooth UART group; this project assigns them to K230, so do not enable the original Bluetooth module.
- PA24 is shared by S28A radar/ultrasonic concepts; this project keeps PA24 reserved, so do not enable the radar assignment. PA9 is fixed to the active-high buzzer.
- The S28A resource map warns that ultrasonic and original CCD line-following share resources, and that the handle and original line-following interface share resources. This project does not use the original CCD module or handle.
- PB16/PB17 are fixed to the PAN/TILT servos, PA9 is fixed to the buzzer, and PA22/PA8/PA12 are grayscale address pins. Do not restore the S28A original CCD/handle defaults over these assignments.
- The current eight-channel grayscale topology is project-specific. Do not infer its configuration from the S28A's original `CCD`/`ADC0` label or from the bundled module examples' PA14/PA15/PA16/PA17 and X1/X2/X3/X4 mappings.

## Board labels versus TI names

Board documents can use simplified or legacy labels. Verified examples from the bundled board documents and TI datasheet:

- Board-document legacy labels do not override the current project routes: PB16 is generated as `TIMG7_C1` and PB17 as `TIMA1_C0`.
- PA10/PA11 may be labeled `USART1` by the board material, while TI names their UART route `UART0`.

Use the board documents to answer “where is it wired?” and TI/current-SysConfig sources to answer “what is the exact peripheral/configuration name?”.
