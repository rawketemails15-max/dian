# S28A board map for this project

Read this together with repository-root [`硬件.md`](../../../../硬件.md). `硬件.md` wins for the intended project assignment; the S28A documents establish physical bottom-board routing and shared-resource warnings.

Bundled sources: [`S28A schematic`](board-docs/s28a/4.C07A适配S28A底板原理图.pdf), [`resource map`](board-docs/s28a/3.C07A搭配S28A底板资源分配表25.7.29.pdf), and the complete [S28A document index](board-docs/index.md).

## Fixed project routing

| Function | MSPM0 pins | S28A/C07A routing fact | Project constraint |
|---|---|---|---|
| Motor A encoder/direction/PWM | PA25, PA26; PA13, PA14; PB2 | Routed through the Motor A encoder/output and TB6612 interfaces | Keep PB2 as TIMG6_C0; do not swap motor A/B pins to fix code |
| Motor B encoder/direction/PWM | PB20, PB24; PA17, PA16; PB3 | Routed through the Motor B encoder/output and TB6612 interfaces | Keep PB3 as TIMG6_C1; motor side/polarity remain calibration facts |
| MPU6050 | PA0 SDA, PA1 SCL, PA7 INT | Routed to the S28A MPU6050 module header | Use I2C0 pin capability from TI docs; confirm pull-ups from actual hardware/project before diagnosing I2C |
| K230 UART | PB6 TX, PB7 RX, common GND | Reuses the S28A Bluetooth UART signal group; original Bluetooth is not used | MSPM0 PB6 → K230 40Pin pin 10 RX1(IO4)/UART1_RXD; K230 40Pin pin 8 TX1(IO3)/UART1_TXD → PB7; TX/RX are crossed and all grounds are common |
| D36A ball-rod channel 1 | PB16 STEP1/TIMG7_C1, PB17 DIR1, PA24 EN1/SLEEP#; common GND | Uses direct/broken-out C07A/S28A pins and the D36A P1 control header | Keep PB16 as hardware STEP, PB17 as GPIO direction, and PA24 as GPIO wake/sleep; channel 2 is unused |
| Eight-channel grayscale address/output | PA22 AD0, PA8 AD1, PA12 AD2, PA27 OUT | Project-specific address/output wiring | Supply the module from 5 V; OUT is a digital signal with a maximum of 3.3 V; configure PA27 as GPIO input, not ADC |
| OLED | PA28, PA31, PB14, PB15 | C07A/S28A reserve the four GPIOs for the existing OLED | Preserve the existing driver and discover exact signal roles from the actual project |
| BLS button / LED / buzzer | PA18 / PB9 / PA9 | The C07A schematic shows BLS on PA18 with an external 47 kOhm pull-down; PB9 is the status LED; the user reassigned PA9 as active-high buzzer control | Treat BLS and the buzzer control as active high; PB8 is not this project's BLS input |

## Power routing facts

- The repository records a 7.4 V battery and P03B nominal 5 V/3 A logic rail.
- The user selected a raw 7.4 V battery branch for D36A VIN. Do not feed D36A through the P03B 5 V rail.
- Leave the D36A 5 V output disconnected from the S28A/P03B 5 V rail; never parallel the two regulators.
- The D36A vendor manual uses 12 V. ATD5984 and RT8279 each document a 5.5 V operating lower bound, but chip limits do not prove full D36A or loaded stepper performance at 7.4 V.
- Start with DIP 4/5/6 all ON, approximately 0.55 A, until the installed motor's phase-current rating and thermal behavior are known.
- Connect all control-system grounds together.

## Shared-resource conflicts inherited from S28A

- PB6/PB7 are the original Bluetooth UART group; this project assigns them to K230, so do not enable the original Bluetooth module.
- PB16/PB17/PA24 are fixed to D36A STEP1/DIR1/EN1, PA9 is fixed to the buzzer, and PA22/PA8/PA12 are grayscale address pins. Do not restore any S28A legacy module assignments over these project functions.
- This project does not use the original CCD module or handle.
- The current eight-channel grayscale topology is project-specific. Do not infer its configuration from the S28A's original `CCD`/`ADC0` label or from the bundled module examples' PA14/PA15/PA16/PA17 and X1/X2/X3/X4 mappings.

## Board labels versus TI names

Board documents can use simplified or legacy labels. Verified examples from the bundled board documents and TI datasheet:

- Board-document legacy labels do not override the project hardware assignments. PB16/TIMG7_C1 was confirmed in a previously verified generated configuration, but the active project must regenerate and confirm its route and macro names. PB17 and PA24 are plain GPIO outputs for this project.
- PA10/PA11 may be labeled `USART1` by the board material, while TI names their UART route `UART0`.

Use the board documents to answer “where is it wired?” and TI/current-SysConfig sources to answer “what is the exact peripheral/configuration name?”.
