# Project pinmux summary

The C07A V1.1 schematic identifies the MCU as **MSPM0G3507SPTR**, so the physical numbers below use the **48-pin LQFP (PT/PTR)** column of the local TI datasheet. This is a project-only summary, not a replacement for TI Table 6-2.

Source abbreviations used in every row:

- **H**: repository-root [`硬件.md`](../../../../硬件.md), which establishes the fixed project purpose.
- **D**: bundled TI [`mspm0g3507.pdf`](official/ti/mspm0g3507.pdf), SLASEX6C Table 6-2, which establishes the package pin and mux capability.
- **C**: bundled [`C07A core-board schematic`](board-docs/c07a/C07A核心板原理图_V1.1（MSPM0G3507）.pdf), which establishes the installed part/package and breakout wiring.
- **S**: bundled [`S28A schematic`](board-docs/s28a/4.C07A适配S28A底板原理图.pdf) and [`resource map`](board-docs/s28a/3.C07A搭配S28A底板资源分配表25.7.29.pdf), which establish bottom-board routing and labels.
- **M**: bundled [eight-channel grayscale module documents](module-docs/grayscale/index.md), which establish the module's digital OUT behavior.
- **U**: the user's explicit correction of the installed hardware assignment when a stale repository row conflicts with the observed board.

`PF` values below are datasheet pin-function selections, not `.syscfg` fields or generated macros.

| Physical pin | Project purpose | Verified peripheral function | Authoritative source |
|---|---|---|---|
| PA25, LQFP-48 pin 45 | Motor A Encoder A | GPIO input; project uses GPIO interrupt | H §2; D Table 6-2; C; S |
| PA26, LQFP-48 pin 46 | Motor A Encoder B | GPIO input; project uses GPIO interrupt | H §2; D Table 6-2; C; S |
| PA13, LQFP-48 pin 28 | Motor A Direction 1 | GPIO output | H §2; D Table 6-2; C; S |
| PA14, LQFP-48 pin 29 | Motor A Direction 2 | GPIO output | H §2; D Table 6-2; C; S |
| PB2, LQFP-48 pin 14 | Motor A PWM | TIMG6_C0, PF7 | H §2; D Table 6-2; C; S |
| PB20, LQFP-48 pin 41 | Motor B Encoder A | GPIO input; project uses GPIO interrupt | H §2; D Table 6-2; C; S |
| PB24, LQFP-48 pin 42 | Motor B Encoder B | GPIO input; project uses GPIO interrupt | H §2; D Table 6-2; C; S |
| PA17, LQFP-48 pin 32 | Motor B Direction 1 | GPIO output | H §2; D Table 6-2; C; S |
| PA16, LQFP-48 pin 31 | Motor B Direction 2 | GPIO output | H §2; D Table 6-2; C; S |
| PB3, LQFP-48 pin 15 | Motor B PWM | TIMG6_C1, PF7 | H §2; D Table 6-2; C; S |
| PA0, LQFP-48 pin 1 | MPU6050 SDA | I2C0_SDA, PF3 | H §3; D Table 6-2; C; S |
| PA1, LQFP-48 pin 2 | MPU6050 SCL | I2C0_SCL, PF3 | H §3; D Table 6-2; C; S |
| PA7, LQFP-48 pin 13 | MPU6050 INT | GPIO input; project uses GPIO interrupt | H §3; D Table 6-2; C; S |
| PB7, LQFP-48 pin 21 | K230 → MSPM0 receive | UART1_RX, PF2 | H §4; D Table 6-2; C; S |
| PB6, LQFP-48 pin 20 | MSPM0 → K230 transmit | UART1_TX, PF2 | H §4; D Table 6-2; C; S |
| PA8, LQFP-48 pin 16 | Grayscale AD1 | GPIO output, channel-address bit | H §6; D Table 6-2; C; S |
| PA22, LQFP-48 pin 40 | Grayscale AD0 | GPIO output, channel-address bit | H §6; D Table 6-2; C; S |
| PA18, LQFP-48 pin 33 | BLS push button | GPIO input; external 47 kOhm pull-down to GND, press connects to 3V3, active high | C; U |
| PB17, LQFP-48 pin 36 | TILT servo PWM | TIMA1_C0; route confirmed by current 48-pin SysConfig and generated header | H §5; D Table 6-2; C; S |
| PB16, LQFP-48 pin 26 | PAN servo PWM | TIMG7_C1; route confirmed by current 48-pin SysConfig and generated header | H §5; D Table 6-2; C; S |
| PA12, LQFP-48 pin 27 | Grayscale AD2 | GPIO output, channel-address bit | H §6; D Table 6-2; C; S |
| PA27, LQFP-48 pin 47 | Grayscale OUT | GPIO input. The module OUT is a digital signal with a project-confirmed maximum of 3.3 V; do not configure PA27 as ADC for this module. | H §6; D Table 6-2; C; S; M |
| PA28, LQFP-48 pin 3 | Existing OLED signal | GPIO reserved for existing OLED; exact signal role comes from the existing driver/project | H §7; D Table 6-2; C; S |
| PA31, LQFP-48 pin 5 | Existing OLED signal | GPIO reserved for existing OLED; exact signal role comes from the existing driver/project | H §7; D Table 6-2; C; S |
| PB14, LQFP-48 pin 24 | Existing OLED signal | GPIO reserved for existing OLED; exact signal role comes from the existing driver/project | H §7; D Table 6-2; C; S |
| PB15, LQFP-48 pin 25 | Existing OLED signal | GPIO reserved for existing OLED; exact signal role comes from the existing driver/project | H §7; D Table 6-2; C; S |
| PB8, LQFP-48 pin 22 | Reserved/broken-out IO, not the BLS button used by this project | GPIO-capable pin; BLS is fixed to PA18 | D Table 6-2; C; U |
| PB9, LQFP-48 pin 23 | On-board status LED | GPIO output | H §8; D Table 6-2; C; S |
| PA24, LQFP-48 pin 44 | Reserved ultrasonic interface | GPIO reserved; do not allocate to radar or another permanent module | H §9; D Table 6-2; C; S |
| PA9, LQFP-48 pin 17 | Active-high buzzer control | GPIO output; drive high to sound and low to silence | H §8; D Table 6-2; C; S; U |

## Boundaries

- Obtain actual SysConfig instance names, fields, macros, IRQ names, handler aliases, and init functions from the active `.syscfg` and generated `ti_msp_dl_config.h`, never from this table.
- Treat S28A labels such as `TIMG0`, `USART1`, or `ADC0` as board-document labels. Use TI Table 6-2 and current SysConfig metadata for exact TI peripheral naming.
- Recheck the datasheet and Device View if the active `.syscfg` does not select MSPM0G3507 in a 48-pin LQFP package; do not change package metadata to force a solution.
