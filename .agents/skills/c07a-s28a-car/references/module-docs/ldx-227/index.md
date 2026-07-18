# LDX-227 module reference

Read the bundled Hiwonder [`LDX-227数字舵机规格书.pdf`](LDX-227数字舵机规格书.pdf), V1.0, initially released 2023-09-23, for manufacturer module facts. Repository-root [`硬件.md`](../../../../../../硬件.md) remains authoritative for this project's wiring and power decisions.

## Established manufacturer facts

| Category | Fact |
|---|---|
| Type and motion | Metal-gear digital PWM servo; 0~270° |
| Supply | 6~8.4 V |
| Stall torque | 15 kg·cm at 6 V; 17 kg·cm at 7.4 V |
| Speed | 0.16 s/60° at 7.4 V |
| Current | 100 mA no-load; 2.4~3 A stall |
| PWM | 500~2500 us maps to 0~270°; 1500 us neutral; 50~330 Hz |
| Direction | Counterclockwise as pulse width increases from 500 to 2500 us |
| Control dead band | 4 us |
| Temperature | -25~70 ℃ operating; -30~80 ℃ storage |
| Mechanical | 40×20×51.4 mm; 57 g; gear ratio 293; dual bearing |
| Cable and connector | 300±5 mm; PH2.0-3P |
| Other | 3-pole motor; no waterproof rating |

Wire definitions in the document are white for PWM signal, middle black for supply positive, and outer black for supply negative/GND.

## Project boundaries

- Do not reinterpret the product-description phrase `6V 25kg Servo` as a tested torque value. Use the electrical table's 15 kg·cm at 6 V and 17 kg·cm at 7.4 V values.
- Keep the separate 6~8.4 V Servo Power Rail and common GND required by `硬件.md`. Do not use the S28A/P03B 5 V/3 A rail as the formal two-servo supply.
- Do not choose direct battery feed versus a separate BEC without user confirmation.
- Treat 500~2500 us and 50~330 Hz as servo capabilities, not as proof of a current SysConfig timer setup.
- Obtain the current PB17/TIMA1_C0 TILT timer configuration, generated PWM macros, instance names, and DriverLib calls from the active `.syscfg`, generated header, and installed SDK.
