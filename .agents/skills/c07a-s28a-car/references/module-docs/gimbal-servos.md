# Gimbal servo reference

Use this page for the installed two-axis gimbal. The user has explicitly confirmed the installed models and the revised PWM assignment below; repository-root `硬件.md` is the canonical editable wiring source.

## Installed assignment

| Axis | Fixed PWM signal | Installed servo |
|---|---|---|
| PAN / horizontal | PB16 / TIMG7_C1 | Hiwonder LD-3015MG |
| TILT / pitch | PB17 / TIMA1_C0 | Hiwonder LDX-227 |

Read the repository-root Hiwonder [`LD-3015MG datasheet`](../../../../../LD-3015MG数字舵机规格书.pdf), V1.0, initially released 2023-09-23, for PAN-servo manufacturer facts. Read the bundled [LDX-227 reference](ldx-227/index.md) and its manufacturer PDF for TILT-servo facts.

## Common control and power facts

Both manufacturer documents specify:

- 6-8.4 V supply.
- 500-2500 us pulse width mapping to 0-270 degrees.
- 1500 us neutral.
- 50-330 Hz control frequency.
- 4 us dead band.
- Counterclockwise shaft rotation as pulse width increases.
- 100 mA no-load current and 2.4-3 A stall current.
- 15 kg-cm stall torque at 6 V and 17 kg-cm at 7.4 V.

The same initial 50 Hz, 1500 us PWM configuration is therefore valid for both servo electronics. Do not infer identical installed-axis direction, safe mechanical range, PID gains, backlash, or load response from this electrical compatibility; calibrate PAN and TILT independently.

The LD-3015MG electrical table prints `0.16 sec/60 degrees 12V` even though the same document limits operation to 6-8.4 V. Treat this as an internally inconsistent speed entry. Never apply 12 V to this servo based on that row.

## Differences that matter during bring-up

| Item | LD-3015MG (PAN) | LDX-227 (TILT) |
|---|---|---|
| Size | 40 x 20 x 40.5 mm | 40 x 20 x 51.4 mm |
| Weight | 62 g | 57 g |
| Bearing | Single bearing | Dual bearing |
| Connector | Dupont 2.54-3P | PH2.0-3P |
| Wire definition | White signal, red positive, black GND | White signal, middle black positive, outer black GND |

Never assume that the connectors are pin-compatible merely because the PWM ranges match. Verify signal, positive supply, and GND by wire definition and connector position before applying servo power. Keep the independent 6-8.4 V Servo Power Rail and common control-system GND for the formal design.

The user reproduced the vendor example's two-axis recenter/button movement with PB17/PB16 PWM, PA18 BLS, and a 5 V bench supply. Record this as a successful bench observation only. It does not change either manufacturer's 6-8.4 V rating and does not establish stall-current margin or long-term reliability at 5 V.

## Hardware diagnosis boundary

The active generated configuration must establish PB16/TIMG7_C1 for PAN and PB17/TIMA1_C0 for TILT. The current 48-pin SysConfig solver and generated `ti_msp_dl_config.h` have confirmed both routes. If the same servo behaves differently after swapping only PWM signals, first distinguish signal-only swapping from moving the entire three-wire connector. Then check connector continuity, common GND at each servo, rail voltage during startup, and actual PB16/PB17 waveforms before changing direction signs or PID values.
