# D36A and ball-rod mechanism reference

Read this file for D36A electrical behavior, connector definitions, DIP settings, and the supplied 42-stepper linkage. Repository-root [`硬件.md`](../../../../../../硬件.md) remains authoritative for this project's pin assignment, channel selection, and power topology.

## Bundled sources

- [`D36A驱动用户手册_V1.5_2026.6.17.pdf`](D36A驱动用户手册_V1.5_2026.6.17.pdf): manufacturer module overview, P1 signal definitions, DIP truth tables, 12 V example wiring, and example pulse-frequency calculations.
- [`D36A驱动问题排查和检测方法.pdf`](D36A驱动问题排查和检测方法.pdf): manufacturer power, unloaded-motor, signal, and waveform checks.
- [`D36A双路步进电机驱动模块_V1.1_2026.04.20.pdf`](D36A双路步进电机驱动模块_V1.1_2026.04.20.pdf): board schematic and exact P1/CN1/CN2 routing.
- [`ATD5984.pdf`](ATD5984.pdf): motor-driver chip electrical range, logic thresholds, STEP/DIR timing, and SLEEP behavior.
- [`RT8279.pdf`](RT8279.pdf): D36A onboard 5 V buck-converter input range.
- [`电阻调节电流参数表.xlsx`](电阻调节电流参数表.xlsx): board current-setting resistor calculations.
- [`步进电机摆杆安装手册.pdf`](步进电机摆杆安装手册.pdf): supplied 42-stepper bracket, coupler, crank, lifting arm, angle bracket, screws, and assembly order.
- [`步进电机摇臂摆杆_尺寸图.pdf`](步进电机摇臂摆杆_尺寸图.pdf) and [`步进电机摇臂摆杆_三维模型截图.png`](步进电机摇臂摆杆_三维模型截图.png): mechanical envelope and geometry.

The bundle intentionally excludes vendor videos, Windows executables, example projects for unrelated MCUs, and their board-specific pin names.

## D36A connector facts

P1 is a 2x5 control header:

| P1 pin | Signal | P1 pin | Signal |
|---|---|---|---|
| 1 | ADC = VIN/11 | 2 | 5 V output |
| 3 | EN2 | 4 | EN1 |
| 5 | DIR2 | 6 | DIR1 |
| 7 | STEP2 | 8 | STEP1 |
| 9 | GND | 10 | GND |

The board labels its wake/sleep inputs `EN1` and `EN2`, but the schematic routes them through 22 ohm series resistors to each ATD5984 `SLEEP#` pin. Therefore low means sleep and high means awake. Do not apply the ATD5984 chip's separate active-low `ENABLE#` semantics to the D36A header.

Motor 1/CN1 is:

| CN1 pin | Winding terminal |
|---|---|
| 1 | B1- |
| 2 | B1+ |
| 3 | A1- |
| 4 | A1+ |

Prefer the supplied keyed four-wire cable. If reterminating, identify both winding pairs with an ohmmeter instead of copying wire colors from a photograph. A sample cable or another motor's color order is not an installed-motor fact.

## Logic and timing facts

- ATD5984 logic input low is at most 0.8 V and logic input high is at least 2.8 V; 3.3 V MSPM0 GPIO is compatible.
- Each STEP rising edge advances one microstep.
- STEP high and low pulse widths must each be at least 1 us.
- DIR and other control changes require at least 200 ns setup before and 200 ns hold after the STEP rising edge.
- After `SLEEP#` rises, wait at least 1 ms for the charge pump to stabilize before issuing STEP pulses.
- Pulling the D36A EN header low enters sleep and removes holding torque.

Treat these as module capabilities, not proof of a current timer, counter, pin mux, or waveform. Obtain the active timer route and generated names from the project `.syscfg` and regenerated `ti_msp_dl_config.h`.

## DIP settings

The board manual defines `ON = 1` and open/OFF = `0`. DIP 1/2/3 set both channels' microstep mode:

| DIP 1 | DIP 2 | DIP 3 | Microstep |
|---|---|---|---|
| 1 | 1 | 1 | Full step |
| 0 | 1 | 1 | 1/2 |
| 0 | 1 | 0 | 1/4 |
| 1 | 0 | 0 | 1/8 |
| 0 | 0 | 0 | 1/16 |
| 1 | 0 | 1 | 1/32 |

DIP 4/5/6 set both channels' current. The supplied workbook orders its switch columns as 6/5/4; the values below are normalized to the physical 4/5/6 labels:

| DIP 4 | DIP 5 | DIP 6 | Approximate current |
|---|---|---|---|
| 0 | 0 | 0 | 1.44 A |
| 0 | 0 | 1 | 1.22 A |
| 0 | 1 | 0 | 0.93 A |
| 0 | 1 | 1 | 0.83 A |
| 1 | 0 | 0 | 0.77 A |
| 1 | 0 | 1 | 0.70 A |
| 1 | 1 | 0 | 0.59 A |
| 1 | 1 | 1 | 0.55 A |

The repository selects 1/16 microstepping and the lowest 0.55 A setting for first bring-up. Do not increase current until the installed motor's rated phase current and thermal behavior are known.

## Power evidence boundary

- The D36A manufacturer manual and troubleshooting guide use an external 12 V supply for the complete module.
- ATD5984 specifies a 5.5-28 V motor-supply operating range.
- RT8279 specifies a 5.5-36 V input range for the onboard 5 V buck converter.
- Those chip limits show why a 7.4 V source is electrically plausible; they do not establish D36A torque, speed, fan rail, low-battery behavior, or balance performance at 7.4 V.
- Never parallel the D36A 5 V output with an independently regulated logic rail.

Follow the user-confirmed 7.4 V direct-feed project decision only as recorded in `硬件.md`, and require loaded measurements from full charge through the intended low-battery condition.

## Mechanical evidence boundary

The supplied mechanism contains one 42-frame stepper, its bracket, a clamp-style coupler, a crank, a lifting arm, and an angle bracket. The assembly uses:

- four M3x7 screws for the motor bracket;
- three M3x8 screws for the crank-to-coupler joint;
- M4x16 screws and locknuts for the crank/lifting-arm and lifting-arm/beam pivots.

Do not overtighten either moving M4 locknut; excessive preload obstructs the linkage. The supplied documents do not establish motor model, phase current, winding colors, absolute zero, allowable beam angle, direction polarity, closed-loop gains, or safe travel. Keep those as calibration facts.
