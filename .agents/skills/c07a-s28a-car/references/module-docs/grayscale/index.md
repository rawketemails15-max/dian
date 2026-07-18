# Eight-channel grayscale module reference

Read these bundled vendor documents as module evidence:

- [`用户入门手册.pdf`](用户入门手册.pdf): electrical/mechanical facts, channel truth table, variants, and working-height guidance.
- [`数据读取.pdf`](数据读取.pdf): a digital-read example for a different Yabo MSPM0G3507 board.
- [`八路灰度模块.pdf`](八路灰度模块.pdf): another-board wiring and wrapper-code example, including configurable active value.

Repository-root [`硬件.md`](../../../../../../硬件.md) remains authoritative for this project's fixed wiring and confirmed electrical facts.

## Established module facts

- Supply the module with 5 V.
- Read `OUT` as a digital high/low signal from the CD4051 output. The project-confirmed maximum OUT level is 3.3 V.
- Drive AD0 as the least-significant address bit and AD2 as the most-significant bit: `000` selects CH1 through `111` selecting CH8.
- Leave EN to its onboard 10 kΩ pulldown during normal use; no external EN control is required.
- The board is approximately 91×32.50 mm and has eight probes. The recommended height is 18 mm.
- Red, green, blue, and white illumination variants exist. The installed project variant is not yet identified.
- The white-light variant lists 10~30 mm operation on both white and black backgrounds. Red/green/blue variants list 16~24 mm on white and 12~20 mm on black.

## Fixed project binding

Use only the mapping in `硬件.md`:

| Signal | MSPM0 pin | Direction |
|---|---|---|
| AD0 | PA22 | GPIO output |
| AD1 | PA8 | GPIO output |
| AD2 | PA12 | GPIO output |
| OUT | PA27 | Digital GPIO input |
| GND | GND | Common ground |
| VCC | 5 V | Module supply |

Do not configure PA27 as an ADC input for this module.

## Example-code boundaries and unresolved behavior

- `数据读取.pdf` uses PA14/PA15/PA16/PA17 on another board. `八路灰度模块.pdf` uses abstract X1/X2/X3/X4 pins. Neither mapping may replace PA22/PA8/PA12/PA27.
- Example helper names and generated-looking tokens are not TI DriverLib or current-project macro evidence. Recreate the behavior through the active `.syscfg`, generated `ti_msp_dl_config.h`, and current SDK headers.
- The documents disagree on address-settling delay: example code uses 50 us and 100 us, while prose mentions 1 ms. Keep the delay configurable and determine a reliable value through conservative hardware testing; do not silently select one as a manufacturer guarantee.
- One example reports a lit channel indicator as digital `1`, and another exposes an `ACTIVE_VALUE` option. Neither establishes black-line versus white-background polarity for every illumination variant and surface. Calibrate the installed hardware and record the resulting polarity before final control tuning.
