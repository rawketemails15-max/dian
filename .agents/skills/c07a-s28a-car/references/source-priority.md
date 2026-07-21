# Source priority

Use the narrowest source that is authoritative for the fact being decided. A higher-ranked source in one category does not answer a different category.

| Fact category | Priority, highest first | Establishes | Must not be used to infer |
|---|---|---|---|
| 1. Board-level facts | Repository-root [`硬件.md`](../../../../硬件.md); actual C07A/S28A schematic and wiring/resource map | Fixed project wiring, connector routing, pin reservations, module assignment, confirmed power decisions | MCU pin capability not shown by the board document; generated names; SDK APIs |
| 2. External-module hardware | Bundled manufacturer/vendor module documents; confirmed measurements recorded in `硬件.md` | Module voltage, signal type, timing range, mechanical data, connector/wire definitions, and documented channel behavior | C07A/S28A wiring; MCU pinmux; SysConfig fields; generated macros; DriverLib APIs |
| 3. Chip hardware capability | Current official MSPM0G3507 datasheet; current MSPM0 G-Series Technical Reference Manual; current device errata | Physical pin/package presence, mux routes, peripheral hardware, ADC channels, electrical limits, silicon behavior | Current `.syscfg` schema, generated macros, IRQ/ISR spelling, current DriverLib signatures |
| 4. SysConfig configuration facts | Active project `.syscfg`; current SDK metadata; matching current-SDK official examples; SysConfig CLI/generation result | Device/package/product metadata, supported modules/fields/enums, configured instances and pins, generation diagnostics | Board wiring; C macro spelling before generation |
| 5. C macros and instance facts | Active generated `ti_msp_dl_config.h` | Exact instance, port, pin, IRQ, ISR alias, init function, timer, ADC, UART, and other generated tokens | Physical wiring not represented by the active configuration |
| 6. DriverLib API facts | Current installed SDK headers; same-version API docs; same-version device-matching official examples | Exact function names, signatures, enums, structures, and supported usage | APIs from a different SDK version or device family |
| 7. K230 CanMV API facts | Installed CanMV firmware version/build and runtime help; matching-version CanMV API docs; verified same-version example; bundled unknown-version examples last | Exact FPIOA, UART, Sensor, Display, MediaManager, image, KPU, touch, and network APIs available on the target | Project wiring, APIs from another firmware, missing model/assets, or host-side Python behavior |

## Local materials already present

- TI MSPM0G350x datasheet: [`official/ti/mspm0g3507.pdf`](official/ti/mspm0g3507.pdf), TI document SLASEX6C (Rev. C). Use Table 6-2 for the C07A's 48-pin LQFP pin map.
- C07A core-board schematic: [`board-docs/c07a/C07A核心板原理图_V1.1（MSPM0G3507）.pdf`](board-docs/c07a/C07A核心板原理图_V1.1（MSPM0G3507）.pdf). It identifies the MCU as MSPM0G3507SPTR.
- S28A schematic: [`board-docs/s28a/4.C07A适配S28A底板原理图.pdf`](board-docs/s28a/4.C07A适配S28A底板原理图.pdf).
- S28A resource map: [`board-docs/s28a/3.C07A搭配S28A底板资源分配表25.7.29.pdf`](board-docs/s28a/3.C07A搭配S28A底板资源分配表25.7.29.pdf).
- Complete board-document inventory: [`board-docs/index.md`](board-docs/index.md).
- Hiwonder LDX-227 V1.0 manufacturer datasheet and evidence boundaries: [`module-docs/ldx-227/index.md`](module-docs/ldx-227/index.md).
- Hiwonder LD-3015MG V1.0 manufacturer datasheet at repository root, the installed PAN/TILT model assignment, and cross-model bring-up boundaries: [`module-docs/gimbal-servos.md`](module-docs/gimbal-servos.md).
- Eight-channel grayscale user manual and two vendor/example documents, including fixed project binding and example-code boundaries: [`module-docs/grayscale/index.md`](module-docs/grayscale/index.md).
- K230 Kendryte datasheet snapshot, Hiwonder development-board V1.0 schematic, and 74 CanMV/MicroPython reference examples: [`module-docs/k230/index.md`](module-docs/k230/index.md).
- Known-tested K230 runtime evidence from a prior integrated build: `CanMV v1.4-19-ga7de1c8` on `k230_canmv_hiwonder`; runtime introspection confirmed that build's UART1 TX/RX FPIOA tokens and UART instance. Re-probe the active device after any firmware change.

The bundled S28A files are canonical copies. The former nested duplicate set was byte-identical by SHA-256 and was removed during packaging.

Method-only reference: [`mc3545dada/mspm0-skill`](https://github.com/mc3545dada/mspm0-skill) may inform project inspection, local SDK example search, and generated-name checks. It has no authority over C07A/S28A wiring, reservations, or defaults; never import its Tianmengxing board assumptions.

## Missing or version-sensitive references

- Current MSPM0 G-Series 80MHz Technical Reference Manual.
- Current MSPM0G350x silicon errata.
- The actual CCS/SysConfig project, installed MSPM0 SDK, SDK API docs/examples, and exact CCS/SysConfig/compiler versions.
- Matching CanMV API documentation for the known-tested build, the active device's currently installed firmware/build, exact touch-panel variant, and the model/utility assets referenced under `/sdcard` by many examples.
- Confirmed redistribution license for the supplied K230 PDFs and example bundle; no license file accompanied them.
- Exact installed grayscale illumination variant, calibrated black/white active polarity, and verified address-settling time. Bundled examples disagree on the delay, so none is a final project value.
- Selected servo-rail regulator/BEC or direct-battery design documentation; the final supply topology is intentionally undecided.
- Motor/encoder datasheet and measured polarity, reduction ratio, PPR, wheel diameter, and track width.
- OLED controller/protocol documentation if its existing driver cannot establish the four signal roles.

Do not fill these gaps from memory. Record an unknown as an unknown and stop only the part of the task that depends on it.
