---
name: c07a-s28a-car
description: Grounded workflow for the C07A + S28A NUEDC control car using TI MSPM0G3507, CCS, SysConfig, MSPM0 SDK, DriverLib, K230 HiWonder CanMV/MicroPython, and bidirectional UART. Use when Codex inspects, creates, modifies, builds, flashes, debugs, or diagnoses this repository's MSPM0 firmware or .syscfg files; writes or diagnoses K230 camera, vision, display, touch, UART, or HMI code; configures GPIO, UART, I2C, SPI, timers, PWM, ADC, or interrupts; integrates the motors, encoders, LD-3015MG/LDX-227 gimbal servos, MPU6050, eight-channel grayscale sensor, OLED, K230, or power wiring; or checks C07A/S28A pinmux and board conflicts. Do not apply its board-specific pin assumptions to other MSPM0 boards.
---

# C07A + S28A Car

## Ground every task in repository facts

1. Locate the active project root, then search that directory and its parents for the repository-root `硬件.md`. Do not assume a Git worktree or a fixed relative depth. If more than one candidate can govern the active project, resolve the project boundary before continuing.
2. Read the complete discovered `硬件.md` before inspecting or changing MSPM0 code.
3. Treat repository-root `硬件.md` as the only editable board-level hardware fact source. Read [references/hardware.md](references/hardware.md) for the no-duplication contract.
4. Never reassign a fixed project pin to resolve a conflict. Report the conflict and request a hardware decision.
5. Treat C07A/S28A labels as board wiring facts, not as proof of TI peripheral names, generated macros, or SysConfig schema.
6. Treat bundled external-module documents as evidence only for that module's electrical, mechanical, and signal behavior. Never copy their other-board pin maps, wrapper names, generated-looking macros, or SDK calls into this project without current-project evidence.

Always read [references/source-priority.md](references/source-priority.md). For pin or connector work, also read [references/pinmux-summary.md](references/pinmux-summary.md) and [references/s28a-board-map.md](references/s28a-board-map.md). For UART behavior, control conventions, HMI, ISR structure, or bring-up work, read [references/project-architecture.md](references/project-architecture.md). For K230 CanMV/MicroPython, camera, display, vision, touch, UART, or HMI work, read [references/module-docs/k230/index.md](references/module-docs/k230/index.md). For either gimbal servo's power, PWM, wiring, connector, or mechanical work, read [references/module-docs/gimbal-servos.md](references/module-docs/gimbal-servos.md). For eight-channel grayscale wiring, timing, GPIO, or control work, read [references/module-docs/grayscale/index.md](references/module-docs/grayscale/index.md). For any `.syscfg`, generated-code, DriverLib, build, or debug work, read [references/sysconfig-workflow.md](references/sysconfig-workflow.md). Use [references/official/index.md](references/official/index.md) and [references/board-docs/index.md](references/board-docs/index.md) to locate the bundled TI and board documents.

If any CCS Project, SysConfig, Debug, or Serial MCP tool is missing, disconnected, or unexpectedly unavailable, read and follow [references/ccs-mcp-troubleshooting.md](references/ccs-mcp-troubleshooting.md) before blaming the project, editing configuration files, or switching to unsupported build/configuration paths.

## Mandatory workflow

### A. Before modifying an MSPM0 project

1. Locate the project root and distinguish it from the repository root when the project is nested.
2. Read the complete repository-root `硬件.md`.
3. Find every `.syscfg`, then identify the one actually used by the active project. Do not choose arbitrarily when multiple files exist.
4. Read its metadata exactly as written: `device`, `package`, SDK product/version, and SysConfig version. Preserve `@cliArgs`, `@v2CliArgs`, and `@versions` metadata.
5. Find the current generated `ti_msp_dl_config.h` and `ti_msp_dl_config.c`. Treat all copies as read-only evidence and determine which build configuration produced the active copy.
6. Inspect the project structure and current toolchain, including CCS project metadata, generated build rules, compiler, target configuration, and any existing build command.
7. List every pin that the requested change will use or modify, including power, ground, and external-module signal pins.
8. Check those pins and their purposes against `硬件.md`; report conflicts before editing.

Run `python .agents/skills/c07a-s28a-car/scripts/inspect_mspm0_project.py <project-root>` as a read-only preflight when available. Treat its output as an index to source files, not as a replacement for reading them. If the actual `.syscfg`, metadata, or generated header is absent, state which fact is missing instead of inventing it.

### B. When deciding whether a physical pin supports a function

Apply this priority:

1. This project's `硬件.md` and actual C07A/S28A schematic or wiring diagram for board-level wiring and reservations.
2. The current official TI MSPM0G3507 datasheet for physical-pin existence, pin mux, UART/I2C/SPI functions, timer/PWM routes, ADC channels, and electrical limits.
3. The current `device`/`package` SysConfig Device View and metadata for availability in the selected part and tool version.

Never use the datasheet to infer current generated macros, SysConfig instance names, IRQ macros, ISR names, or version-specific `.syscfg` fields. Do not import Tianmengxing or another development board's pin reservations.

### C. When modifying SysConfig

1. Do not guess `.syscfg` fields, enum values, module names, or instance names.
2. Inspect the active project's existing `.syscfg` first.
3. Inspect official examples from the currently installed MSPM0 SDK that match the device, SDK version, and peripheral.
4. If still necessary, inspect the same SDK's SysConfig metadata, including relevant `.meta/*.syscfg.js` files.
5. Change only the smallest necessary configuration surface and preserve unrelated settings and metadata.
6. Never hand-edit generated `ti_msp_dl_config.c` or `ti_msp_dl_config.h` files.
7. Regenerate with the project's real SysConfig/build flow or rebuild the project after the change.

### D. Before writing or updating C code

Regenerate first, then reread the active generated `ti_msp_dl_config.h`. Obtain every peripheral instance macro, GPIO port macro, GPIO pin macro, IRQ number, IRQ handler, SysConfig init function, timer instance, ADC instance, and UART instance from that generated header or the current SDK headers.

Never construct names from memory, including `UART_0_INST`, `TIMER_0_INST`, `GPIO_PORT`, or `UART0_IRQHandler`, unless the exact token exists in the active generated file. Use semantic names in prose only; use generated names in code.

### E. When using DriverLib

Obtain function names, parameters, and enum values from, in order:

1. Headers in the currently installed SDK.
2. API documentation for that same SDK version.
3. Official examples matching the current device and SDK.

Do not infer a DriverLib API from an online example for another SDK version. If local evidence is unavailable, stop at a documented code plan rather than fabricating a compiling call.

### F. Compile and generation validation

1. Run the project's real build command after modification; do not invent a build path.
2. On failure, capture and read the complete first effective error, not only the final summary.
3. Do not try successive guessed macros or enum values.
4. Return to the generated header, SDK header, SDK metadata/example, and `.syscfg` to resolve the fact.
5. Classify the result as a SysConfig generation error, compile error, link error, flash/debug error, or runtime/hardware error.

Report SysConfig warnings separately. A warning-producing generation is not clean validation.

### G. Hardware validation

When code builds but hardware does not respond, check in this order:

1. Power and required current capacity.
2. Common GND.
3. Logic levels.
4. UART TX/RX crossover.
5. I2C pull-ups.
6. External-module operating or boot mode.
7. Actual soldering and connector continuity.
8. Pin mux.
9. Peripheral instance.
10. Interrupt enable and handler path.

Never describe compile success as hardware validation. State the last verified layer: source inspection, SysConfig generation, compilation, linking, flashing/debugging, or real-hardware behavior.

### H. When modifying K230 CanMV/MicroPython

1. Identify and record the exact board revision, CanMV firmware/build, camera sensor, display/touch hardware, and required SD-card assets.
2. Keep GPIO11/UART2_TXD and GPIO12/UART2_RXD fixed. Never copy UART1/GPIO3/GPIO4/LED52 from the bundled reference example or the board schematic's 40Pin TX1/RX1 route.
3. Obtain FPIOA, UART, Sensor, Display, MediaManager, image, and KPU API names from the installed firmware's matching documentation, runtime help, or a verified same-version example.
4. Treat every bundled example as reference code with an unknown firmware version. Check missing models/assets, hard-coded thresholds, credentials, cleanup, blocking behavior, and host-side versus device-side role before reuse.
5. Run syntax/import checks and then test peripheral initialization on the actual K230. Test UART loopback and MSPM0 interoperability before integrating vision or HMI traffic.
6. State the last verified layer. Desktop syntax success is not CanMV execution, UART interoperability, camera validation, or on-car validation.

## Preserve project boundaries

- Preserve user code, project structure, comments, compiler choice, SDK version, device/package, and unrelated `.syscfg` settings.
- Do not choose between direct battery power and an independent BEC for the servo rail without user confirmation.
- Keep K230 CanMV/MicroPython changes separate from MSPM0 DriverLib facts; verify the exact K230 firmware/API version before using version-sensitive APIs.
- Ask for missing high-risk hardware facts, but continue all safe read-only inspection and report precisely what remains unknown.
