# SysConfig and project workflow

## 1. Inspect without changing files

Run the Skill's read-only indexer from the repository root:

```text
python .agents/skills/c07a-s28a-car/scripts/inspect_mspm0_project.py <project-root>
```

Then independently inspect the reported files. Useful searches are:

```text
rg --files <project-root> -g "*.syscfg" -g "ti_msp_dl_config.h" -g "ti_msp_dl_config.c"
rg --files <project-root> -g ".project" -g ".cproject" -g "*.projectspec" -g "*.ccxml" -g "makefile" -g "subdir_rules.mk"
```

When more than one `.syscfg` or generated header exists, trace the active CCS project/build configuration instead of selecting the newest file by timestamp.

## 2. Read metadata literally

Read the active `.syscfg` header comments and preserve all metadata, especially:

- `@cliArgs` or `@v2CliArgs`
- `--device`
- `--package`
- `--product` and its SDK version
- `@versions` and the SysConfig tool version

Do not normalize or upgrade these values while adding a peripheral. Confirm that the selected package is consistent with the C07A's MSPM0G3507SPTR/48-pin LQFP hardware.

## 3. Establish schema and enum facts

Search in this order:

1. The active `.syscfg` for an existing module with the same field pattern.
2. Official examples inside the exact installed MSPM0 SDK.
3. The exact SDK's SysConfig metadata, including relevant `source/ti/driverlib/.meta/*.syscfg.js` files.
4. SysConfig Device View or CLI diagnostics for the exact device/package/product versions.

Use `rg` against the discovered SDK root; do not assume an SDK installation path:

```text
rg -n "<module-or-field>" <sdk-root>/examples <sdk-root>/source/ti/driverlib/.meta
```

An online example may suggest what to search for, but it cannot establish a field, enum, generated macro, IRQ, or API for the local version.

## 4. Make the minimum change

- Preserve `device`, `package`, product/tool versions, clocks, unrelated instances, comments, and pin locks.
- Preserve fixed pin purposes from `硬件.md`.
- Do not copy a complete `.syscfg` from LP-MSPM0G3507, Tianmengxing, or another board.
- Do not edit any `ti_msp_dl_config.c/.h`, object, map, linker output, or generated build file as the configuration fix.

## 5. Regenerate and reread

Use the generation command already recorded by CCS build rules/logs, or invoke the project's existing CCS build. Do not invent a SysConfig CLI path or command line.

After successful generation, reread the active `ti_msp_dl_config.h` and extract exact tokens for:

- init functions
- instance macros
- GPIO ports and pins
- timer/PWM instances
- ADC instances/channels
- UART instances
- IRQ numbers
- ISR or handler aliases

Only then edit application C. Confirm each DriverLib call in the installed SDK headers.

## 6. Build and diagnose by layer

Run the build command proved by the current project metadata/build output. On failure, capture the first effective error with surrounding context and classify it:

| Layer | Typical evidence | Return to |
|---|---|---|
| SysConfig generation | schema, solution, pin conflict, product/version diagnostic | `.syscfg`, Device View, SDK `.meta`, matching example |
| Compile | unknown macro/type/function, bad argument/enum | generated header, SDK header/API docs |
| Link | undefined/multiple symbol, section/memory error | project sources, linker inputs, linker command file |
| Flash/debug | probe, target config, connection, erase/program/reset error | actual probe, `.ccxml`, CCS/UniFlash path |
| Runtime/hardware | build and flash succeed but behavior is wrong | power/GND/levels/wiring/module mode, then pinmux/instance/IRQ |

Do not respond to an unknown macro by trying spelling variants. Find the fact source that owns the name.

## 7. Use semantic documentation names

When creating a new instance and the active project has no conflicting naming convention, prefer purpose-oriented SysConfig instance names such as:

```text
UART_K230
I2C_IMU
PWM_MOTOR
PWM_GIMBAL
GPIO_LINE_ADDR
GPIO_LINE_OUT
GPIO_MOTOR_DIR
GPIO_ENCODER_A
GPIO_ENCODER_B
GPIO_STATUS
```

These are naming recommendations only. Never infer the resulting C macro, IRQ, handler, or instance spelling; reread the generated `ti_msp_dl_config.h`.

## 8. Report the validation boundary

Report exactly which of these layers completed: source inspection, SysConfig generation, compilation, linking, flashing/debugging, and real-hardware observation. A successful build proves no wiring, power integrity, UART crossover, I2C pull-up, mechanical range, or sensor semantics.
