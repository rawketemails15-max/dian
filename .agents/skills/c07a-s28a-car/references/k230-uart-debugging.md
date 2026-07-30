# K230 bidirectional UART bring-up playbook

Use this playbook when K230 vision/display works but MSPM0 results do not reach it, MSPM0 does not receive K230 coordinates, or communication becomes unreliable after integration.

Repository-root `硬件.md` remains authoritative for wiring and power. Re-read the active `.syscfg`, generated `ti_msp_dl_config.h/.c`, installed SDK headers, and deployed K230 source before reusing historical tokens.

## Previously verified baseline

The following facts were observed on a prior integrated build:

- K230 runtime: `CanMV v1.4-19-ga7de1c8`, MicroPython `e00a144`, build date 2025-11-06, board string `k230_canmv_hiwonder`.
- GC2093 CSI2 camera and 800×480 ST7701 LCD operated together in the observed scene.
- K230 40Pin pin 8 TX1(IO3)/UART1_TXD connected to MSPM0 PB7/UART1_RX.
- MSPM0 PB6/UART1_TX connected to K230 40Pin pin 10 RX1(IO4)/UART1_RXD.
- Both directions used 115200 baud, 8 data bits, no parity, 1 stop bit, and a common ground.

Do not substitute the debug-header TX3/RX3, the superseded GPIO11/GPIO12 UART2 route, or unrelated pins from examples. These observations prove only that historical hardware/runtime combination.

## Diagnose each direction separately

### K230 to MSPM0

1. Confirm K230 IO3 is mapped to `FPIOA.UART1_TXD` and the UART object is `UART.UART1` on the active firmware.
2. Scope IO3 and then PB7. An IO3 waveform that does not reach PB7 is a wiring or contact fault.
3. If bytes reach PB7 but the MSPM0 sees no frame, inspect the active PB7 pinmux, UART RX instance, baud/framing, IRQ enable, FIFO drain, parser counters, and interbyte timeout.
4. A valid ball-position or line-result update on the MSPM0 proves this direction for that run; it does not prove the return direction.

### MSPM0 to K230

1. Scope PB6. Expect idle near 3.3 V and application-defined status bursts.
2. Scope the actual K230 40Pin pin 10/IO4 pad, not only the MSPM0 end of the jumper.
3. PB6 activity with no IO4 activity is wiring, connector position, or contact continuity.
4. PB6 and IO4 activity with a raw receive count of zero points to K230 FPIOA/electrical configuration or CanMV UART reception.
5. Raw bytes with no valid frames point to baud, framing, length, checksum, buffer, timeout, or resynchronization logic.
6. No PB6 activity points to the active MSPM0 image, application liveness, PB6 pinmux, UART TX service, or scheduling.

## Historical K230 IO4 receive configuration

A prior program mapped `UART1_RXD` with only `ie=1` and used a zero read timeout. Its raw receive count stayed zero. The historical hardware began receiving valid status only after applying this combined configuration:

```python
fpioa.set_function(
    3, FPIOA.UART1_TXD,
    ie=0, oe=1, pu=0, pd=0)
fpioa.set_function(
    4, FPIOA.UART1_RXD,
    ie=1, oe=0, pu=1, pd=0, st=1)

uart = UART(
    UART.UART1,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
    timeout=2)

fpioa.help(4)
```

The 2 ms timeout covers approximately 23 bytes at 115200 8N1 while staying short relative to a camera loop. Explicit IO4 input enable, output disable, pull-up, pull-down disable, and Schmitt input prevent unintended retained attributes. Confirm the mapping with `fpioa.help(4)`.

The individual change responsible was not isolated. Do not claim that the timeout, pull-up, or Schmitt input alone fixed the issue, and revalidate every token and electrical attribute on the active CanMV firmware.

## RX-only and loopback isolation

If the normal camera application still receives nothing:

1. Create a temporary diagnostic that skips camera and LCD initialization.
2. Map only IO4 as UART1 RX, call `uart.read()`, print raw byte blocks, and count raw bytes, valid frames, checksum errors, format errors, and resynchronizations.
3. If IO4 has a valid 3.3 V waveform and RX-only mode still sees no bytes while `fpioa.help(4)` is correct, run a controlled UART1 TX-to-RX loopback.
4. Only after raw bytes arrive should protocol parsing be debugged.

Keep a disconnected UART RX pin at idle high. Drain RX FIFO and clear peripheral/NVIC pending state before enabling the UART IRQ. Give the real-time control tick higher priority than UART reception when the active architecture requires it.

## Protocol robustness

- Separate UART buffering from camera/display work.
- Handle fragmented frames, concatenated frames, invalid lengths, bad checksums, interbyte timeouts, and resynchronization.
- Keep raw-byte, valid-frame, checksum-error, format-error, and timeout counters visible on the diagnostic display or console.
- On stale/invalid vision data, move the actuator only according to the defined safety state; never turn an absent frame into an invented ball coordinate.

## Validation boundary

Report static source inspection, SysConfig generation, compile/link, flashed artifact identity, waveform at the source pin, waveform at the receiving pad, raw bytes, valid frames, and closed-loop behavior separately. A successful build, one toggling TX pin, or one working direction does not prove bidirectional UART.
