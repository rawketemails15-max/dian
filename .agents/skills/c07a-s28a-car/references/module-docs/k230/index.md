# K230 module and CanMV reference

Read this file before changing K230 CanMV/MicroPython, camera, display, UART, vision, touch, or HMI code. Repository-root [`硬件.md`](../../../../../../硬件.md) remains authoritative for the project's physical wiring.

## Bundled documents

- [`01 K230 Product Full Datasheet — K230 Linux+RT-Smart SDK.pdf`](docs/01%20K230%20Product%20Full%20Datasheet%20%E2%80%94%20K230%20Linux%2BRT-Smart%20SDK.pdf) is a 32-page local snapshot of the Kendryte K230 web datasheet captured on 2025-07-15. It establishes SoC capability such as five UARTs, 64 GPIOs, MIPI interfaces, and chip electrical conditions. It does not establish Hiwonder board routing or CanMV API spelling.
- [`02 SCH_K230_DK-board V1.0.pdf`](docs/02%20SCH_K230_DK-board%20V1.0.pdf) is a six-page Hiwonder K230 development-board schematic, V1.0, dated 2025-07-21 in the drawing. It establishes board nets, a 3.5-inch 800×480 MIPI LCD interface, and a 40Pin header that includes TX1/RX1. It does not override this project's fixed GPIO11/GPIO12 UART wiring.

Do not use generic SoC electrical tables alone to claim the logic level of a particular board connector. Trace the board schematic and verify the installed board and IO-bank supply.

## Bundled example set

The [`examples/`](examples/) directory contains 74 user-provided Hiwonder CanMV/MicroPython `.py` files. They pass CPython syntax compilation, but that does not prove compatibility with the installed CanMV firmware or hardware.

No firmware-version manifest, API manual, model files, utility assets, or license file accompanied the examples. Preserve the examples as reference evidence; do not describe the whole set as runnable or redistributable without resolving those gaps.

High-value routes for this car project:

| Task | Start with | Evidence boundary |
|---|---|---|
| UART and FPIOA | [`UART.py`](examples/UART.py), [`FPIOA.py`](examples/FPIOA.py) | Shows one environment's `UART1` on GPIO3/GPIO4 and LED52; these are not project pins |
| Camera/media lifecycle | [`Sensor.py`](examples/Sensor.py), [`Display.py`](examples/Display.py) | API calls and cleanup are firmware-sensitive; display modes vary across examples |
| Vision line tracking | [`line_patrol.py`](examples/line_patrol.py) | LAB threshold, ROIs, and 800×480 scaling require track/camera calibration; example has no MSPM0 UART output or complete cleanup |
| Color and geometry | [`Single_color_recognition.py`](examples/Single_color_recognition.py), [`Multiple color recognition.py`](examples/Multiple%20color%20recognition.py), [`Line_segment_detection.py`](examples/Line_segment_detection.py), [`circular_detection.py`](examples/circular_detection.py), [`rectangular_detection.py`](examples/rectangular_detection.py) | Thresholds and geometry parameters are examples, not project calibration |
| Tags and codes | [`AprilTags.py`](examples/AprilTags.py), [`qrcodes.py`](examples/qrcodes.py), [`barcode.py`](examples/barcode.py), [`Data Matrix.py`](examples/Data%20Matrix.py) | Select only the detector needed by the task and verify firmware support |
| Touch and local HMI | [`touch.py`](examples/touch.py), [`touch_draw.py`](examples/touch_draw.py), [`touch_photo.py`](examples/touch_photo.py), [`Lvgl.py`](examples/Lvgl.py) | Requires the matching LCD/touch stack and, for LVGL, unbundled SD-card assets |
| Network transport | [`TCP-Client.py`](examples/TCP-Client.py), [`UDP-Client.py`](examples/UDP-Client.py), [`HTTP-Client.py`](examples/HTTP-Client.py) | Demo credentials/endpoints are not production configuration |
| KPU/AI | Detection, recognition, OCR, gesture, pose, ASR, TTS, and LLM examples | Most require unbundled `/sdcard/examples/kmodel`, `utils`, audio, database, or network assets |

`VLLM_demo.py` is a PC-side Flask relay, not K230 firmware. Keep host-side tools separate from code deployed to CanMV.

## Mandatory K230 workflow

1. Read `硬件.md`, then identify the exact Hiwonder board revision, installed CanMV firmware/build, camera sensor, LCD/touch hardware, and available SD-card assets.
2. Preserve the project connection: K230 GPIO11/UART2_TXD goes to MSPM0 PB7/UART1_RX; MSPM0 PB6/UART1_TX goes to K230 GPIO12/UART2_RXD; all grounds are common.
3. Do not copy the bundled UART example's `UART.UART1`, GPIO3, GPIO4, or LED52. Do not switch to the schematic's 40Pin TX1/RX1 merely because it is documented.
4. Before writing K230 UART code, prove the exact UART2 and FPIOA token names using the installed firmware's API documentation or runtime `FPIOA.help()`/introspection. The example proves only the UART1 spellings in its own unknown firmware version.
5. Configure or verify 115200 baud, 8 data bits, no parity, and 1 stop bit according to the exact current UART API. Keep TX/RX crossed and verify logic levels before connecting.
6. Treat `UART.py` as a one-byte LED demonstration, not as the car protocol. Add framing, buffering, validation, timeout/recovery, and bidirectional command/result handling according to `project-architecture.md`.
7. Keep image thresholds, ROIs, resolution, frame rate, and 30 Hz result publishing separately configurable. Do not make the camera loop block on UART or LCD work.
8. Use `try/finally` cleanup appropriate to the current firmware. Verify `sensor.stop()`, `Display.deinit()`, `MediaManager.deinit()`, UART deinit, and `os.exitpoint()` availability before copying those calls.
9. Validate in layers: syntax/imports, peripheral initialization, camera/display output, UART loopback, MSPM0 interoperability, then full on-car behavior. A script compiling under desktop Python proves only syntax.

## Security and dependency boundaries

- Never commit real Wi-Fi passwords, API keys, tokens, or service credentials. Several network/AI examples contain placeholders, and some contain the demo pair `hiwonder`/`hiwonder`; replace configuration through a local untracked mechanism.
- Do not invent missing `.kmodel`, font, image, audio, database, or `/sdcard` files. List each required artifact and stop that feature until it is supplied.
- Verify permission before publicly redistributing the PDFs and example bundle; no license file was included with the supplied K230 materials.
