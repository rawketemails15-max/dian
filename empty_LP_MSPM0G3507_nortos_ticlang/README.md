## Example Summary

Empty project using DriverLib.
This example shows a basic empty project using DriverLib with just main file
and SysConfig initialization.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## Example Usage

Compile, load and run the example.

## 小车任务启动方式

- 短按 BLS：等待 1 秒双击判定窗口结束后执行第一阶段，直行检测到黑线后停车并蜂鸣。
- 按住 BLS 1 秒：执行原四段完整路线。
- 1 秒内短按 BLS 两次：执行第三问。第二次必须为短按；若第二次按住达到 1 秒，则仍执行原四段完整路线。

根据 `24H控制.pdf` 的题图，A、C 为一条斜对角线，B、D 为另一条斜对角线，第三问路线为 A→C→B→D→A。以启动时车头方向为 0 度，并规定左转角度增大、右转角度减小：先右转到 -40 度，沿 AC 直行；到 C 后循迹半圆至 B，丢线后补到累计 180 度，再左转到 220 度，沿 BD 直行；到 D 后循迹半圆至 A，丢线后补到累计 360 度（与 0 度同向），再右转到 320 度（与 -40 度同向），开始下一圈。后续补角基准按 540、720……递增，奇数半圆在基准上加 40 度左转，偶数半圆在基准上减 40 度右转。上述逻辑共循迹 8 个半圆，并在第 8 个半圆丢线点停车。每次到达 C、B、D、A 时，PA9 蜂鸣器与 PB9 指示灯同步提示一次。
