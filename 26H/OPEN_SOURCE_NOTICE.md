# Open-source balance-controller adaptation

The questions 4, 5 and 6 ball-balancing state logic in
`ball_rod_control.c` is adapted from:

`2026diansaikaiyuanqingdane-master/H题/上车后平衡球开源代码/平衡球控制/balance_controller.py`

The upstream repository is licensed under the Academic Free License 3.0.
See `LICENSE.AFL-3.0` for the complete license text.

This MSPM0 derivative is materially modified for the installed hardware:

- The upstream ZDT closed-loop absolute-angle motor interface is replaced by
  D36A STEP/DIR relative microsteps.
- The upstream mechanical 40..100 degree absolute-angle limits are not copied.
  This project keeps the existing manually levelled zero, +/-64 microstep work
  range and unconditional -238..238 microstep software travel limits.
- Direct integer-position least-squares velocity estimation, approach-gated
  velocity damping, progressive breakaway compensation, settled-angle latch,
  slow disturbance trim, endpoint recovery, vision-loss leveling and bounded
  output slew are retained in adapted form.
- PC UI, serial ZDT protocol, automatic direction calibration, adaptive tuning,
  trajectory experiments and unsafe large hard-pulse angles are not included.

The OLED question selector, PA18 interaction, chassis line following, K230 UART
protocol, SysConfig and pin assignments are unchanged by this adaptation.
