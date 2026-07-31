# 26H 第五问：启停角度补偿与终点平滑停车

详细操作、调参和验证说明见同目录 `README.md`。

最常调的三个参数位于 `app_config.h`：

```c
#define APP_Q5_BALL_START_ACCEL_COMP_STEPS     (12.0f)
#define APP_Q5_BALL_BRAKE_ACCEL_COMP_STEPS    (-12.0f)
#define APP_Q5_FINISH_ALIGN_PWM                (1800U)
```

- 启动钢球仍向左（x 减小）：启动补偿每次增加 2～4 微步；若更严重则反向调节。
- 刹车钢球仍向右（x 增大）：刹车补偿每次增加负值绝对值 2～4 微步；若更严重则向 0 或正向调节。
- 终点后不足约 34 cm：小幅增加终点 PWM；超过约 34 cm：降低终点 PWM。

循迹启动过程为 100 ms 预倾和 500 ms 平滑加速。连续两次检测到至少四路相邻黑线后，执行 100 ms 低速摆正和 900 ms 线性减速，随后刹车。K230、UART 和引脚分配均未修改；SysConfig 仅将 OLED 的 PB14/RST 上电初值改为低。

`SYSCFG_DL_init()` 完成基础引脚配置后，OLED 会在所有其他应用驱动之前初始化；启动时先全屏点亮约 300 ms，再显示 `0.0s`，用于确认 PB14 复位及 PA28/PA31 软件 SPI 通信正常。
