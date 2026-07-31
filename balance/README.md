# 26H：钢球静态居中与循迹程序

工程默认运行“车轮持续刹车、球杆保持钢球中心”的静态模式。原有循迹代码
仍保留，通过 `app_config.h` 中的 `APP_OPERATION_MODE` 切换。

## 标定结果

本机已经完成首次标定，当前正常闭环配置为：

```c
#define APP_BALL_CALIBRATION_MODE (0U)
#define APP_BALL_MIN_STEPS        (-238)
#define APP_BALL_MAX_STEPS        (238)
#define APP_BALL_DIR_INVERT       (0U)
```

每次上电前仍需手动把球杆放到近似水平；此位置被定义为相对 0 步。如果机构
拆装或电机方向改变，应临时改回标定模式并重新测量：

- PA18 短按：按当前方向以 100 Hz 点动 16 微步；
- PA18 长按 0.6～2 秒：切换方向，不运动；
- 持续按住满 2 秒：立即停止 STEP 并锁定故障；
- OLED 显示 DIR 实际电平、当前位置 POS、已到达 MIN/MAX；
- 标定固件硬限制为 ±256 微步，不能代替机械限位。

分别找到两个方向无卡滞的最大位置，各向内减 16 步作为安全余量，填写
`APP_BALL_MIN_STEPS` 和 `APP_BALL_MAX_STEPS`。根据实际升降方向填写
`APP_BALL_DIR_INVERT`，然后把 `APP_BALL_CALIBRATION_MODE` 改为 `0U`
并重新构建、烧录。

## 接线与正常闭环

- K230 IO3/UART1_TXD → MSPM0 PB7/UART1_RX；
- MSPM0 PB6/UART1_TX → K230 IO4/UART1_RXD；
- PB16/TIMG7_C1 → D36A STEP1；
- PB17 → D36A DIR1；
- PA24 → D36A EN1/SLEEP#；
- 所有模块必须共地。

正常模式连续收到 5 帧有效视觉数据后才唤醒 D36A，并保证唤醒后至少 1 ms
才输出 STEP。目标中心默认 X=160 px，视觉超过 150 ms 未更新时回到相对
0 步，超过 1 s 显示视觉故障。OLED 显示视觉状态、球坐标/误差、当前/目标
步数和 CRC 错误计数。

K230 程序为 `K230/steel_ball_motion_wifi_320_full_v3.py`，保留 LCD、热点和
RTSP，并通过 UART 以约 30 Hz 发送带 CRC16 的坐标帧；电脑不需要先连接热点。

## 原有循迹

将 `APP_OPERATION_MODE` 改回 `APP_OPERATION_MODE_LINE_FOLLOW` 可恢复原循迹
状态机。灰度 PID、启停线、里程和电机 PI 参数仍集中在 `app_config.h`。
