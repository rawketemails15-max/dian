# 26H H题要求2：循迹一圈并停回 A 点

本工程面向 C07A + S28A（MSPM0G3507，LQFP-48），实现：

- PA18 按键启动并开始计时；
- 八路红外灰度沿黑线顺时针循迹；
- 使用 5 ms 灰度位置闭环 PID，带积分限幅、微分滤波、中心死区和 PWM 斜坡；
- 识别 A 点宽启停线后制动并冻结 OLED 时间；
- 100 ms 丢线、300 ms 堵转和 20 s 超时保护；
- 完成或故障后再次按键可重新测试。

工程不配置 MPU6050、I2C 或 PA7 中断，只需电机、编码器、灰度、按键和
OLED 即可运行本题功能。上车时需让车头在 A 点朝顺时针方向放置，并以灰度
传感器中心在车身中心轴上的标记作为比赛唯一测试位置。

## 集中调参

常用实车参数都在 `app_config.h`：

- `APP_TRACK_BASE_PWM`：正常循迹 PWM，默认 3000；
- `APP_TRACK_CENTER_DEADBAND`：中心误差死区，默认 50；
- `APP_TRACK_ERROR_FILTER_DIVISOR`：误差低通系数，默认 4；
- `APP_TRACK_PWM_SLEW_PER_TICK`：每 5 ms 单轮 PWM 最大变化量，默认 200；
- `APP_LINE_PID_KP/KI/KD`：灰度位置 PID，默认 P=7、I=1/64、D=10；
- `APP_LINE_PID_OUTPUT_LIMIT`：PID 最大差速修正，默认 3000；
- `APP_FINAL_APPROACH_COUNTS`：末段减速里程，默认 45500；
- `APP_MARKER_MIN_CONTIGUOUS_CHANNELS`：A 线连续黑色通道阈值，默认 4。

首次实车应先架空确认两轮前进方向、编码器计数和灰度黑线高电平，再在赛道
上逐步提高速度。默认里程按约 7640 计数/米估算，需要用实际一圈编码器数据
校正。
