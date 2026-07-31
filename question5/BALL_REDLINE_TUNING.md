# 红线定点闭环：首次台架调试

本版本只做静止底盘上的红线定点。K230 可在触摸屏上拖动红线，
默认目标为 `x=171 px`（`xQ4=2736`），允许范围为 `x=0..319`。
底盘电机始终刹车。程序没有杆角传感器、步进电机
编码器或限位开关，因此每次上电前都必须人工把球杆调平；上电位置即
软件的 `0 step`。

## 操作方式

- 正常模式短按按键：在“等待视觉/闭环运行”和“暂停”之间切换。
- 暂停只停止 STEP，D36A 的 EN 保持高电平，球杆保持当前位置。
- 持续按住 2 秒：急停，STEP 停止且 EN 拉低。急停锁存到下次复位，
  防止误触后自动重新启动。
- `APP_BALL_CALIBRATION_MODE=1` 时进入独立点动模式：移走钢球后，
  单击等待双击窗口结束会正向点动 16 步；双击会负向点动 16 步。

## 必做检查顺序

1. 移走钢球，人工调平球杆后上电。
2. 在点动模式分别验证正、负方向。若物理方向与定义相反，只修改
   `APP_BALL_DIR_INVERT`。
3. 核对相对水平零点及两端安全行程。执行器无条件限制为
   `-238..+238 step`；没有重新实测前不要扩大。
4. 恢复 `APP_BALL_CALIBRATION_MODE=0`，确认摄像头红线初始标定值。
   默认是 `APP_BALL_DEFAULT_TARGET_X_Q4=2736`。触点必须先落在红线
   左右30个LCD像素内才能捕获拖动。
5. 放入钢球并短按启动。若 `x<171` 时钢球反而继续向左运动，翻转
   `APP_BALL_POSITION_TO_TILT_SIGN`；不要用 `DIR_INVERT` 修正控制符号。
6. 分别测量正、负方向刚好能克服静摩擦的倾角，填写
   `APP_BALL_BREAKAWAY_STEPS_POS/NEG`。两边允许不同。
7. 先只调 `APP_BALL_POSITION_KP`，使球能回到红线附近；再逐步增加
   `APP_BALL_POSITION_KD` 抑制越线和振荡。
8. 最后验证红线附近稳态误差绝对值不超过 1 cm，并反复测试视觉丢失
   与恢复。视觉超过 150 ms 没有真实帧时，程序应停 STEP、保持 EN
   和当前位置；连续 3 个真实帧后才无冲击恢复。

## 主要调参量

- `APP_BALL_POSITION_KP/KD`：位置 PD。
- `APP_BALL_HOLD_ENTER_ERROR_Q4`、`APP_BALL_HOLD_RELEASE_ERROR_Q4`：
  保持区及其回差。
- `APP_BALL_BREAKAWAY_STEPS_POS/NEG`：两方向最小有效倾角。
- `APP_BALL_FRICTION_INCREMENT_STEPS`、`APP_BALL_FRICTION_MAX_STEPS`：
  卡滞时逐级增加的脱困补偿。
- `APP_BALL_WORK_TILT_LIMIT_STEPS`：正常闭环倾角限幅，默认 64 步，
  小于执行器的绝对软限位。

K230 屏幕显示 `ACK` 表示3507已经应用目标，`WAIT`表示正在等待，
`LINK LOST`表示超过300 ms没有合法状态回传。OLED 在 ACTIVE/HOLD
状态显示当前钢球 X 坐标，故障显示 `Err`。UART
状态帧还包含目标 X、Q4 误差、滤波速度、连续倾角命令、静摩擦补偿、
当前/目标步数、STEP 频率、视觉年龄、软限位和故障原因，适合记录后
离线调参。

本版本已通过主机确定性测试及 CCS TI Clang 编译、链接和 HEX 生成；
未烧录，也未对实物“红线咬合”效果作结论。
