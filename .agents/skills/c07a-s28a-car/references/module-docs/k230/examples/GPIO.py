from machine import Pin

# 将引脚 2 配置为输出模式，无上下拉，驱动能力为 7
pin = Pin(2, Pin.OUT, pull=Pin.PULL_NONE, drive=7)

# 设置引脚 2 输出高电平
pin.value(1)

# 设置引脚 2 输出低电平
pin.value(0)

