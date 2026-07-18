from machine import Pin
from machine import FPIOA
import time

fpioa = FPIOA()
fpioa.set_function(52,FPIOA.GPIO52)

LED=Pin(52,Pin.OUT) #将引脚52设置为输出模式，控制LED灯
LED.value(1)# 设置引脚52输出高电平，点亮LED灯


while True:
    pass
