from machine import Pin, PWM
from machine import FPIOA
import time

fpioa = FPIOA()
fpioa.set_function(43, FPIOA.PWM1)  #

Beep = PWM(1)      # 创建PWM通道1对象
Beep.freq(200)     # 设定200Hz频率
Beep.duty(50)      # 50%占空比，蜂鸣器响

time.sleep(1)

Beep.freq(400)
time.sleep(1)

Beep.freq(600)
time.sleep(1)

Beep.freq(800)
time.sleep(1)

Beep.freq(1000)
time.sleep(1)

Beep.duty(0)
