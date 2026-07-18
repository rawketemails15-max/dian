from machine import Pin, Timer, FPIOA

# 初始化GPIO52为普通GPIO输出
fpioa = FPIOA()
fpioa.set_function(52, FPIOA.GPIO52)
LED = Pin(52, Pin.OUT)

led_state = False  # LED初始状态关闭

# 定时器回调，切换LED状态
def led_toggle(timer):
    global led_state
    led_state = not led_state
    LED.value(1 if led_state else 0)

# 创建软定时器，1Hz周期，循环调用led_toggle
tim = Timer(-1)
tim.init(freq=1, mode=Timer.PERIODIC, callback=led_toggle)

# 主线程不需要做事，保持空循环防止程序结束
while True:
    pass
