import time
import os
import gc

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
import image

# ===================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ===================================================================
select_display = 3  # 1=HDMI，2=LCD，3=IDE虚拟显示

def init_display(select_display, width, height):
    if select_display == 1:
        # HDMI显示，注意根据你显示器能力调整分辨率和fps
        Display.init(Display.LT9611, width=width, height=height, fps=60, to_ide=True)
        print(f"初始化HDMI显示，分辨率：{width}x{height} @60Hz")
    elif select_display == 2:
        # LCD显示，ST7701驱动屏幕，固定800x480
        Display.init(Display.ST7701, to_ide=True)
        print("初始化LCD显示，默认分辨率800x480")
    elif select_display == 3:
        # IDE虚拟显示，分辨率及fps可任意设置
        Display.init(Display.VIRT, width=width, height=height, fps=100, to_ide=True)
        print(f"初始化IDE虚拟显示，分辨率：{width}x{height} fps=100")
    else:
        raise ValueError("select_display参数错误，只能是1、2或3")

def deinit_display():
    Display.deinit()
    print("释放显示资源")

# 根据显示模式设置摄像头分辨率
if select_display == 2:
    DETECT_WIDTH = 800
    DETECT_HEIGHT = 480
elif select_display == 1:
    DETECT_WIDTH = 640
    DETECT_HEIGHT = 480
else:
    DETECT_WIDTH = 640
    DETECT_HEIGHT = 480

sensor = None

try:
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    init_display(select_display, DETECT_WIDTH, DETECT_HEIGHT)

    MediaManager.init()
    sensor.run()

    fps = time.clock()

    while True:
        fps.tick()
        img = sensor.snapshot()

        # 二维码检测
        for code in img.find_qrcodes():
            rect = code.rect()
            img.draw_rectangle(rect, color=(255, 0, 0), thickness=5)
            img.draw_string_advanced(rect[0], rect[1], 32, code.payload())
            print(f"找到二维码内容: {code.payload()}")

        Display.show_image(img)

        gc.collect()

except KeyboardInterrupt:
    print("用户停止")
except Exception as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    deinit_display()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
