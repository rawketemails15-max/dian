import time
from media.sensor import *
from media.display import *
from media.media import *
from media.display import Display

thresholds = [
    (0, 100, 29, 127, 3, 127),  # 红色阈值
    (30, 100, -64, -8, 50, 70),   # 绿色阈值
    (0, 100, -128, -18, 1, 127)     # 蓝色阈值
]
# ==================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 2  # 1=HDMI，2=LCD，3=IDE虚拟显示

def init_display(select_display):
    if select_display == 1:
        width, height = 640, 480
        Display.init(Display.LT9611, width=width, height=height, to_ide=True)
        print(f"初始化HDMI显示，分辨率：{width}x{height}")
    elif select_display == 2:
        width, height = 800, 480
        Display.init(Display.ST7701, width=width, height=height, to_ide=True)
        print(f"初始化LCD显示，分辨率：{width}x{height}")
    elif select_display == 3:
        width, height = 1920, 1080
        Display.init(Display.VIRT, width=width, height=height, fps=100, to_ide=True)
        print(f"初始化IDE虚拟显示，分辨率：{width}x{height}")
    else:
        raise ValueError("select_display参数错误，只能是1、2或3")
    return width, height


def deinit_display():
    Display.deinit()
    print("释放显示资源")

# 主程序
width, height = init_display(select_display)

sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=width, height=height)
sensor.set_pixformat(Sensor.RGB565)

MediaManager.init()
sensor.run()

clock = time.clock()

try:
    while True:
        clock.tick()
        img = sensor.snapshot()

        blobs = img.find_blobs([thresholds[0]])

        if blobs:
            for b in blobs:
                img.draw_rectangle(b[0:4], thickness=4)
                img.draw_cross(b[5], b[6], thickness=2)

        img.draw_string_advanced(0, 0, 30, 'FPS: {:.3f}'.format(clock.fps()), color=(255, 255, 255))
        Display.show_image(img)

        print(clock.fps())
except KeyboardInterrupt:
    print("用户中断程序")
finally:
    deinit_display()
