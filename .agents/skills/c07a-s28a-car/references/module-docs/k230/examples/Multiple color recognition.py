import time
from media.sensor import *
from media.display import *
from media.media import *

# 颜色识别阈值(LAB模型)
thresholds = [
    (20, 32, 21, 37, 3, 14),   # 红色阈值
    (28, 42, -34, -15, -7, 11),    # 绿色阈值
    (36, 46, -4, 1, -31, -23)      # 蓝色阈值
]

colors1 = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
colors2 = ['RED', 'GREEN', 'BLUE']

select_display = 1  # 1=HDMI，2=LCD，3=IDE虚拟显示

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

if __name__ == "__main__":
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

            for i in range(3):
                blobs = img.find_blobs([thresholds[i]])
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4], thickness=4, color=colors1[i])
                        img.draw_cross(b[5], b[6], thickness=2)
                        img.draw_string_advanced(b[0], b[1]-35, 30, colors2[i], color=colors1[i])

            img.draw_string_advanced(0, 0, 30, 'FPS: {:.3f}'.format(clock.fps()), color=(255, 255, 255))
            Display.show_image(img)
            print(clock.fps())

    except KeyboardInterrupt:
        print("用户中断程序")

    finally:
        deinit_display()
