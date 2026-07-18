import time, gc
from media.sensor import *
from media.display import *
from media.media import *

select_display = 2

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
    # 注意：摄像头采集分辨率通常不要超过显示，否则影响性能
    sensor.set_framesize(width=320, height=240)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    MediaManager.init()
    sensor.run()

    clock = time.clock()

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()

            # 使用Canny边缘检测
            img.find_edges(image.EDGE_CANNY, threshold=(50, 80))

            # 居中显示
            x_offset = round((width - sensor.width()) / 2)
            y_offset = round((height - sensor.height()) / 2)

            Display.show_image(img, x=x_offset, y=y_offset)

            print(f"FPS: {clock.fps():.2f}")

    except KeyboardInterrupt:
        print("用户中断程序")

    finally:
        deinit_display()
