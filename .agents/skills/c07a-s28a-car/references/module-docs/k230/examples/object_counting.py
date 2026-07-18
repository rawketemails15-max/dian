import time
from media.sensor import *
from media.display import *
from media.media import *

# 颜色阈值，黄色跳线帽阈值
thresholds = [(18, 72, -13, 31, 18, 83)]

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

if __name__ == "__main__":
    # 初始化显示并获取宽高
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
                    img.draw_rectangle(b[0:4])
                    img.draw_cross(b[5], b[6])

            # 显示FPS和计数
            img.draw_string_advanced(0, 0, 30, f'FPS: {clock.fps():.3f}    Num: {len(blobs)}', color=(255, 255, 255))

            Display.show_image(img)

            print(clock.fps())

    except KeyboardInterrupt:
        print("用户中断程序")

    finally:
        deinit_display()
