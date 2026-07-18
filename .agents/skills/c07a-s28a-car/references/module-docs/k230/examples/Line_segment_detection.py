import time
from media.sensor import *  # 导入sensor模块，使用摄像头相关接口
from media.display import *  # 导入display模块，使用display相关接口
from media.media import *  # 导入media模块，使用media相关接口

select_display = 3  # 1=HDMI，2=LCD，3=IDE虚拟显示

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

def main():
    width, height = init_display(select_display)

    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=320, height=240)
    sensor.set_pixformat(Sensor.RGB565)

    MediaManager.init()
    sensor.run()

    enable_lens_corr = False

    clock = time.clock()

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()

            if enable_lens_corr:
                img.lens_corr(1.8)  # for 2.8mm lens...

            for l in img.find_line_segments(merge_distance=0, max_theta_diff=5):
                img.draw_line(l.line(), color=(255, 0, 0), thickness=2)
                print(l)

            x_offset = round((width - sensor.width()) / 2)
            y_offset = round((height - sensor.height()) / 2)
            Display.show_image(img, x=x_offset, y=y_offset)

            print(f"FPS: {clock.fps():.2f}")

    except KeyboardInterrupt:
        print("用户中断程序")

    finally:
        deinit_display()

if __name__ == "__main__":
    main()
