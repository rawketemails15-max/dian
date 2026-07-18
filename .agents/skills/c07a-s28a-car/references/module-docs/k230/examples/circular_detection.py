import time
from media.sensor import *    # 导入摄像头模块
from media.display import *   # 导入显示模块
from media.media import *     # 导入媒体资源管理模块

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
    lcd_width, lcd_height = init_display(select_display)

    sensor = Sensor()  # 构建摄像头对象
    sensor.reset()     # 复位摄像头，必须先调用
    sensor.set_framesize(width=320, height=240)  # 设置采集分辨率
    sensor.set_pixformat(Sensor.RGB565)          # 设置像素格式为RGB565

    MediaManager.init()   # 初始化media资源
    sensor.run()          # 启动采集

    clock = time.clock()

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()  # 拍摄一帧

            # 圆形参数解释：
            # c.x(), c.y(): 圆心坐标
            # c.r(): 半径
            # magnitude（量级）：数字越大，检测越可靠
            # threshold 控制检测数量，越大检测越严格
            # x_margin, y_margin, r_margin 控制合并检测圆的参数

            for c in img.find_circles(threshold=2000, x_margin=10, y_margin=10,
                                      r_margin=10, r_min=2, r_max=100, r_step=2):
                img.draw_circle(c.x(), c.y(), c.r(), color=(255, 0, 0), thickness=2)  # 画红圈指示
                print(c)  # 打印检测圆信息

            # 居中显示图像
            x_offset = round((lcd_width - sensor.width()) / 2)
            y_offset = round((lcd_height - sensor.height()) / 2)
            Display.show_image(img, x=x_offset, y=y_offset)

            print(f"FPS: {clock.fps():.2f}")

    except KeyboardInterrupt:
        print("用户中断程序")

    finally:
        deinit_display()
