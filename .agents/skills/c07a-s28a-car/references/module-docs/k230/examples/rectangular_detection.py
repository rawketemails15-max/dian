import time
from media.sensor import *   # 导入摄像头相关模块
from media.display import *  # 导入显示相关模块
from media.media import *    # 导入media资源管理模块

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

    sensor = Sensor()       # 构建摄像头对象
    sensor.reset()          # 复位摄像头，必须先调用
    sensor.set_framesize(width=320, height=240)  # 设置采集分辨率，推荐不超过320x240
    sensor.set_pixformat(Sensor.RGB565)          # 设置像素格式为RGB565

    MediaManager.init()     # 初始化媒体资源管理
    sensor.run()            # 启动摄像头采集

    clock = time.clock()

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()  # 拍摄一帧

            # 设置阈值过滤噪声，只检测亮度较强的矩形
            for r in img.find_rects(threshold=10000):
                # 画出检测到的矩形边框，红色，线宽2
                img.draw_rectangle(r.rect(), color=(255, 0, 0), thickness=2)

                # 在矩形四个角画绿圈
                for p in r.corners():
                    img.draw_circle(p[0], p[1], 5, color=(0, 255, 0))

                print(r)  # 打印矩形信息

            # 居中显示图像
            x_offset = round((lcd_width - sensor.width()) / 2)
            y_offset = round((lcd_height - sensor.height()) / 2)
            Display.show_image(img, x=x_offset, y=y_offset)

            print(f"FPS: {clock.fps():.2f}")  # 打印帧率

    except KeyboardInterrupt:
        print("用户中断程序")

    finally:
        deinit_display()
