import time
import math
import os
import gc
import sys

from media.sensor import *
from media.display import *
from media.media import *

# ==================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 2  # 1=HDMI，2=LCD，3=IDE虚拟显示

def init_display(select_display):
    """
    初始化显示设备并返回其分辨率。
    根据 select_display 的值选择 HDMI、LCD 或 IDE 虚拟显示。
    """
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
    """
    释放显示资源。
    """
    Display.deinit()
    print("释放显示资源")

def main():
    sensor = None # 初始化 sensor 变量

    try:
        # 1. 初始化显示设备并获取其分辨率
        display_width, display_height = init_display(select_display)
        print(f"显示模式: 已初始化为 {display_width}x{display_height}")

        # 2. 初始化摄像头 (Sensor)，使用显示设备的分辨率
        sensor = Sensor(width=display_width, height=display_height)
        sensor.reset()
        sensor.set_framesize(width=display_width, height=display_height)
        # sensor.set_pixformat(Sensor.GRAYSCALE) # Data Matrix 在灰度图上显示，根据需要取消注释
        sensor.set_pixformat(Sensor.RGB565)

        # 3. 初始化媒体管理器并启动摄像头
        MediaManager.init()
        sensor.run()
        print("初始化完成，开始检测 Data Matrix 码...")

        fps_clock = time.clock() # 使用不同的变量名以避免与 fps.fps() 混淆

        # 4. 主循环：捕获图像并检测
        while True:
            fps_clock.tick()
            os.exitpoint() # 检查是否应该退出

            # 从摄像头捕获一帧图像
            img = sensor.snapshot()

            # 在图像中查找 Data Matrix 码
            for matrix in img.find_datamatrices():
                # 用红色矩形框出识别到的码
                img.draw_rectangle([v for v in matrix.rect()], color=(255, 0, 0))
                # 准备并打印识别结果
                print_args = (matrix.rows(), matrix.columns(), matrix.payload(), (180 * matrix.rotation()) / math.pi, fps_clock.fps())
                print("矩阵 [%d:%d], 内容 \"%s\", 旋转 %.1f (度), FPS %.2f" % print_args)

            # 将带有检测结果的图像显示出来
            Display.show_image(img)
            gc.collect()

    except KeyboardInterrupt:
        print("用户停止运行。")
    except BaseException as e:
        print(f"程序异常退出: '{e}'")
    finally:
        # 5. 清理和释放资源
        print("正在释放资源...")
        if isinstance(sensor, Sensor):
            sensor.stop()
        deinit_display() # 调用 deinit_display 函数
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        print("程序已完全退出。")

if __name__ == "__main__":
    main()
