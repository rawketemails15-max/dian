import time
import math
import os
import gc
import sys

from media.sensor import *
from media.display import *
from media.media import *
import image

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
        width, height = 640, 480
        Display.init(Display.VIRT, width=width, height=height, fps=100, to_ide=True)
    else:
        raise ValueError("select_display参数错误，只能是1、2或3")
    return width, height

def deinit_display():
    """
    释放显示资源。
    """
    Display.deinit()
    print("释放显示资源")

# 定义可用的标签族
tag_families = 0
tag_families |= image.TAG16H5  # 4x4 方形标签
tag_families |= image.TAG25H7  # 5x7 方形标签
tag_families |= image.TAG25H9  # 5x9 方形标签
tag_families |= image.TAG36H10 # 6x10 方形标签
tag_families |= image.TAG36H11 # 6x11 方形标签（默认）
tag_families |= image.ARTOOLKIT # ARToolKit 标签

# 函数: 获取标签族的名称
def family_name(tag):
    family_dict = {
        image.TAG16H5: "TAG16H5",
        image.TAG25H7: "TAG25H7",
        image.TAG25H9: "TAG25H9",
        image.TAG36H10: "TAG36H10",
        image.TAG36H11: "TAG36H11",
        image.ARTOOLKIT: "ARTOOLKIT",
    }
    return family_dict.get(tag.family(), "未知标签族")

def main():
    sensor = None # 初始化 sensor 变量

    try:
        # 1. 初始化显示设备并获取其分辨率
        # 使用 init_display 函数来获取显示分辨率，作为传感器分辨率的依据
        display_width, display_height = init_display(select_display)

        # 2. 初始化摄像头 (Sensor)，使用显示设备的分辨率
        sensor = Sensor(width=display_width, height=display_height)
        sensor.reset()
        sensor.set_framesize(width=display_width, height=display_height)
        sensor.set_pixformat(Sensor.RGB565)

        # 3. 初始化媒体管理器并启动摄像头
        MediaManager.init()
        sensor.run()

        fps_clock = time.clock() # 使用不同的变量名以避免与 fps.fps() 混淆

        # 4. 主循环：捕获图像并检测
        while True:
            fps_clock.tick()
            os.exitpoint() # 检查是否应该退出

            img = sensor.snapshot()
            for tag in img.find_apriltags(families=tag_families):
                img.draw_rectangle([v for v in tag.rect()], color=(255,0,0))
                img.draw_cross(tag.cx(), tag.cy(), color=(0,255,0))
                print_args = (family_name(tag), tag.id(), (180 * tag.rotation()) / math.pi)
                print("标签族 %s, 标签 ID %d, 旋转 %f (度)" % print_args)

            Display.show_image(img)
            gc.collect()

    except KeyboardInterrupt:
        print("用户停止")
    except BaseException as e:
        print(f"异常 '{e}'")
    finally:
        # 5. 清理和释放资源
        if isinstance(sensor, Sensor):
            sensor.stop()
        deinit_display() # 调用 deinit_display 函数
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        print("程序已完全退出。")

if __name__ == "__main__":
    main()
