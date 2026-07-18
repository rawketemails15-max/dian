import time, os, urandom, sys
from media.display import *
from media.media import *
import image

# ==================================================================
# == 请在这里选择显示模式 (1: HDMI, 2: LCD, 3: IDE 缓冲区) ==
# ==================================================================
select_display = 3
# ==================================================================

def ALIGN_UP(x, align):
    return (x + (align - 1)) & ~(align - 1)

def display_test():
    print(f"开始显示测试...")

    # --- 步骤 1: 根据选择初始化显示设备和分辨率 ---
    if select_display == 1:
        # 模式 1: HDMI 输出
        print("当前模式: HDMI (1920x1080)")
        DISPLAY_WIDTH = ALIGN_UP(1920, 16)
        DISPLAY_HEIGHT = 1080
        # 初始化 LT9611 芯片以驱动 HDMI
        Display.init(Display.LT9611, to_ide = True)
    elif select_display == 2:
        # 模式 2: LCD 屏幕输出
        print("当前模式: LCD (800x480)")
        DISPLAY_WIDTH = ALIGN_UP(800, 16)
        DISPLAY_HEIGHT = 480
        # 初始化 ST7701 芯片以驱动 LCD
        Display.init(Display.ST7701, width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT, to_ide = True)
    elif select_display == 3:
        # 模式 3: IDE 虚拟缓冲区输出
        print("当前模式: IDE 缓冲区 (640x480)")
        DISPLAY_WIDTH = ALIGN_UP(640, 16)
        DISPLAY_HEIGHT = 480
        # 初始化虚拟显示设备，用于在 IDE 中预览
        Display.init(Display.VIRT, width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT, fps = 60, to_ide=True)

    else:
        print(f"错误：无效的显示模式选择 ({select_display})。请选择 1, 2, 或 3。")
        return

    # --- 步骤 2: 初始化媒体管理器并创建图像画布 ---
    MediaManager.init()
    # 创建一个用于绘制的图像对象，使用 ARGB8888 格式以支持颜色和透明度
    img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

    try:
        # --- 步骤 3: 主循环，用于绘制和显示 ---
        while True:
            # 清空画布，准备重新绘制
            img.clear()

            # 在画布的随机位置绘制10个彩色的字符串
            for i in range(10):
                x = (urandom.getrandbits(11) % img.width())
                y = (urandom.getrandbits(11) % img.height())
                r = (urandom.getrandbits(8))
                g = (urandom.getrandbits(8))
                b = (urandom.getrandbits(8))
                size = (urandom.getrandbits(30) % 64) + 32

                # 调用高级绘图函数来绘制中英文字符串
                img.draw_string_advanced(x, y, size, "Helle Hiwonder", color=(r, g, b))

            # 将绘制好的图像内容显示到屏幕上
            Display.show_image(img)
            time.sleep(1)
            os.exitpoint()

    except KeyboardInterrupt as e:
        print("用户手动停止程序: ", e)
    except BaseException as e:
        print(f"程序出现异常: {e}")
    finally:
        # --- 步骤 4: 清理和释放资源 ---
        print("正在关闭显示并释放资源...")
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        print("清理完成。")


if __name__ == "__main__":
    # 启用退出点功能，这对于在循环中安全地中断程序很重要
    os.exitpoint(os.EXITPOINT_ENABLE)
    display_test()
