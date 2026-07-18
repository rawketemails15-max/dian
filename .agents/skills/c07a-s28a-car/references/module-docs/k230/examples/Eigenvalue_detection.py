import time, os, gc
from media.sensor import *
from media.display import *
from media.media import *

# ==================================================================
# == 请在这里选择显示模式 (1: HDMI, 2: LCD, 3: IDE 缓冲区) ==
# ==================================================================
select_display = 2

DETECT_WIDTH = ALIGN_UP(640, 16)
DETECT_HEIGHT = 480

min_degree = 0
max_degree = 179

sensor = None

def init_display(select):
    """
    根据选择初始化显示设备
    1 - HDMI 640x480
    2 - LCD
    3 - IDE虚拟显示 (640x480)
    """
    if select == 1:
        Display.init(Display.LT9611, width=640, height=480, to_ide=True)
        print("显示初始化为HDMI 640x480")
    elif select == 2:
        Display.init(Display.ST7701, to_ide=True)
        print("显示初始化为LCD屏幕")
    elif select == 3:
        Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=100, to_ide=True)
        print("显示初始化为IDE虚拟显示")
    else:
        raise ValueError("select_display 参数错误，只能是1、2或3")

def camera_init():
    global sensor
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    # sensor.set_hmirror(False)
    # sensor.set_vflip(False)
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    init_display(select_display)
    MediaManager.init()
    sensor.run()

def camera_deinit():
    global sensor
    sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()

def capture_picture():
    global sensor
    fps = time.clock()
    while True:
        fps.tick()
        try:
            os.exitpoint()
            img = sensor.snapshot()

            lines = img.find_lines(threshold=1000, theta_margin=25, rho_margin=25)
            for l in lines:
                if min_degree <= l.theta() <= max_degree:
                    img.draw_line(l.line(), color=(255, 0, 0),thickness=5)
                    print(l)

            Display.show_image(img)
            del img  # 显式释放引用

            gc.collect()
            print("FPS:", fps.fps())

        except KeyboardInterrupt:
            print("用户中断，退出。")
            break
        except Exception as e:
            print(f"异常退出: {e}")
            break

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    try:
        print("初始化摄像头和显示...")
        camera_init()
        camera_is_init = True
        print("开始检测直线...")
        capture_picture()
    except Exception as e:
        print(f"发生异常: {e}")
    finally:
        if camera_is_init:
            print("释放摄像头及相关资源...")
            camera_deinit()

if __name__ == "__main__":
    main()
