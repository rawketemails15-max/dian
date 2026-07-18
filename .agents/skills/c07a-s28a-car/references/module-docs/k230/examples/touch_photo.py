import uos
import sys
import time
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
from machine import TOUCH

sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=800, height=480)
sensor.set_pixformat(Sensor.RGB565)

Display.init(Display.ST7701, to_ide=False)
MediaManager.init()
sensor.run()

# 初始化触摸对象并读取触摸事件
tp = TOUCH(0)

# 保存照片的目录
save_dir = "/sdcard"

def ensure_dir_exists(path):
    try:
        uos.listdir(path)
    except OSError:
        print("目录 {} 不存在，正在创建...".format(path))
        uos.mkdir(path)
        print("目录 {} 创建完成".format(path))

ensure_dir_exists(save_dir)

while True:
    img = sensor.snapshot()
    img.draw_circle(720, 240, 25, color=(255, 255, 255), thickness=1, fill=True)
    img.draw_circle(720, 240, 40, color=(200, 200, 200), thickness=10)

    p = tp.read(1)
    if p != ():
        print(p)
        x, y, event = p[0].x, p[0].y, p[0].event

        if event == 2 or event == 3:
            img.draw_cross(x, y, color=(255, 0, 0), size=10, thickness=6)

        if 720 - 40 < x < 720 + 40 and 240 - 40 < y < 240 + 40 and event == 2:
            timestamp = time.ticks_ms()
            filename = "{}/photo_{}.jpg".format(save_dir, timestamp)


            img = sensor.snapshot()
            img.save(filename)
            print("照片已保存:", filename)
            time.sleep(1)

    # 显示实时图像
    Display.show_image(img)
