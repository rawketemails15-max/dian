from machine import TOUCH
import time
from media.display import Display
from media.media import MediaManager
import image

# 初始化显示器(示例按LCD模式)
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480

Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=False)
MediaManager.init()

# 画布
img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB565)
red = (255, 0, 0)

# 实例化TOUCH设备0
tp = TOUCH(0)

try:
    while True:
        p = tp.read()  # 读取所有触摸点

        img.clear()  # 清空画布背景默认黑色，可按需修改

        if p != ():
            for i, point in enumerate(p):
                text = f'x{i}={point.x} y{i}={point.y}'
                print('x'+str(i)+'=',p[i].x, 'y'+str(i)+'=',p[i].y)

                # 在屏幕左上方依次向下显示
                img.draw_string_advanced(10, 30 + i*40, 30, text, color=red, scale=3)

        Display.show_image(img)
        time.sleep_ms(50)

except KeyboardInterrupt:
    pass
finally:
    Display.deinit()
    MediaManager.deinit()
