import time, math
from media.sensor import *
from media.display import *
from media.media import *

# 3.5寸屏幕分辨率
lcd_width = 800
lcd_height = 480

# 彩色黑线阈值 (LAB)
BLACK_THRESHOLD = [(0, 24, -128, 4, -128, 7)]

# 原 ROI (针对320x240)
ORIGINAL_ROIS = [
    (0, 200, 320, 40, 0.7),
    (0, 100, 320, 40, 0.3),
    (0, 0, 320, 40, 0.1)
]

# 放大 ROI 以适应 800x480
ROIS = []
for r in ORIGINAL_ROIS:
    x = int(r[0] * 800 / 320)
    y = int(r[1] * 480 / 240)
    w = int(r[2] * 800 / 320)
    h = int(r[3] * 480 / 240)
    ROIS.append((x, y, w, h, r[4]))

weight_sum = sum([r[4] for r in ROIS])

# 初始化显示
Display.init(Display.ST7701, width=lcd_width, height=lcd_height, to_ide=True)

sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=800, height=480)
sensor.set_pixformat(Sensor.RGB565)  # 彩色

MediaManager.init()
sensor.run()

clock = time.clock()
fps_list = []  # 用于平滑FPS

while True:
    clock.tick()
    img = sensor.snapshot()

    centroid_sum = 0
    for r in ROIS:
        blobs = img.find_blobs(BLACK_THRESHOLD, roi=r[0:4], merge=True)
        if blobs:
            # 取最大面积的 blob
            largest_blob = max(blobs, key=lambda b: b.pixels())
            # 绘制矩形和中心点
            img.draw_rectangle(largest_blob[0:4])
            img.draw_cross(largest_blob[5], largest_blob[6])
            centroid_sum += largest_blob.cx() * r[4]

    # 计算平滑FPS
    fps = max(clock.fps(), 0)
    fps_list.append(fps)
    if len(fps_list) > 10:
        fps_list.pop(0)
    avg_fps = sum(fps_list) / len(fps_list)

    # FPS显示在右上角
    fps_text = 'FPS: %.2f' % avg_fps
    text_width = len(fps_text) * 12  # 估算文字宽度，12为字体大小近似
    img.draw_string_advanced(lcd_width - text_width - 2, 2, 20, fps_text, color=(255, 255, 255))

    # 居中显示图像
    x_offset = round((lcd_width - sensor.width()) / 2)
    y_offset = round((lcd_height - sensor.height()) / 2)
    Display.show_image(img, x=x_offset, y=y_offset)

    print("FPS: %.2f" % avg_fps)
