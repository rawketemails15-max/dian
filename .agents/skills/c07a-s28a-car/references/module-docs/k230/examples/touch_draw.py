import time, os, sys, urandom
from media.display import *
from media.media import *
from machine import TOUCH
from collections import namedtuple

# 屏幕大小
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480

# 绘图区域大小（画布）
CANVAS_W = 800
CANVAS_H = 400
CANVAS_X = 0
CANVAS_Y = 120   # 从屏幕 y=80 开始

Point = namedtuple("Point", ["x", "y"])

try:
    # 初始化显示器
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=False)

    # 初始化媒体管理器
    MediaManager.init()

    # --- 两张图像 ---
    ui_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB565)   # 全屏UI层
    canvas_img = image.Image(CANVAS_W, CANVAS_H, image.RGB565)          # 绘画层
    background_color = (255, 255, 255)
    canvas_img.clear()
    canvas_img.draw_rectangle(0, 0, CANVAS_W, CANVAS_H, color=background_color, fill=True)

    # 颜色与画笔
    base_color_1 = (0, 255, 0)  # 绿色
    base_color_2 = (255, 0, 0)  # 红色
    current_color = base_color_1 # 初始颜色
    brush_size = 10

    # UI 按钮与滑杆区域
    clear_button_area = (DISPLAY_WIDTH - 130, 0, 130, 50)
    color_button_area = (0, 0, 130, 50)
    save_button_area = (0, 60, 130, 50)
    eraser_button_area = (DISPLAY_WIDTH // 2 - 80, 0, 220, 50)
    slider_area = (DISPLAY_WIDTH // 2 - 150, 70, 300, 40)

    forbidden_draw_areas = [
        clear_button_area,
        color_button_area,
        save_button_area,
        eraser_button_area,
        slider_area
    ]

    tp = TOUCH(0)
    last_point = None
    eraser_mode = False
    threshold_value = 128

    # --- UI 绘制函数 ---
    def draw_button(x, y, w, h, text, bg_color, text_color):
        ui_img.draw_rectangle(x, y, w, h, color=bg_color, fill=True)
        ui_img.draw_string_advanced(x + (w - len(text) * 15) // 2, y + 12, 30,
                                    text, color=text_color, scale=2)

    def draw_clear_button():
        draw_button(*clear_button_area, "清除", (255, 0, 0), (255, 255, 255))

    def draw_color_buttons():
        draw_button(*color_button_area, "随机", (255, 255, 0), (0, 0, 0))
        ui_img.draw_circle(color_button_area[0] + 170, 25, 20,
                           color=current_color, thickness=3, fill=True)

    def draw_save_button():
        draw_button(*save_button_area, "保存", (0, 128, 255), (255, 255, 255))

    def draw_eraser_button():
        bg = (128, 128, 128) if eraser_mode else (0, 200, 200)
        text = "橡皮擦 ON" if eraser_mode else "橡皮擦 OFF"
        draw_button(*eraser_button_area, text, bg, (255, 255, 255))

    def apply_threshold_color(base1, base2, threshold):
        r = base1[0] + (base2[0] - base1[0]) * threshold / 255
        g = base1[1] + (base2[1] - base1[1]) * threshold / 255
        b = base1[2] + (base2[2] - base1[2]) * threshold / 255
        return (int(r), int(g), int(b))

    def draw_slider():
        x, y, w, h = slider_area
        ui_img.draw_rectangle(x, y + h // 3, w, h // 3,
                              color=(200, 200, 200), fill=True)
        knob_x = x + int((threshold_value / 255) * w)
        knob_y = y + h // 2
        knob_color = apply_threshold_color(base_color_1, base_color_2, threshold_value)
        ui_img.draw_circle(knob_x, knob_y, 12, color=knob_color, fill=True)

        txt = f'阈值: {threshold_value}'
        ui_img.draw_string_advanced(x + w + 10, y + 8, 30,
                                    txt, color=(0, 0, 0), scale=2)

    # --- UI 交互 ---
    def is_in_area(x, y, area):
        ax, ay, aw, ah = area
        return ax <= x <= ax + aw and ay <= y <= ay + ah

    def select_color(x, y):
        global current_color, eraser_mode
        if is_in_area(x, y, color_button_area):
            eraser_mode = False
            current_color = (urandom.getrandbits(8),
                             urandom.getrandbits(8),
                             urandom.getrandbits(8))
            print(f"随机颜色 {current_color}")

    def check_clear_button(x, y):
        if is_in_area(x, y, clear_button_area):
            canvas_img.draw_rectangle(0, 0, CANVAS_W, CANVAS_H,
                                      color=background_color, fill=True)
            print("画布已清空为白色背景")

    def check_save_button(x, y):
        if is_in_area(x, y, save_button_area):
            filename = '/sdcard/1.jpg'
            canvas_img.save(filename)
            print(f"图像保存至 {filename}")

    def check_eraser_button(x, y):
        global eraser_mode
        if is_in_area(x, y, eraser_button_area):
            eraser_mode = not eraser_mode
            print(f"橡皮擦模式 {'开启' if eraser_mode else '关闭'}")

    def check_slider_touch(x, y):
        global threshold_value, current_color
        if is_in_area(x, y, slider_area):
            sx, sy, sw, sh = slider_area
            relative_x = min(max(0, x - sx), sw)
            new_threshold = int((relative_x / sw) * 255)
            if new_threshold != threshold_value:
                threshold_value = new_threshold
                if not eraser_mode:
                    current_color = apply_threshold_color(base_color_1, base_color_2, threshold_value)

                print(f"阈值调整为 {threshold_value}")

    # --- 画布绘制 ---
    def draw_line_between_points(last_point, current_point):
        global current_color
        if eraser_mode:
            draw_color = background_color
        else:
            draw_color = current_color

        if last_point is not None:
            dx = current_point.x - last_point.x
            dy = current_point.y - last_point.y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist > 30:
                return
            min_dist = 10
            if dist > min_dist:
                steps = int(dist // min_dist)
                for i in range(1, steps + 1):
                    nx = last_point.x + i * dx / (steps + 1)
                    ny = last_point.y + i * dy / (steps + 1)
                    if 0 <= nx < CANVAS_W and 0 <= ny < CANVAS_H:
                        canvas_img.draw_circle(int(nx), int(ny), brush_size,
                                               color=draw_color, thickness=3, fill=True)

        if 0 <= current_point.x < CANVAS_W and 0 <= current_point.y < CANVAS_H:
            canvas_img.draw_circle(current_point.x, current_point.y,
                                   brush_size, color=draw_color, thickness=3, fill=True)

    # --- 主循环 ---
    while True:
        os.exitpoint()
        p = tp.read(1)

        if p != ():
            for point in p:
                x, y = point.x, point.y

                # UI 交互
                select_color(x, y)
                check_clear_button(x, y)
                check_save_button(x, y)
                check_eraser_button(x, y)
                check_slider_touch(x, y)

                # 画布区域 (触摸点减去偏移量 CANVAS_Y)
                if y >= CANVAS_Y:
                    cp = Point(x, y - CANVAS_Y)
                    draw_line_between_points(last_point, cp)
                    last_point = cp
                else:
                    last_point = None
        else:
            last_point = None

        # 每次刷新 UI 层
        ui_img.clear()
        ui_img.draw_rectangle(0, 0, DISPLAY_WIDTH, CANVAS_Y,
                              color=(230, 230, 230), fill=True)  # UI背景
        draw_clear_button()
        draw_color_buttons()
        draw_save_button()
        draw_eraser_button()
        draw_slider()

        # 把 canvas 画到 ui_img 的 (0,80) 区域
        ui_img.draw_image(canvas_img, CANVAS_X, CANVAS_Y)

        # 显示最终合成图
        Display.show_image(ui_img)
        time.sleep_ms(10)

except KeyboardInterrupt:
    print("用户中断程序")
except BaseException as e:
    print(f"异常: {e}")
finally:
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
