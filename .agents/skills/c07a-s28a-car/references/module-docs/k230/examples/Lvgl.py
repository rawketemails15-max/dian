from media.display import *
from media.media import *
import time, os, sys, gc
import lvgl as lv
import uos

# ==================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 2  # 1=HDMI，2=LCD，3=IDE虚拟显示

# 根据显示模式设置分辨率
display_mode_map = {
    1: {"type": Display.LT9611, "size": (1920, 1080), "name": "HDMI"},
    2: {"type": Display.ST7701, "size": (800, 480), "name": "LCD"},
    3: {"type": Display.VIRT, "size": (1280, 720), "name": "IDE虚拟显示"}
}

# 确认的资源路径
RESOURCE_PATH = "/sdcard/examples/15-LVGL/data/"

def verify_resources():
    """验证所有需要的资源文件是否存在"""
    required_files = {
        "fonts": [
            "font/montserrat-16.fnt",
            "font/lv_font_simsun_16_cjk.fnt"
        ],
        "images": [
            "img/animimg001.png",
            "img/animimg002.png",
            "img/animimg003.png"
        ]
    }

    missing_files = []

    # 检查字体文件
    for font in required_files["fonts"]:
        path = RESOURCE_PATH + font
        try:
            with open(path, 'rb') as f:
                pass
            print(f"✔ Found file: {path}")
        except:
            missing_files.append(path)
            print(f"✖ Missing file: {path}")

    # 检查图片文件
    for img in required_files["images"]:
        path = RESOURCE_PATH + img
        try:
            with open(path, 'rb') as f:
                pass
            print(f"✔ Found file: {path}")
        except:
            missing_files.append(path)
            print(f"✖ Missing file: {path}")

    return len(missing_files) == 0

def display_init():
    """Initialize display"""
    mode = display_mode_map.get(select_display, display_mode_map[1])
    width, height = mode["size"]

    # Initialize display
    Display.init(mode["type"], width=width, height=height, to_ide=True)
    print(f"Initializing {mode['name']} display, resolution: {width}x{height}")

    # Initialize media manager
    MediaManager.init()
    return width, height

def display_deinit():
    """Release display resources"""
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(50)
    Display.deinit()
    MediaManager.deinit()

def disp_drv_flush_cb(disp_drv, area, color):
    """LVGL display flush callback"""
    global disp_img1, disp_img2

    if disp_drv.flush_is_last():
        if disp_img1.virtaddr() == uctypes.addressof(color.__dereference__()):
            Display.show_image(disp_img1)
        else:
            Display.show_image(disp_img2)
    disp_drv.flush_ready()

def lvgl_init(display_width, display_height):
    """Initialize LVGL"""
    global disp_img1, disp_img2

    lv.init()
    disp_drv = lv.disp_create(display_width, display_height)
    disp_drv.set_flush_cb(disp_drv_flush_cb)
    disp_img1 = image.Image(display_width, display_height, image.ARGB8888)
    disp_img2 = image.Image(display_width, display_height, image.ARGB8888)
    disp_drv.set_draw_buffers(disp_img1.bytearray(), disp_img2.bytearray(),
                             disp_img1.size(), lv.DISP_RENDER_MODE.DIRECT)

def lvgl_deinit():
    """Release LVGL resources"""
    global disp_img1, disp_img2

    lv.deinit()
    del disp_img1
    del disp_img2

def load_resource(path, resource_type="image"):
    """Generic function to load resources"""
    try:
        with open(RESOURCE_PATH + path, 'rb') as f:
            data = f.read()

        if resource_type == "font":
            full_path = "A:" + RESOURCE_PATH + path
            font = lv.font_load(full_path)
            print(f"Successfully loaded font: {full_path}")
            return font
        else:  # image
            return lv.img_dsc_t({
                'data_size': len(data),
                'data': data
            })
    except Exception as e:
        print(f"Failed to load resource: {RESOURCE_PATH + path}, error: {e}")
        return None

def user_gui_init():
    """Initialize user interface"""
    print("\n=== Loading resources ===")
    if not verify_resources():
        print("Warning: Some resource files are missing, UI may be incomplete")

    # Load fonts
    font_montserrat = load_resource("font/montserrat-16.fnt", "font")
    font_simsun = load_resource("font/lv_font_simsun_16_cjk.fnt", "font")

    # Create text labels
    if font_montserrat:
        ltr_label = lv.label(lv.scr_act())
        ltr_label.set_text("LVGL demo running...\nSystem is working properly")
        ltr_label.set_style_text_font(font_montserrat, 0)
        ltr_label.set_width(300)
        ltr_label.align(lv.ALIGN.TOP_MID, 0, 20)

    if font_simsun:
        cz_label = lv.label(lv.scr_act())
        cz_label.set_style_text_font(font_simsun, 0)
        cz_label.set_text("hello hiwonder\nResource path: " + RESOURCE_PATH)
        cz_label.set_width(300)
        cz_label.align(lv.ALIGN.BOTTOM_MID, 0, -20)

    # Load animation images
    anim_imgs = []
    for i in range(1, 4):
        img_num = f"{i:03d}"
        img = load_resource(f"img/animimg{img_num}.png")
        if img:
            anim_imgs.append(img)
            print(f"Successfully loaded animation frame {i}/3")

    if len(anim_imgs) >= 3:
        anim_imgs.append(anim_imgs[0])  # Loop animation
        animimg0 = lv.animimg(lv.scr_act())
        animimg0.center()
        animimg0.set_size(200, 200)  # Set fixed size
        animimg0.set_src(anim_imgs, len(anim_imgs))
        animimg0.set_duration(2000)
        animimg0.set_repeat_count(lv.ANIM_REPEAT_INFINITE)
        animimg0.start()
        print("Animation created successfully")
    else:
        print("Incomplete animation images, cannot create animation")

        # Create fallback content
        placeholder = lv.label(lv.scr_act())
        placeholder.set_text("Animation resources missing")
        placeholder.align(lv.ALIGN.CENTER, 0, 0)

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        print("\n=== Application initialization ===")
        display_width, display_height = display_init()
        lvgl_init(display_width, display_height)

        print("=== Creating user interface ===")
        user_gui_init()

        print("\n=== Entering main loop ===")
        while True:
            time.sleep_ms(lv.task_handler())
    except Exception as e:
        print(f"\n!!! Runtime exception: {e}")
#        import traceback
#        traceback.print_exception(e)
    finally:
        print("\n=== Cleaning up resources ===")
        lvgl_deinit()
        display_deinit()
        gc.collect()
        print("=== Program end ===")

if __name__ == "__main__":
    main()
