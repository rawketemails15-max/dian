import time, math, os, gc

from media.sensor import *
from media.display import *
from media.media import *

# ==================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 1  # 1=HDMI，2=LCD，3=IDE虚拟显示

def init_display(select_display, width, height):
    if select_display == 1:
        Display.init(Display.LT9611, width=width, height=height, to_ide=True)
        print(f"初始化HDMI显示，分辨率：{width}x{height}")
    elif select_display == 2:
        Display.init(Display.ST7701, to_ide=True)
        print("初始化LCD显示，默认分辨率800x480")
    elif select_display == 3:
        Display.init(Display.VIRT, width=width, height=height, fps=100, to_ide=True)
        print(f"初始化IDE虚拟显示，分辨率：{width}x{height}")
    else:
        raise ValueError("select_display参数错误，只能是1、2或3")

def deinit_display():
    Display.deinit()
    print("释放显示资源")

def barcode_name(code):
    if(code.type() == image.EAN2):
        return "EAN2"
    if(code.type() == image.EAN5):
        return "EAN5"
    if(code.type() == image.EAN8):
        return "EAN8"
    if(code.type() == image.UPCE):
        return "UPCE"
    if(code.type() == image.ISBN10):
        return "ISBN10"
    if(code.type() == image.UPCA):
        return "UPCA"
    if(code.type() == image.EAN13):
        return "EAN13"
    if(code.type() == image.ISBN13):
        return "ISBN13"
    if(code.type() == image.I25):
        return "I25"
    if(code.type() == image.DATABAR):
        return "DATABAR"
    if(code.type() == image.DATABAR_EXP):
        return "DATABAR_EXP"
    if(code.type() == image.CODABAR):
        return "CODABAR"
    if(code.type() == image.CODE39):
        return "CODE39"
    if(code.type() == image.PDF417):
        return "PDF417"
    if(code.type() == image.CODE93):
        return "CODE93"
    if(code.type() == image.CODE128):
        return "CODE128"

# 构建摄像头对象
sensor = Sensor()
sensor.reset() # 复位和初始化摄像头

# 定义摄像头分辨率
CAM_WIDTH = 640
CAM_HEIGHT = 480

# 设置帧大小为摄像头分辨率，默认通道0
sensor.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT)
# 设置输出图像格式为RGB565，默认通道0
sensor.set_pixformat(Sensor.RGB565)

init_display(select_display, CAM_WIDTH, CAM_HEIGHT)

MediaManager.init()

# 启动摄像头
sensor.run()

# 创建时钟对象，用于计算帧率
clock = time.clock()

try:
    # 主循环
    while True:
        clock.tick() # 更新时钟，用于计算帧率

        img = sensor.snapshot() # 拍摄一张图片

        codes = img.find_barcodes() # 在图片中查找所有条形码

        # 遍历所有找到的条形码
        for code in codes:
            # 对条码绘制矩形框以进行标记，颜色默认为白色，线宽2像素
            img.draw_rectangle(code.rect(), thickness=2)

            # 打印条码相关信息到控制台，使用中文描述和f-string格式化
            # rotation() 返回的是弧度，转换为度数
            print(f"条码类型: {barcode_name(code)}, 内容: \"{code.payload()}\", 旋转角度: {(180 * code.rotation()) / math.pi:.2f} 度, 质量: {code.quality()}")

            img.draw_string_advanced(0, 0, 30, f"类型: {barcode_name(code)}", color = (255, 255, 255))
            img.draw_string_advanced(0, 30, 30, f"内容: {code.payload()}", color = (255, 255, 255))

        Display.show_image(img) # 在显示器上显示处理后的图片

        print(f"帧率: {clock.fps():.2f} FPS") # 打印当前帧率到控制台

except KeyboardInterrupt:
    print("程序中断，正在释放资源...")
finally:
    deinit_display()
    sensor.stop() # 停止摄像头
    print("资源已释放，程序退出。")

