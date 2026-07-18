# Camera 示例
import time
import os
import sys

from media.sensor import *
from media.display import *
from media.media import *

sensor = None

try:
    print("camera_test")

    # 根据默认配置构建 Sensor 对象
    sensor = Sensor()
    # 复位 sensor
    sensor.reset()

    # 设置通道 0 分辨率为 1920x1080
    sensor.set_framesize(Sensor.FHD)
    # 设置通道 0 格式为 YUV420SP
    sensor.set_pixformat(Sensor.YUV420SP)
    # 绑定通道 0 到显示 VIDEO1 层
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # 设置通道 1 分辨率和格式
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_1)

    # 设置通道 2 分辨率和格式
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_2)

    # 初始化 HDMI 和 IDE 输出显示，若屏幕无法点亮，请参考 API 文档中的 K230_CanMV_Display 模块 API 手册进行配置
    Display.init(Display.LT9611, to_ide=True, osd_num=2)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动 sensor
    sensor.run()

    while True:
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_1)
        Display.show_image(img, alpha=128)

        img = sensor.snapshot(chn=CAM_CHN_ID_2)
        Display.show_image(img, x=1920 - 640, layer=Display.LAYER_OSD1)

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    # 停止 sensor
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 销毁显示
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()
