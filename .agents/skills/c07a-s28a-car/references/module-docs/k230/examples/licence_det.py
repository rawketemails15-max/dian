from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math
from media.media import *
from media.display import Display
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo

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

# 自定义车牌检测类
class LicenceDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                 nn.ai2d_format.NCHW_FMT,
                                 np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1,3, self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            det_res = aidemo.licence_det_postprocess(
                results,
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                self.model_input_size,
                self.confidence_threshold,
                self.nms_threshold)
            return det_res

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            pl.osd_img.clear()
            if dets:
                point_8 = np.zeros((8), dtype=np.int16)
                for det in dets:
                    for i in range(4):
                        x = det[i*2+0] / self.rgb888p_size[0] * self.display_size[0]
                        y = det[i*2+1] / self.rgb888p_size[1] * self.display_size[1]
                        point_8[i*2+0] = int(x)
                        point_8[i*2+1] = int(y)
                    for i in range(4):
                        pl.osd_img.draw_line(
                            point_8[i*2+0], point_8[i*2+1],
                            point_8[((i+1)%4)*2+0], point_8[((i+1)%4)*2+1],
                            color=(255, 0, 255, 0), thickness=4)

if __name__=="__main__":
    display_mode_map = {1:"hdmi", 2:"lcd", 3:"ide"}
    display_mode = display_mode_map.get(select_display, "hdmi")

    if display_mode == "hdmi":
        display_size = [640,480]
    elif display_mode == "lcd":
        display_size = [800,480]
    else:
        display_size = [1280,720]

    rgb888p_size = [640,640]
    kmodel_path = "/sdcard/examples/kmodel/LPD_640.kmodel"
    confidence_threshold = 0.2
    nms_threshold = 0.2

    init_display(select_display, width=display_size[0], height=display_size[1])

    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size=pl.get_display_size()

    licence_det = LicenceDetectionApp(
        kmodel_path,
        model_input_size=[640,640],
        confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold,
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=0)
    licence_det.config_preprocess()

    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = licence_det.run(img)
                licence_det.draw_result(pl, res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        licence_det.deinit()
        pl.destroy()
        deinit_display()
