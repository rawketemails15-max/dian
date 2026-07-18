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
import aicube

# ==================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 3  # 1=HDMI，2=LCD，3=IDE虚拟显示

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

class HandDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels, anchors,
                 confidence_threshold=0.2, nms_threshold=0.5, nms_option=False,
                 strides=[8,16,32], rgb888p_size=[224,224], display_size=[1920,1080],
                 debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.labels = labels
        self.anchors = anchors
        self.strides = strides
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.nms_option = nms_option
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                 nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = center_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                            [1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            dets = aicube.anchorbasedet_post_process(
                results[0], results[1], results[2], self.model_input_size,
                self.rgb888p_size, self.strides, len(self.labels),
                self.confidence_threshold, self.nms_threshold, self.anchors,
                self.nms_option)
            return dets

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for det_box in dets:
                    x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                    w = float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    x1 = int(x1 * self.display_size[0] // self.rgb888p_size[0])
                    y1 = int(y1 * self.display_size[1] // self.rgb888p_size[1])
                    x2 = int(x2 * self.display_size[0] // self.rgb888p_size[0])
                    y2 = int(y2 * self.display_size[1] // self.rgb888p_size[1])
                    if (h < (0.1 * self.display_size[0])):
                        continue
                    if (w < (0.25 * self.display_size[0]) and ((x1 < (0.03 * self.display_size[0])) or (x2 > (0.97 * self.display_size[0])))):
                        continue
                    if (w < (0.15 * self.display_size[0]) and ((x1 < (0.01 * self.display_size[0])) or (x2 > (0.99 * self.display_size[0])))):
                        continue
                    pl.osd_img.draw_rectangle(x1, y1, int(w), int(h), color=(255, 0, 255, 0), thickness=2)
                    pl.osd_img.draw_string_advanced(x1, y1-50, 32,
                                                   " " + self.labels[det_box[0]] + " " + str(round(det_box[1], 2)),
                                                   color=(255, 0, 255, 0))
            else:
                pl.osd_img.clear()

if __name__=="__main__":
    display_mode_map = {1:"hdmi", 2:"lcd", 3:"ide"}
    display_mode = display_mode_map.get(select_display, "hdmi")

    if display_mode == "hdmi":
        display_size = [1920,1080]
        rgb888p_size = [1920, 1080]
    elif display_mode == "lcd":
        display_size = [800,480]
        rgb888p_size = [1920, 1080]
    else:
        display_size=[640,480]
        rgb888p_size = [1280, 960]

    kmodel_path = "/sdcard/examples/kmodel/hand_det.kmodel"
    confidence_threshold = 0.2
    nms_threshold = 0.5
    labels = ["hand"]
    anchors = [26,27, 53,52, 75,71, 80,99, 106,82, 99,134, 140,113, 161,172, 245,276]

    init_display(select_display, width=display_size[0], height=display_size[1])

    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    hand_det = HandDetectionApp(
        kmodel_path,
        model_input_size=[512,512],
        labels=labels,
        anchors=anchors,
        confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold,
        nms_option=False,
        strides=[8,16,32],
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=0
    )
    hand_det.config_preprocess()

    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = hand_det.run(img)
                hand_det.draw_result(pl, res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        hand_det.deinit()
        pl.destroy()
        deinit_display()
