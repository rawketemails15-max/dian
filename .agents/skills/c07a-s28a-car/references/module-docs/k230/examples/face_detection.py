from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
from media.display import Display
import os, sys, ujson, gc, math
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo

# ==================================================================
# == 请在这里选择显示模式 (1: HDMI, 2: LCD, 3: IDE 缓冲区) ==
# ==================================================================
select_display = 3  # 设置显示模式，1=HDMI，2=LCD，3=IDE虚拟显示

_display_mode_map = {
    1: 'hdmi',
    2: 'lcd',
    3: 'ide'
}

def init_display(select_display, width, height):
    """
    初始化显示设备，根据选择开启HDMI、LCD或IDE虚拟显示。
    当为IDE时，不初始化HDMI或LCD。
    """
    if select_display == 1:
        Display.init(Display.LT9611, width=width, height=height, to_ide=True)
        print(f"初始化HDMI显示，分辨率{width}x{height}")
    elif select_display == 2:
        Display.init(Display.ST7701, to_ide=True)
        print("初始化LCD显示，默认分辨率800x480")
    elif select_display == 3:
        Display.init(Display.VIRT, width=width, height=height, fps=100, to_ide=True)
        print(f"初始化IDE虚拟显示，分辨率{width}x{height}")
    else:
        raise ValueError("select_display参数错误，应为1、2或3")

def deinit_display():
    """释放显示相关资源"""
    Display.deinit()
    print("释放显示资源")

class FaceDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors,
                 confidence_threshold=0.5, nms_threshold=0.2,
                 rgb888p_size=[224, 224], display_size=[1920, 1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right,_ = letterbox_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0,0,0,0, top, bottom, left, right], 0, [104,117,123])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1,3,ai2d_input_size[1], ai2d_input_size[0]],
                [1,3,self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            post_ret = aidemo.face_det_post_process(
                self.confidence_threshold,
                self.nms_threshold,
                self.model_input_size[1],
                self.anchors,
                self.rgb888p_size,
                results)
            if len(post_ret) == 0:
                return post_ret
            else:
                return post_ret[0]

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for det in dets:
                    x,y,w,h = map(lambda x:int(round(x,0)), det[:4])
                    x = x * self.display_size[0] // self.rgb888p_size[0]
                    y = y * self.display_size[1] // self.rgb888p_size[1]
                    w = w * self.display_size[0] // self.rgb888p_size[0]
                    h = h * self.display_size[1] // self.rgb888p_size[1]
                    pl.osd_img.draw_rectangle(x, y, w, h, color=(255,255,0,255), thickness=2)
            else:
                pl.osd_img.clear()

if __name__ == "__main__":
    display_mode = _display_mode_map.get(select_display, 'hdmi')
    print(f"当前选定显示模式: {display_mode}")

    # 根据显示模式配置分辨率
    if display_mode == "hdmi":
        display_size = [1920, 1080]
    elif display_mode == "lcd":
        display_size = [800, 480]
    else:  # ide虚拟显示
        display_size = [1280, 720]

    rgb888p_size = [1280, 720]
    kmodel_path = "/sdcard/examples/kmodel/face_detection_320.kmodel"
    confidence_threshold = 0.5
    nms_threshold = 0.2
    anchor_len = 4200
    det_dim = 4
    anchors_path = "/sdcard/examples/utils/prior_data_320.bin"
    anchors = np.fromfile(anchors_path, dtype=np.float)
    anchors = anchors.reshape((anchor_len, det_dim))

    # 初始化显示设备，根据选择的display模式
    init_display(select_display, width=display_size[0], height=display_size[1])

    # 创建PipeLine对象
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    # 实例化检测类并配置
    face_det = FaceDetectionApp(kmodel_path,
                                model_input_size=[320,320],
                                anchors=anchors,
                                confidence_threshold=confidence_threshold,
                                nms_threshold=nms_threshold,
                                rgb888p_size=rgb888p_size,
                                display_size=display_size,
                                debug_mode=0)
    face_det.config_preprocess()

    try:
        while True:
            with ScopedTiming("total",1):
                img = pl.get_frame()
                res = face_det.run(img)
                face_det.draw_result(pl, res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断退出程序")
    finally:
        face_det.deinit()
        pl.destroy()
        deinit_display()
