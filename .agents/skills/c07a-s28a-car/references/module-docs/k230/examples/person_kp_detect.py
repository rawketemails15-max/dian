from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math
from media.media import *
from media.display import Display  # 确保SDK支持
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


# 自定义人体关键点检测类
class PersonKeyPointApp(AIBase):
    def __init__(self, kmodel_path, model_input_size,
                 confidence_threshold=0.2, nms_threshold=0.5,
                 rgb888p_size=[1280, 720], display_size=[1920, 1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        # 骨骼连接定义
        self.SKELETON = [
            (16, 14), (14, 12), (17, 15), (15, 13), (12, 13), (6, 12), (7, 13),
            (6, 7), (6, 8), (7, 9), (8, 10), (9, 11), (2, 3), (1, 2),
            (1, 3), (2, 4), (3, 5), (4, 6), (5, 7)
        ]
        # 肢体颜色
        self.LIMB_COLORS = [
            (255, 51, 153, 255), (255, 51, 153, 255), (255, 51, 153, 255),
            (255, 51, 153, 255), (255, 255, 51, 255), (255, 255, 51, 255),
            (255, 255, 51, 255), (255, 255, 128, 0), (255, 255, 128, 0),
            (255, 255, 128, 0), (255, 255, 128, 0), (255, 255, 128, 0),
            (255, 0, 255, 0), (255, 0, 255, 0), (255, 0, 255, 0),
            (255, 0, 255, 0), (255, 0, 255, 0), (255, 0, 255, 0),
            (255, 0, 255, 0)
        ]
        # 关键点颜色，共17个
        self.KPS_COLORS = [
            (255, 0, 255, 0), (255, 0, 255, 0), (255, 0, 255, 0),
            (255, 0, 255, 0), (255, 0, 255, 0), (255, 255, 128, 0),
            (255, 255, 128, 0), (255, 255, 128, 0), (255, 255, 128, 0),
            (255, 255, 128, 0), (255, 255, 128, 0), (255, 51, 153, 255),
            (255, 51, 153, 255), (255, 51, 153, 255), (255, 51, 153, 255),
            (255, 51, 153, 255), (255, 51, 153, 255)
        ]

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                 nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = center_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1, 3, self.model_input_size[1], self.model_input_size[0]])

    def preprocess(self, input_np):
        with ScopedTiming("preprocess", self.debug_mode > 0):
            return [nn.from_numpy(input_np)]

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            results = aidemo.person_kp_postprocess(
                results[0], [self.rgb888p_size[1], self.rgb888p_size[0]],
                self.model_input_size,
                self.confidence_threshold,
                self.nms_threshold)
            return results

    def draw_result(self, pl, res):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            pl.osd_img.clear()
            if res[0]:
                kpses = res[1]
                for i in range(len(res[0])):
                    # 画关键点
                    for k in range(17 + 2):
                        if k < 17:
                            kps_x, kps_y, kps_s = round(kpses[i][k][0]), round(kpses[i][k][1]), kpses[i][k][2]
                            kps_x1 = int(float(kps_x) * self.display_size[0] // self.rgb888p_size[0])
                            kps_y1 = int(float(kps_y) * self.display_size[1] // self.rgb888p_size[1])
                            if kps_s > 0:
                                pl.osd_img.draw_circle(kps_x1, kps_y1, 5, self.KPS_COLORS[k], 4)
                        # 画骨架连接
                        if k < len(self.SKELETON):
                            ske = self.SKELETON[k]
                            pos1_x, pos1_y = round(kpses[i][ske[0] - 1][0]), round(kpses[i][ske[0] - 1][1])
                            pos1_x_ = int(float(pos1_x) * self.display_size[0] // self.rgb888p_size[0])
                            pos1_y_ = int(float(pos1_y) * self.display_size[1] // self.rgb888p_size[1])

                            pos2_x, pos2_y = round(kpses[i][ske[1] - 1][0]), round(kpses[i][ske[1] - 1][1])
                            pos2_x_ = int(float(pos2_x) * self.display_size[0] // self.rgb888p_size[0])
                            pos2_y_ = int(float(pos2_y) * self.display_size[1] // self.rgb888p_size[1])

                            pos1_s, pos2_s = kpses[i][ske[0] - 1][2], kpses[i][ske[1] - 1][2]
                            if pos1_s > 0.0 and pos2_s > 0.0:
                                pl.osd_img.draw_line(pos1_x_, pos1_y_, pos2_x_, pos2_y_, self.LIMB_COLORS[k], 4)
                    gc.collect()

if __name__ == "__main__":
    display_mode_map = {1: "hdmi", 2: "lcd", 3: "ide"}
    display_mode = display_mode_map.get(select_display, "hdmi")

    if display_mode == "hdmi":
        display_size = [1920, 1080]
    elif display_mode == "lcd":
        display_size = [800, 480]
    else:
        display_size = [1280, 720]

    rgb888p_size = [320, 320]
    kmodel_path = "/sdcard/examples/kmodel/yolov8n-pose.kmodel"
    confidence_threshold = 0.2
    nms_threshold = 0.5

    init_display(select_display, width=display_size[0], height=display_size[1])

    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    person_kp = PersonKeyPointApp(
        kmodel_path,
        model_input_size=[320, 320],
        confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold,
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=0
    )
    person_kp.config_preprocess()

    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = person_kp.run(img)
                person_kp.draw_result(pl, res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        person_kp.deinit()
        pl.destroy()
        deinit_display()
