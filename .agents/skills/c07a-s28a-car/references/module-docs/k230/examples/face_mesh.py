from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os
import sys
import ujson
import gc
import math
from media.media import *
from media.display import Display
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo

# ==================================================================
# == 请在这里选择显示模式 (1: HDMI, 2: LCD, 3: IDE 缓冲区) ==
# ==================================================================
select_display = 3  # 1=HDMI，2=LCD，3=IDE虚拟显示

# 显示模式映射
_display_mode_map = {
    1: 'hdmi',
    2: 'lcd',
    3: 'ide'
}

# 显示设备初始化函数
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
        raise ValueError("select_display参数错误，应为1、2或3")

def deinit_display():
    Display.deinit()
    print("释放显示资源")

class FaceDetApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors,
                 confidence_threshold=0.25, nms_threshold=0.3,
                 rgb888p_size=[1280,720], display_size=[1920,1080],
                 debug_mode=0):
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
            np.uint8,
            np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = letterbox_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0,0,0,0, top, bottom, left, right], 0, [104,117,123])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1,3,ai2d_input_size[1], ai2d_input_size[0]],
                [1,3,self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            res = aidemo.face_det_post_process(
                self.confidence_threshold,
                self.nms_threshold,
                self.model_input_size[0],
                self.anchors,
                self.rgb888p_size,
                results)
            if len(res) == 0:
                return res
            else:
                return res[0]

class FaceMeshApp(AIBase):
    def __init__(self, kmodel_path, model_input_size,
                 rgb888p_size=[1920,1080], display_size=[1920,1080],
                 debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.param_mean = np.array([
            0.0003492636315058917,2.52790130161884e-07,-6.875197868794203e-07,60.1679573059082,
            -6.295513230725192e-07,0.0005757200415246189,-5.085391239845194e-05,74.2781982421875,
            5.400917189035681e-07,6.574138387804851e-05,0.0003442012530285865,-66.67157745361328,
            -346603.6875,-67468.234375,46822.265625,-15262.046875,4350.5888671875,-54261.453125,
            -18328.033203125,-1584.328857421875,-84566.34375,3835.960693359375,-20811.361328125,
            38094.9296875,-19967.85546875,-9241.3701171875,-19600.71484375,13168.08984375,
            -5259.14404296875,1848.6478271484375,-13030.662109375,-2435.55615234375,-2254.20654296875,
            -14396.5615234375,-6176.3291015625,-25621.919921875,226.39447021484375,-6326.12353515625,
            -10867.2509765625,868.465087890625,-5831.14794921875,2705.123779296875,-3629.417724609375,
            2043.9901123046875,-2446.6162109375,3658.697021484375,-7645.98974609375,-6674.45263671875,
            116.38838958740234,7185.59716796875,-1429.48681640625,2617.366455078125,-1.2070955038070679,
            0.6690792441368103,-0.17760828137397766,0.056725528091192245,0.03967815637588501,
            -0.13586315512657166,-0.09223993122577667,-0.1726071834564209,-0.015804484486579895,
            -0.1416848599910736], dtype=np.float)
        self.param_std = np.array([
            0.00017632152594160289,6.737943476764485e-05,0.00044708489440381527,26.55023193359375,
            0.0001231376954820007,4.493021697271615e-05,7.923670636955649e-05,6.982563018798828,
            0.0004350444069132209,0.00012314890045672655,0.00017400001524947584,20.80303955078125,
            575421.125,277649.0625,258336.84375,255163.125,150994.375,160086.109375,111277.3046875,
            97311.78125,117198.453125,89317.3671875,88493.5546875,72229.9296875,71080.2109375,
            50013.953125,55968.58203125,47525.50390625,49515.06640625,38161.48046875,44872.05859375,
            46273.23828125,38116.76953125,28191.162109375,32191.4375,36006.171875,32559.892578125,
            25551.1171875,24267.509765625,27521.3984375,23166.53125,21101.576171875,19412.32421875,
            19452.203125,17454.984375,22537.623046875,16174.28125,14671.640625,15115.6884765625,
            13870.0732421875,13746.3125,12663.1337890625,1.5870834589004517,1.5077009201049805,
            0.5881357789039612,0.5889744758605957,0.21327851712703705,0.2630201280117035,
            0.2796429395675659,0.38030216097831726,0.16162841022014618,0.2559692859649658], dtype=np.float)
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8)

    def config_preprocess(self, det, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            roi = self.parse_roi_box_from_bbox(det)
            self.ai2d.crop(int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]))
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]])
            return roi

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            param = results[0] * self.param_std + self.param_mean
            return param

    def parse_roi_box_from_bbox(self, bbox):
        x1, y1, w, h = map(lambda x: int(round(x, 0)), bbox[:4])
        old_size = (w + h) / 2
        center_x = x1 + w / 2
        center_y = y1 + h / 2 + old_size * 0.14
        size = int(old_size * 1.58)
        x0 = center_x - float(size) / 2
        y0 = center_y - float(size) / 2
        x1 = x0 + size
        y1 = y0 + size
        x0 = max(0, min(x0, self.rgb888p_size[0]))
        y0 = max(0, min(y0, self.rgb888p_size[1]))
        x1 = max(0, min(x1, self.rgb888p_size[0]))
        y1 = max(0, min(y1, self.rgb888p_size[1]))
        roi = (x0, y0, x1 - x0, y1 - y0)
        return roi

class FaceMeshPostApp(AIBase):
    def __init__(self, kmodel_path, model_input_size,
                 rgb888p_size=[1920,1080], display_size=[1920,1080],
                 debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8)

    def preprocess(self, param):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            param = param[0]
            trans_dim, shape_dim, exp_dim = 12, 40, 10
            R_ = param[:trans_dim].copy().reshape((3, -1))
            R = R_[:, :3].copy()
            offset = R_[:, 3].copy()
            offset = offset.reshape((3, 1))
            alpha_shp = param[trans_dim:trans_dim + shape_dim].copy().reshape((-1, 1))
            alpha_exp = param[trans_dim + shape_dim:].copy().reshape((-1, 1))
            R_tensor = nn.from_numpy(R)
            offset_tensor = nn.from_numpy(offset)
            alpha_shp_tensor = nn.from_numpy(alpha_shp)
            alpha_exp_tensor = nn.from_numpy(alpha_exp)
            return [R_tensor, offset_tensor, alpha_shp_tensor, alpha_exp_tensor]

    def postprocess(self, results, roi):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            x, y, w, h = map(lambda x: int(round(x, 0)), roi[:4])
            x = x * self.display_size[0] // self.rgb888p_size[0]
            y = y * self.display_size[1] // self.rgb888p_size[1]
            w = w * self.display_size[0] // self.rgb888p_size[0]
            h = h * self.display_size[1] // self.rgb888p_size[1]
            roi_array = np.array([x, y, w, h], dtype=np.float)
            aidemo.face_mesh_post_process(roi_array, results[0])
            return results[0]

class FaceMesh:
    def __init__(self, face_det_kmodel, face_mesh_kmodel, mesh_post_kmodel,
                 det_input_size, mesh_input_size, anchors,
                 confidence_threshold=0.25, nms_threshold=0.3,
                 rgb888p_size=[1920,1080], display_size=[1920,1080],
                 debug_mode=0):
        self.face_det_kmodel = face_det_kmodel
        self.face_mesh_kmodel = face_mesh_kmodel
        self.mesh_post_kmodel = mesh_post_kmodel
        self.det_input_size = det_input_size
        self.mesh_input_size = mesh_input_size
        self.anchors = anchors
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode

        self.face_det = FaceDetApp(
            self.face_det_kmodel,
            model_input_size=self.det_input_size,
            anchors=self.anchors,
            confidence_threshold=self.confidence_threshold,
            nms_threshold=self.nms_threshold,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            debug_mode=debug_mode)

        self.face_mesh = FaceMeshApp(
            self.face_mesh_kmodel,
            model_input_size=self.mesh_input_size,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            debug_mode=debug_mode)

        self.face_mesh_post = FaceMeshPostApp(
            self.mesh_post_kmodel,
            model_input_size=self.mesh_input_size,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            debug_mode=debug_mode)

        self.face_det.config_preprocess()

    def run(self, input_np):
        det_boxes = self.face_det.run(input_np)
        mesh_res = []
        for det_box in det_boxes:
            roi = self.face_mesh.config_preprocess(det_box)
            param = self.face_mesh.run(input_np)
            tensors = self.face_mesh_post.preprocess(param)
            results = self.face_mesh_post.inference(tensors)
            res = self.face_mesh_post.postprocess(results, roi)
            mesh_res.append(res)
        return det_boxes, mesh_res

    def draw_result(self, pl, dets, mesh_res):
        pl.osd_img.clear()
        if dets:
            draw_img_np = np.zeros((self.display_size[1], self.display_size[0], 4), dtype=np.uint8)
            draw_img = image.Image(self.display_size[0], self.display_size[1], image.ARGB8888,
                                   alloc=image.ALLOC_REF, data=draw_img_np)
            for vertices in mesh_res:
                aidemo.face_draw_mesh(draw_img_np, vertices)
            pl.osd_img.copy_from(draw_img)

if __name__ == "__main__":
    display_mode = _display_mode_map.get(select_display, 'hdmi')

    if display_mode == 'hdmi':
        display_size = [1920, 1080]
    elif display_mode == 'lcd':
        display_size = [800, 480]
    else:
        display_size = [1280, 720]

    rgb888p_size = [1280, 720]

    face_det_kmodel_path = "/sdcard/examples/kmodel/face_detection_320.kmodel"
    face_mesh_kmodel_path = "/sdcard/examples/kmodel/face_alignment.kmodel"
    face_mesh_post_kmodel_path = "/sdcard/examples/kmodel/face_alignment_post.kmodel"
    anchors_path = "/sdcard/examples/utils/prior_data_320.bin"

    face_det_input_size = [320, 320]
    face_mesh_input_size = [120, 120]
    confidence_threshold = 0.5
    nms_threshold = 0.2
    anchor_len = 4200
    det_dim = 4

    anchors = np.fromfile(anchors_path, dtype=np.float)
    anchors = anchors.reshape((anchor_len, det_dim))

    # 初始化显示
    init_display(select_display, width=display_size[0], height=display_size[1])

    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    fm = FaceMesh(
        face_det_kmodel_path,
        face_mesh_kmodel_path,
        face_mesh_post_kmodel_path,
        det_input_size=face_det_input_size,
        mesh_input_size=face_mesh_input_size,
        anchors=anchors,
        confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold,
        rgb888p_size=rgb888p_size,
        display_size=display_size,
        debug_mode=0)

    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                det_boxes, mesh_res = fm.run(img)
                fm.draw_result(pl, det_boxes, mesh_res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断退出")
    finally:
        fm.face_det.deinit()
        fm.face_mesh.deinit()
        fm.face_mesh_post.deinit()
        pl.destroy()
        deinit_display()
