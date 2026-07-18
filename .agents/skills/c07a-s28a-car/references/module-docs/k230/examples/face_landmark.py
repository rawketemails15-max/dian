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
# == 请在这里选择显示模式 (1: HDMI, 2: LCD, 3: IDE 缓冲区) ==
# ==================================================================
select_display = 1  # 设置显示模式，1=HDMI，2=LCD，3=IDE虚拟显示

def init_display(select_display, width, height):
    """
    根据select_display初始化对应显示设备：
    1: HDMI，默认分辨率1920x1080
    2: LCD ，默认分辨率800x480
    3: IDE虚拟显示，仅启用IDE缓冲区，分辨率可自定义，不初始化HDMI和LCD
    """
    if select_display == 1:
        # HDMI显示初始化，使用例LT9611芯片，分辨率1920x1080
        Display.init(Display.LT9611, width=width, height=height, to_ide=True)
        print("初始化HDMI显示，分辨率：{}x{}".format(width, height))
    elif select_display == 2:
        # LCD显示初始化，例ST7701屏幕，分辨率800x480
        Display.init(Display.ST7701, to_ide=True)
        print("初始化LCD显示，分辨率：800x480")
    elif select_display == 3:
        # IDE虚拟显示，仅启用虚拟显示缓冲区，不初始化HDMI和LCD
        Display.init(Display.VIRT, width=1280, height=720, to_ide=True)
        print("初始化IDE虚拟显示，分辨率：{}x{}".format(width, height))
    else:
        raise ValueError("select_display 参数错误，应为 1、2 或 3")

def deinit_display():
    """
    统一释放显示资源，不管哪种设备都调用
    """
    Display.deinit()
    print("释放显示资源")

# ------- 人脸检测类 -------
class FaceDetApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors,
                 confidence_threshold=0.25, nms_threshold=0.3,
                 rgb888p_size=[1280,720], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                nn.ai2d_format.NCHW_FMT,
                                np.uint8,
                                np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = letterbox_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [104, 117, 123])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                            [1,3,self.model_input_size[1],self.model_input_size[0]])

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


# ------- 人脸关键点识别类 -------
class FaceLandMarkApp(AIBase):
    def __init__(self, kmodel_path, model_input_size,
                 rgb888p_size=[1920,1080], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.matrix_dst = None
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                nn.ai2d_format.NCHW_FMT,
                                np.uint8,
                                np.uint8)

    def config_preprocess(self, det, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            self.matrix_dst = self.get_affine_matrix(det)
            affine_matrix = [
                self.matrix_dst[0][0],
                self.matrix_dst[0][1],
                self.matrix_dst[0][2],
                self.matrix_dst[1][0],
                self.matrix_dst[1][1],
                self.matrix_dst[1][2]
            ]
            self.ai2d.affine(nn.interp_method.cv2_bilinear, 0, 0, 127, 1, affine_matrix)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                            [1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            pred = results[0]
            half_input_len = self.model_input_size[0] // 2
            pred = pred.flatten()
            for i in range(len(pred)):
                pred[i] += (pred[i] + 1) * half_input_len
            matrix_dst_inv = aidemo.invert_affine_transform(self.matrix_dst).flatten()
            half_out_len = len(pred) // 2
            for kp_id in range(half_out_len):
                old_x = pred[kp_id * 2]
                old_y = pred[kp_id * 2 + 1]
                new_x = old_x * matrix_dst_inv[0] + old_y * matrix_dst_inv[1] + matrix_dst_inv[2]
                new_y = old_x * matrix_dst_inv[3] + old_y * matrix_dst_inv[4] + matrix_dst_inv[5]
                pred[kp_id * 2] = new_x
                pred[kp_id * 2 + 1] = new_y
            return pred

    def get_affine_matrix(self, bbox):
        with ScopedTiming("get_affine_matrix", self.debug_mode > 1):
            x1, y1, w, h = map(lambda x: int(round(x, 0)), bbox[:4])
            scale_ratio = self.model_input_size[0] / (max(w, h) * 1.5)
            cx = (x1 + w / 2) * scale_ratio
            cy = (y1 + h / 2) * scale_ratio
            half_input_len = self.model_input_size[0] / 2
            matrix_dst = np.zeros((2, 3), dtype=np.float)
            matrix_dst[0, 0] = scale_ratio
            matrix_dst[0, 1] = 0
            matrix_dst[0, 2] = half_input_len - cx
            matrix_dst[1, 0] = 0
            matrix_dst[1, 1] = scale_ratio
            matrix_dst[1, 2] = half_input_len - cy
            return matrix_dst


# ------- 人脸关键点整体管理类 -------
class FaceLandMark:
    def __init__(self, face_det_kmodel, face_landmark_kmodel,
                 det_input_size, landmark_input_size,
                 anchors, confidence_threshold=0.25, nms_threshold=0.3,
                 rgb888p_size=[1920,1080], display_size=[1920,1080], debug_mode=0):
        self.face_det_kmodel = face_det_kmodel
        self.face_landmark_kmodel = face_landmark_kmodel
        self.det_input_size = det_input_size
        self.landmark_input_size = landmark_input_size
        self.anchors = anchors
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        # 人脸关键点区域索引（左眉、右眉、左眼、右眼、瞳孔、鼻梁、鼻翼、外嘴唇、内嘴唇、脸盆）
        self.dict_kp_seq = [
            [43, 44, 45, 47, 46, 50, 51, 49, 48],
            [97, 98, 99, 100, 101, 105, 104, 103, 102],
            [35, 36, 33, 37, 39, 42, 40, 41],
            [89, 90, 87, 91, 93, 96, 94, 95],
            [34, 88],
            [72, 73, 74, 86],
            [77, 78, 79, 80, 85, 84, 83],
            [52, 55, 56, 53, 59, 58, 61, 68, 67, 71, 63, 64],
            [65, 54, 60, 57, 69, 70, 62, 66],
            [1, 9, 10, 11, 12, 13, 14, 15, 16,
             2, 3, 4, 5, 6, 7, 8, 0,
             24, 23, 22, 21, 20, 19, 18,
             32, 31, 30, 29, 28, 27, 26, 25, 17]
        ]

        # 各区域对应颜色（ARGB格式）
        self.color_list_for_osd_kp = [
            (255, 0, 255, 0),
            (255, 0, 255, 0),
            (255, 255, 0, 255),
            (255, 255, 0, 255),
            (255, 255, 0, 0),
            (255, 255, 170, 0),
            (255, 255, 255, 0),
            (255, 0, 255, 255),
            (255, 255, 220, 50),
            (255, 30, 30, 255)
        ]

        self.face_det = FaceDetApp(self.face_det_kmodel,
                                   model_input_size=self.det_input_size,
                                   anchors=self.anchors,
                                   confidence_threshold=self.confidence_threshold,
                                   nms_threshold=self.nms_threshold,
                                   rgb888p_size=self.rgb888p_size,
                                   display_size=self.display_size,
                                   debug_mode=self.debug_mode)

        self.face_landmark = FaceLandMarkApp(self.face_landmark_kmodel,
                                             model_input_size=self.landmark_input_size,
                                             rgb888p_size=self.rgb888p_size,
                                             display_size=self.display_size,
                                             debug_mode=self.debug_mode)

        # 配置检测的预处理
        self.face_det.config_preprocess()

    def run(self, input_np):
        det_boxes = self.face_det.run(input_np)
        landmark_res = []
        for det_box in det_boxes:
            self.face_landmark.config_preprocess(det_box)
            res = self.face_landmark.run(input_np)
            landmark_res.append(res)
        return det_boxes, landmark_res

    def draw_result(self, pl, dets, landmark_res):
        pl.osd_img.clear()
        if dets:
            draw_img_np = np.zeros((self.display_size[1], self.display_size[0], 4), dtype=np.uint8)
            draw_img = image.Image(self.display_size[0], self.display_size[1], image.ARGB8888,
                                   alloc=image.ALLOC_REF, data=draw_img_np)
            for pred in landmark_res:
                for sub_part_index in range(len(self.dict_kp_seq)):
                    sub_part = self.dict_kp_seq[sub_part_index]
                    face_sub_part_point_set = []
                    for kp_index in range(len(sub_part)):
                        real_kp_index = sub_part[kp_index]
                        x, y = pred[real_kp_index*2], pred[real_kp_index*2+1]
                        x = int(x * self.display_size[0] // self.rgb888p_size[0])
                        y = int(y * self.display_size[1] // self.rgb888p_size[1])
                        face_sub_part_point_set.append((x, y))
                    if sub_part_index in (9, 6):
                        color = np.array(self.color_list_for_osd_kp[sub_part_index], dtype=np.uint8)
                        face_sub_part_point_set = np.array(face_sub_part_point_set)
                        aidemo.polylines(draw_img_np, face_sub_part_point_set, False, color, 5, 8, 0)
                    elif sub_part_index == 4:
                        color = self.color_list_for_osd_kp[sub_part_index]
                        for kp in face_sub_part_point_set:
                            draw_img.draw_circle(kp[0], kp[1], 2, color, 1)
                    else:
                        color = np.array(self.color_list_for_osd_kp[sub_part_index], dtype=np.uint8)
                        face_sub_part_point_set = np.array(face_sub_part_point_set)
                        aidemo.contours(draw_img_np, face_sub_part_point_set, -1, color, 2, 8)
            pl.osd_img.copy_from(draw_img)


if __name__ == "__main__":
    # 显示模式映射
    display_mode_map = {
        1: "hdmi",
        2: "lcd",
        3: "ide"
    }

    display_mode = display_mode_map.get(select_display, "hdmi")

    # 根据显示模式设置显示分辨率
    if display_mode == "hdmi":
        display_size = [1920, 1080]
    elif display_mode == "lcd":
        display_size = [800, 480]
    else:
        # IDE虚拟显示使用输入分辨率或者自定义
        display_size = [1280, 720]

    # 设置输入分辨率
    rgb888p_size = [1280, 720]

    # 模型和参数设置
    face_det_kmodel_path = "/sdcard/examples/kmodel/face_detection_320.kmodel"
    face_landmark_kmodel_path = "/sdcard/examples/kmodel/face_landmark.kmodel"
    anchors_path = "/sdcard/examples/utils/prior_data_320.bin"
    face_det_input_size = [320, 320]
    face_landmark_input_size = [192, 192]
    confidence_threshold = 0.5
    nms_threshold = 0.2
    anchor_len = 4200
    det_dim = 4

    anchors = np.fromfile(anchors_path, dtype=np.float)
    anchors = anchors.reshape((anchor_len, det_dim))

    # 初始化显示设备（包含IDE虚拟显示的区别初始化）
    init_display(select_display, width=display_size[0], height=display_size[1])

    # 初始化PipeLine
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()

    # 修正display_size与PipeLine同步
    display_size = pl.get_display_size()

    # 初始化人脸关键点管理类
    flm = FaceLandMark(face_det_kmodel_path,
                       face_landmark_kmodel_path,
                       det_input_size=face_det_input_size,
                       landmark_input_size=face_landmark_input_size,
                       anchors=anchors,
                       confidence_threshold=confidence_threshold,
                       nms_threshold=nms_threshold,
                       rgb888p_size=rgb888p_size,
                       display_size=display_size,
                       debug_mode=0)
    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                det_boxes, landmark_res = flm.run(img)
                flm.draw_result(pl, det_boxes, landmark_res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断退出")
    finally:
        # 清理资源
        flm.face_det.deinit()
        flm.face_landmark.deinit()
        pl.destroy()
        deinit_display()
