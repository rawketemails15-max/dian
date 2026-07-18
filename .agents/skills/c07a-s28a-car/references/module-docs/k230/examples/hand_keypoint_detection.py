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
import aicube

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

# 自定义手掌检测任务类
class HandDetApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,anchors,
                 confidence_threshold=0.2,nms_threshold=0.5,nms_option=False,
                 strides=[8,16,32],rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors
        self.strides = strides
        self.nms_option = nms_option
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                 nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = center_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1,3, self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            dets = aicube.anchorbasedet_post_process(
                results[0], results[1], results[2], self.model_input_size,
                self.rgb888p_size, self.strides, len(self.labels),
                self.confidence_threshold, self.nms_threshold, self.anchors,
                self.nms_option)
            return dets

# 自定义手势关键点检测任务类
class HandKPDetApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.crop_params = []
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                 nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self,det,input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            self.crop_params = self.get_crop_param(det)
            self.ai2d.crop(self.crop_params[0], self.crop_params[1], self.crop_params[2], self.crop_params[3])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1,3, self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            results=results[0].reshape(results[0].shape[0]*results[0].shape[1])
            results_show = np.zeros(results.shape, dtype=np.int16)
            results_show[0::2] = results[0::2]*self.crop_params[3] + self.crop_params[0]
            results_show[1::2] = results[1::2]*self.crop_params[2] + self.crop_params[1]
            results_show[0::2] = results_show[0::2]*(self.display_size[0]/self.rgb888p_size[0])
            results_show[1::2] = results_show[1::2]*(self.display_size[1]/self.rgb888p_size[1])
            return results_show

    def get_crop_param(self,det_box):
        x1,y1,x2,y2 = det_box[2],det_box[3],det_box[4],det_box[5]
        w,h = int(x2 - x1), int(y2 - y1)
        cx = (x1+x2)*0.5
        cy = (y1+y2)*0.5
        length = max(w,h)*0.5
        ratio_num = 1.26 * length
        x1_kp = int(max(0, cx - ratio_num))
        y1_kp = int(max(0, cy - ratio_num))
        x2_kp = int(min(self.rgb888p_size[0]-1, cx + ratio_num))
        y2_kp = int(min(self.rgb888p_size[1]-1, cy + ratio_num))
        w_kp = x2_kp - x1_kp + 1
        h_kp = y2_kp - y1_kp + 1
        return [x1_kp, y1_kp, w_kp, h_kp]

# 手掌关键点检测任务合成类
class HandKeyPointDet:
    def __init__(self, hand_det_kmodel, hand_kp_kmodel,
                 det_input_size, kp_input_size, labels, anchors,
                 confidence_threshold=0.25, nms_threshold=0.3, nms_option=False,
                 strides=[8,16,32], rgb888p_size=[1280,720], display_size=[1920,1080],
                 debug_mode=0):
        self.hand_det_kmodel = hand_det_kmodel
        self.hand_kp_kmodel = hand_kp_kmodel
        self.det_input_size = det_input_size
        self.kp_input_size = kp_input_size
        self.labels = labels
        self.anchors = anchors
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.nms_option = nms_option
        self.strides = strides
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.hand_det = HandDetApp(self.hand_det_kmodel, self.labels,
                                  model_input_size=self.det_input_size, anchors=self.anchors,
                                  confidence_threshold=self.confidence_threshold, nms_threshold=self.nms_threshold,
                                  nms_option=self.nms_option, strides=self.strides,
                                  rgb888p_size=self.rgb888p_size, display_size=self.display_size, debug_mode=0)
        self.hand_kp = HandKPDetApp(self.hand_kp_kmodel,
                                    model_input_size=self.kp_input_size,
                                    rgb888p_size=self.rgb888p_size,
                                    display_size=self.display_size)
        self.hand_det.config_preprocess()

    def run(self, input_np):
        det_boxes = self.hand_det.run(input_np)
        hand_res = []
        boxes = []
        for det_box in det_boxes:
            x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
            w, h = int(x2 - x1), int(y2 - y1)
            if (h < (0.1 * self.rgb888p_size[1])):
                continue
            if (w < (0.25 * self.rgb888p_size[0]) and ((x1 < (0.03 * self.rgb888p_size[0])) or (x2 > (0.97 * self.rgb888p_size[0])))):
                continue
            if (w < (0.15 * self.rgb888p_size[0]) and ((x1 < (0.01 * self.rgb888p_size[0])) or (x2 > (0.99 * self.rgb888p_size[0])))):
                continue
            self.hand_kp.config_preprocess(det_box)
            results_show = self.hand_kp.run(input_np)
            boxes.append(det_box)
            hand_res.append(results_show)
        return boxes, hand_res

    def draw_result(self, pl, dets, hand_res):
        pl.osd_img.clear()
        if dets:
            for k in range(len(dets)):
                det_box = dets[k]
                x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                w, h = int(x2 - x1), int(y2 - y1)
                w_det = int(float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0])
                h_det = int(float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1])
                x_det = int(x1 * self.display_size[0] // self.rgb888p_size[0])
                y_det = int(y1 * self.display_size[1] // self.rgb888p_size[1])
                pl.osd_img.draw_rectangle(x_det, y_det, w_det, h_det, color=(255, 0, 255, 0), thickness=2)

                results_show = hand_res[k]
                for i in range(len(results_show)//2):
                    pl.osd_img.draw_circle(results_show[i*2], results_show[i*2+1], 1, color=(255, 0, 255, 0), fill=False)
                for i in range(5):
                    j = i*8
                    R, G, B = [(255, 0, 0), (255, 0, 255), (255, 255, 0), (0, 255, 0), (0, 0, 255)][i]
                    pl.osd_img.draw_line(results_show[0], results_show[1], results_show[j+2], results_show[j+3], color=(255,R,G,B), thickness=3)
                    pl.osd_img.draw_line(results_show[j+2], results_show[j+3], results_show[j+4], results_show[j+5], color=(255,R,G,B), thickness=3)
                    pl.osd_img.draw_line(results_show[j+4], results_show[j+5], results_show[j+6], results_show[j+7], color=(255,R,G,B), thickness=3)
                    pl.osd_img.draw_line(results_show[j+6], results_show[j+7], results_show[j+8], results_show[j+9], color=(255,R,G,B), thickness=3)

if __name__=="__main__":
    display_mode_map = {1:"hdmi", 2:"lcd", 3:"ide"}
    display_mode = display_mode_map.get(select_display, "hdmi")

    if display_mode == "hdmi":
        display_size = [1920,1080]
        rgb888p_size = [1920, 1080]
    elif display_mode == "lcd":
        display_size = [800,480]
        rgb888p_size = [800, 480]
    else:
        display_size=[640,480]
        rgb888p_size = [640, 480]

    hand_det_kmodel_path = "/sdcard/examples/kmodel/hand_det.kmodel"
    hand_kp_kmodel_path = "/sdcard/examples/kmodel/handkp_det.kmodel"
    anchors = [26,27, 53,52, 75,71, 80,99, 106,82, 99,134, 140,113, 161,172, 245,276]
    hand_det_input_size = [512,512]
    hand_kp_input_size = [256,256]
    confidence_threshold = 0.2
    nms_threshold = 0.5
    labels = ["hand"]

    init_display(select_display, width=display_size[0], height=display_size[1])

    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    hkd = HandKeyPointDet(hand_det_kmodel_path, hand_kp_kmodel_path,
                         det_input_size=hand_det_input_size, kp_input_size=hand_kp_input_size,
                         labels=labels, anchors=anchors,
                         confidence_threshold=confidence_threshold, nms_threshold=nms_threshold,
                         nms_option=False, strides=[8,16,32],
                         rgb888p_size=rgb888p_size, display_size=display_size)

    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                det_boxes, hand_res = hkd.run(img)
                hkd.draw_result(pl, det_boxes, hand_res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        hkd.hand_det.deinit()
        hkd.hand_kp.deinit()
        pl.destroy()
        deinit_display()
