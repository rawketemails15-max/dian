from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
from media.media import *  # Import Display from here
from media.sensor import *
from media.display import Display # 明确导入Display类
import nncase_runtime as nn
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import random
import gc
import sys
import aidemo

# ==================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 2  # 1=HDMI，2=LCD，3=IDE虚拟显示

def init_display(select_display_mode):
    """
    根据选择的模式初始化显示设备。
    Args:
        select_display_mode (int): 1为HDMI，2为LCD，3为IDE虚拟显示。
    Returns:
        tuple: (width, height) 返回实际的显示分辨率。
    """
    if select_display_mode == 1:
        width, height = 640, 480
        Display.init(Display.LT9611, width=width, height=height, to_ide=False)
        print(f"初始化HDMI显示，分辨率：{width}x{height}")
    elif select_display_mode == 2:
        width, height = 800, 480  # LCD (ST7701) 常见分辨率
        Display.init(Display.ST7701, width=width, height=height, to_ide=False)
        print(f"初始化LCD显示，分辨率：{width}x{height}")
    elif select_display_mode == 3:
        width, height = 800, 480
        Display.init(Display.VIRT, width=width, height=height, fps=100, to_ide=True)
        print(f"初始化IDE虚拟显示，分辨率：{width}x{height}")
    else:
        raise ValueError("select_display参数错误，只能是1、2或3")
    return width, height

def deinit_display():
    """
    释放显示资源。
    """
    Display.deinit()
    print("释放显示资源")

# 自定义YOLOv8检测类
class ObjectDetectionApp(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224, 224], display_size=[1920, 1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        # 模型输入分辨率
        self.model_input_size = model_input_size
        # 阈值设置
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        # sensor给到AI的图像分辨率
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # 显示分辨率
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        # 检测框预置颜色值
        self.color_four = [(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
                         (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
                         (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
                         (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
                         (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157)]
        # 宽高缩放比例
        self.x_factor = float(self.rgb888p_size[0]) / self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1]) / self.model_input_size[1]
        # Ai2d实例，用于实现模型预处理
        self.ai2d = Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作，这里使用了resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/libs/AI2D.py查看
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]], [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # 自定义当前任务的后处理
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            result = results[0]
            result = result.reshape((result.shape[0] * result.shape[1], result.shape[2]))
            output_data = result.transpose()
            boxes_ori = output_data[:, 0:4]
            scores_ori = output_data[:, 4:]
            confs_ori = np.max(scores_ori, axis=-1)
            inds_ori = np.argmax(scores_ori, axis=-1)
            boxes, scores, inds = [], [], []
            for i in range(len(boxes_ori)):
                if confs_ori[i] > self.confidence_threshold: # Use self.confidence_threshold
                    scores.append(confs_ori[i])
                    inds.append(inds_ori[i])
                    x = boxes_ori[i, 0]
                    y = boxes_ori[i, 1]
                    w = boxes_ori[i, 2]
                    h = boxes_ori[i, 3]
                    left = int((x - 0.5 * w) * self.x_factor)
                    top = int((y - 0.5 * h) * self.y_factor)
                    right = int((x + 0.5 * w) * self.x_factor)
                    bottom = int((y + 0.5 * h) * self.y_factor)
                    boxes.append([left, top, right, bottom])
            if len(boxes) == 0:
                return []
            boxes = np.array(boxes)
            scores = np.array(scores)
            inds = np.array(inds)
            # NMS过程
            keep = self.nms(boxes, scores, self.nms_threshold) # Use self.nms_threshold
            dets = np.concatenate((boxes, scores.reshape((len(boxes), 1)), inds.reshape((len(boxes), 1))), axis=1)
            dets_out = []
            for keep_i in keep:
                dets_out.append(dets[keep_i])
            dets_out = np.array(dets_out)
            dets_out = dets_out[:self.max_boxes_num, :]
            return dets_out

    # 绘制结果
    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for det in dets:
                    x1, y1, x2, y2 = map(lambda x: int(round(x, 0)), det[:4])
                    x = x1 * self.display_size[0] // self.rgb888p_size[0]
                    y = y1 * self.display_size[1] // self.rgb888p_size[1]
                    w = (x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = (y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    pl.osd_img.draw_rectangle(x, y, w, h, color=self.get_color(int(det[5])), thickness=4)
                    pl.osd_img.draw_string_advanced(x, y - 50, 32, " " + self.labels[int(det[5])] + " " + str(round(det[4], 2)), color=self.get_color(int(det[5])))
            else:
                pl.osd_img.clear()

    # 多目标检测 非最大值抑制方法实现
    def nms(self, boxes, scores, thresh):
        """Pure Python NMS baseline."""
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = np.argsort(scores, axis=0)[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            new_x1, new_y1, new_x2, new_y2, new_areas = [], [], [], [], []
            for order_i in order:
                new_x1.append(x1[order_i])
                new_x2.append(x2[order_i])
                new_y1.append(y1[order_i])
                new_y2.append(y2[order_i])
                new_areas.append(areas[order_i])
            new_x1 = np.array(new_x1)
            new_x2 = np.array(new_x2)
            new_y1 = np.array(new_y1)
            new_y2 = np.array(new_y2)
            xx1 = np.maximum(x1[i], new_x1)
            yy1 = np.maximum(y1[i], new_y1)
            xx2 = np.minimum(x2[i], new_x2)
            yy2 = np.minimum(y2[i], new_y2)
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            new_areas = np.array(new_areas)
            ovr = inter / (areas[i] + new_areas - inter)
            new_order = []
            for ovr_i, ind in enumerate(ovr):
                if ind < thresh:
                    new_order.append(order[ovr_i])
            order = np.array(new_order, dtype=np.uint8)
        return keep

    # 根据当前类别索引获取框的颜色
    def get_color(self, x):
        idx = x % len(self.color_four)
        return self.color_four[idx]

if __name__ == "__main__":
    display_width, display_height = 0, 0
    display_mode_str = ""

    try:
        display_width, display_height = init_display(select_display)

        if select_display == 1:
            display_mode_str = 'hdmi'
        elif select_display == 2:
            display_mode_str = 'st7701'
        elif select_display == 3:
            display_mode_str = 'virtual'

        display_size = [display_width, display_height]
        rgb888p_size = [320, 320]

        # Determine sensor resolution based on the display resolution
        sensor_width, sensor_height = 1920, 1080  # Default sensor resolution
        if display_width == 640 and display_height == 480: # If display is 640x480, use 1280x960 sensor
            sensor_width, sensor_height = 1280, 960

        pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode_str)
        pl.create(Sensor(width=sensor_width, height=sensor_height))

        kmodel_path = "/sdcard/examples/kmodel/yolov8n_320.kmodel"
        labels = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
        confidence_threshold = 0.2
        nms_threshold = 0.2
        max_boxes_num = 50

        ob_det = ObjectDetectionApp(kmodel_path, labels=labels, model_input_size=[320, 320], max_boxes_num=max_boxes_num, confidence_threshold=confidence_threshold, nms_threshold=nms_threshold, rgb888p_size=rgb888p_size, display_size=display_size, debug_mode=0)
        ob_det.config_preprocess()

        clock = time.clock()

        while True:
            clock.tick()

            img = pl.get_frame()
            res = ob_det.run(img)
            ob_det.draw_result(pl, res)
            print(res)
            pl.show_image()
            gc.collect()

            print(clock.fps())
    finally:
        deinit_display()
