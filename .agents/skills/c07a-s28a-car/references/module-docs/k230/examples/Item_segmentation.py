from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math, random
from media.media import *  # This will import Display
from media.sensor import *
from media.display import Display  # 确保SDK支持
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo
import time # Import time for clock

# 自定义YOLOv8分割类
class SegmentationApp(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, confidence_threshold=0.2, nms_threshold=0.5, mask_threshold=0.5, rgb888p_size=[224, 224], display_size=[1920, 1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        # 模型路径
        self.kmodel_path = kmodel_path
        # 分割类别标签
        self.labels = labels
        # 模型输入分辨率
        self.model_input_size = model_input_size
        # 置信度阈值
        self.confidence_threshold = confidence_threshold
        # nms阈值
        self.nms_threshold = nms_threshold
        # mask阈值
        self.mask_threshold = mask_threshold
        # sensor给到AI的图像分辨率
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # 显示分辨率
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        # 检测框预置颜色值
        self.color_four = get_colors(len(self.labels))
        # 分割结果的numpy.array，用于给到aidemo后处理接口
        # Note: self.masks will be populated by aidemo.segment_postprocess
        self.masks = np.zeros((1, self.display_size[1], self.display_size[0], 4), dtype=np.uint8) # Specify dtype

        # Ai2d实例，用于实现模型预处理
        self.ai2d = Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作，这里使用了pad和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = center_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]], [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # 自定义当前任务的后处理
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 这里使用了aidemo的segment_postprocess接口
            seg_res = aidemo.segment_postprocess(results, [self.rgb888p_size[1], self.rgb888p_size[0]], self.model_input_size, [self.display_size[1], self.display_size[0]], self.confidence_threshold, self.nms_threshold, self.mask_threshold, self.masks)
            return seg_res

    # 绘制结果
    def draw_result(self, pl, seg_res):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if seg_res[0]:
                pl.osd_img.clear()
                # Create image object referencing the numpy array populated by aidemo.segment_postprocess
                mask_img = image.Image(self.display_size[0], self.display_size[1], image.ARGB8888, alloc=image.ALLOC_REF, data=self.masks)
                pl.osd_img.copy_from(mask_img)
                dets, ids, scores = seg_res[0], seg_res[1], seg_res[2]
                for i, det in enumerate(dets):
                    x1, y1, w, h = map(lambda x: int(round(x, 0)), det)
                    pl.osd_img.draw_string_advanced(x1, y1 - 50, 32, " " + self.labels[int(ids[i])] + " " + str(round(scores[i], 2)), color=self.color_four[int(ids[i])])
            else:
                pl.osd_img.clear()


# ==================================================================
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 3  # 1=HDMI，2=LCD，3=IDE虚拟显示

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
        Display.init(Display.LT9611, width=width, height=height, to_ide=True)
        print(f"初始化HDMI显示，分辨率：{width}x{height}")
    elif select_display_mode == 2:
        width, height = 800, 480
        Display.init(Display.ST7701, width=width, height=height, to_ide=True)
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

if __name__ == "__main__":
    display_width, display_height = 0, 0
    display_mode_str = ""
    pl = None
    seg = None

    try:
        # Initialize display based on selection
        display_width, display_height = init_display(select_display)

        if select_display == 1:  # HDMI
            display_mode_str = 'hdmi'
        elif select_display == 2:  # LCD
            display_mode_str = 'lcd'
        elif select_display == 3:  # IDE Virtual
            display_mode_str = 'virtual'

        display_size = [display_width, display_height]
        rgb888p_size = [320, 320] # Fixed for model input preprocessing

        # Model and thresholds
        kmodel_path = "/sdcard/examples/kmodel/yolov8n_seg_320.kmodel"
        labels = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
        confidence_threshold = 0.2
        nms_threshold = 0.5
        mask_threshold = 0.5

        # Initialize PipeLine
        pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode_str)

        # Determine sensor resolution based on typical usage for these displays
        sensor_width, sensor_height = 1920, 1080 # Default for 16:9 aspect ratios
        # If display is 640x480 (4:3), use a sensor resolution that maintains a 4:3 aspect ratio
        if display_width == 640 and display_height == 480:
            sensor_width, sensor_height = 1280, 960 # Example 4:3 resolution

        pl.create(Sensor(width=sensor_width, height=sensor_height))

        # Initialize SegmentationApp
        seg = SegmentationApp(kmodel_path, labels=labels, model_input_size=[320, 320],
                              confidence_threshold=confidence_threshold, nms_threshold=nms_threshold,
                              mask_threshold=mask_threshold, rgb888p_size=rgb888p_size,
                              display_size=display_size, debug_mode=0)
        seg.config_preprocess()

        clock = time.clock() # Initialize clock for FPS

        # Main loop
        while True:
            clock.tick() # Start timing for the current frame
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                seg_res = seg.run(img)
                seg.draw_result(pl, seg_res)
                pl.show_image()
                gc.collect()

            print(clock.fps()) # Print FPS

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Ensure resources are properly released
        if seg:
            seg.deinit()
        if pl:
            pl.destroy()
        deinit_display()

