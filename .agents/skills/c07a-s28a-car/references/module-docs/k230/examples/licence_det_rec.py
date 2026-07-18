from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math
from media.media import * # 包含Display类
from media.display import Display # 明确导入Display类
import nncase_runtime as nn # 确保 nn 被正确导入
import ulab.numpy as np
import image
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
        Display.init(Display.LT9611, width=width, height=height, to_ide=True)
        print(f"初始化HDMI显示，分辨率：{width}x{height}")
    elif select_display_mode == 2:
        width, height = 800, 480 # LCD (ST7701) 常见分辨率
        Display.init(Display.ST7701, width=width, height=height, to_ide=True)
        print(f"初始化LCD显示，分辨率：{width}x{height}")
    elif select_display_mode == 3:
        width, height = 1280, 720
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

# 自定义车牌检测类
class LicenceDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
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
            det_res = aidemo.licence_det_postprocess(results,
                                                     [self.rgb888p_size[1], self.rgb888p_size[0]],
                                                     self.model_input_size,
                                                     self.confidence_threshold,
                                                     self.nms_threshold)
            return det_res

# 自定义车牌识别类
class LicenceRecognitionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, rgb888p_size=[1920,1080], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.dict_rec = ["挂", "使", "领", "澳", "港", "皖", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑", "苏", "浙",
                         "京", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤", "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青",
                         "宁", "新", "警", "学", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E",
                         "F", "G", "H", "J", "K", "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "_", "-"]
        self.dict_size = len(self.dict_rec)
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
            output_data = results[0].reshape((-1, self.dict_size))
            max_indices = np.argmax(output_data, axis=1)
            result_str = ""
            for i in range(max_indices.shape[0]):
                index = max_indices[i]
                if index > 0 and (i == 0 or index != max_indices[i - 1]):
                    result_str += self.dict_rec[index - 1]
            return result_str

# 车牌识别整合类
class LicenceRec:
    def __init__(self,
                 licence_det_kmodel,
                 licence_rec_kmodel,
                 det_input_size,
                 rec_input_size,
                 confidence_threshold=0.25,
                 nms_threshold=0.3,
                 rgb888p_size=[1920,1080],
                 display_size=[1920,1080],
                 debug_mode=0):
        self.licence_det_kmodel = licence_det_kmodel
        self.licence_rec_kmodel = licence_rec_kmodel
        self.det_input_size = det_input_size
        self.rec_input_size = rec_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]] # 确保 display_size 宽度16字节对齐
        self.debug_mode = debug_mode
        self.licence_det = LicenceDetectionApp(self.licence_det_kmodel,
                                               model_input_size=self.det_input_size,
                                               confidence_threshold=self.confidence_threshold,
                                               nms_threshold=self.nms_threshold,
                                               rgb888p_size=self.rgb888p_size,
                                               display_size=self.display_size, # 传递给子类
                                               debug_mode=0)
        self.licence_rec = LicenceRecognitionApp(self.licence_rec_kmodel,
                                                 model_input_size=self.rec_input_size,
                                                 rgb888p_size=self.rgb888p_size,
                                                 display_size=self.display_size) # 传递给子类
        self.licence_det.config_preprocess()

    def run(self, input_np):
        det_boxes = self.licence_det.run(input_np)
        imgs_array_boxes = aidemo.ocr_rec_preprocess(input_np,
                                                     [self.rgb888p_size[1], self.rgb888p_size[0]],
                                                     det_boxes)
        imgs_array = imgs_array_boxes[0]
        boxes = imgs_array_boxes[1] # 即使不使用，也保留，因为它从aidemo函数返回
        rec_res = []
        for img_array in imgs_array:
            # 注意：这里需要确保img_array的维度和预期一致，可能是(1, C, H, W)
            # 否则需要调整 input_image_size 的传递方式，例如根据 img_array.shape[2], img_array.shape[3]
            self.licence_rec.config_preprocess(input_image_size=[img_array.shape[3], img_array.shape[2]])
            licence_str = self.licence_rec.run(img_array)
            rec_res.append(licence_str)
            gc.collect()
        return det_boxes, rec_res

    def draw_result(self, pl, det_res, rec_res):
        pl.osd_img.clear()
        if det_res:
            point_8 = np.zeros((8), dtype=np.int16)
            for det_index in range(len(det_res)):
                # 将检测框坐标从AI处理分辨率缩放到实际显示分辨率
                for i in range(4):
                    x = det_res[det_index][i * 2 + 0] / self.rgb888p_size[0] * self.display_size[0]
                    y = det_res[det_index][i * 2 + 1] / self.rgb888p_size[1] * self.display_size[1]
                    point_8[i * 2 + 0] = int(x)
                    point_8[i * 2 + 1] = int(y)
                for i in range(4):
                    pl.osd_img.draw_line(point_8[i * 2 + 0], point_8[i * 2 + 1],
                                         point_8[(i+1) % 4 * 2 + 0], point_8[(i+1) % 4 * 2 + 1],
                                         color=(255, 0, 255, 0), thickness=4)
                # 绘制识别结果文本，调整位置使其可见
                # 假设文本放置在车牌框的左下角附近
                text_x = point_8[6] # 使用第三个点的x坐标
                text_y = point_8[7] + 20 # 在第三个点的y坐标下方一点
                pl.osd_img.draw_string_advanced(text_x, text_y, 40, # 字体大小40
                                               rec_res[det_index], color=(255, 255, 153, 18))


if __name__ == "__main__":
    # 配置常量
    rgb888p_size = [640, 640] # 传感器捕获并传递给AI模型的原始图像分辨率

    licence_det_kmodel_path = "/sdcard/examples/kmodel/LPD_640.kmodel"
    licence_rec_kmodel_path = "/sdcard/examples/kmodel/licence_reco.kmodel"
    licence_det_input_size = [640, 640]
    licence_rec_input_size = [220, 32]
    confidence_threshold = 0.2
    nms_threshold = 0.2

    # 1. 初始化显示设备并获取实际显示分辨率
    actual_display_width, actual_display_height = init_display(select_display)
    # 将实际显示分辨率的宽度进行16字节对齐，以符合硬件要求
    actual_display_size = [ALIGN_UP(actual_display_width, 16), actual_display_height]

    # 2. 根据select_display选择对应的PipeLine显示模式字符串
    display_mode_map_str = {1: "hdmi", 2: "lcd", 3: "virt"}
    pipeline_display_mode_str = display_mode_map_str.get(select_display, "hdmi")

    # 3. 初始化PipeLine，传入传感器输入分辨率和PipeLine的显示模式字符串
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=pipeline_display_mode_str)
    pl.create()

    # 4. 初始化LicenceRec，传入传感器输入分辨率和实际的显示分辨率
    lr = LicenceRec(licence_det_kmodel_path, licence_rec_kmodel_path,
                    det_input_size=licence_det_input_size,
                    rec_input_size=licence_rec_input_size,
                    confidence_threshold=confidence_threshold,
                    nms_threshold=nms_threshold,
                    rgb888p_size=rgb888p_size,
                    display_size=actual_display_size, # 使用实际的显示分辨率
                    debug_mode=0)

    try:
        while True:
            with ScopedTiming("total", 1):
                img = pl.get_frame()    # 获取当前帧
                det_res, rec_res = lr.run(img) # 推理当前帧
                lr.draw_result(pl, det_res, rec_res) # 绘制推理结果
                pl.show_image() # 展示推理效果
                gc.collect()
    except KeyboardInterrupt:
        print("用户中断程序")
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        # 确保在程序结束时释放所有资源
        lr.licence_det.deinit()
        lr.licence_rec.deinit()
        pl.destroy()
        deinit_display() # 释放显示资源
