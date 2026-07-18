from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os,sys,ujson,gc,math
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aicube

# ==================================================================
# == 请在这里选择显示模式 (1: HDMI, 2: LCD, 3: IDE 缓冲区) ==
# ==================================================================
select_display = 1

# 自定义OCR检测类
class OCRDetectionApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,mask_threshold=0.5,box_threshold=0.2,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.mask_threshold=mask_threshold
        self.box_threshold=box_threshold
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            top,bottom,left,right,_=letterbox_pad_param(self.rgb888p_size,self.model_input_size)
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [0,0,0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            hwc_array=self.chw2hwc(self.cur_img)
            det_boxes = aicube.ocr_post_process(results[0][:,:,:,0].reshape(-1), hwc_array.reshape(-1),self.model_input_size,self.rgb888p_size, self.mask_threshold, self.box_threshold)
            return det_boxes

    def chw2hwc(self,features):
        ori_shape = (features.shape[0], features.shape[1], features.shape[2])
        c_hw_ = features.reshape((ori_shape[0], ori_shape[1] * ori_shape[2]))
        hw_c_ = c_hw_.transpose()
        new_array = hw_c_.copy()
        hwc_array = new_array.reshape((ori_shape[1], ori_shape[2], ori_shape[0]))
        del c_hw_
        del hw_c_
        del new_array
        return hwc_array


def draw_detection_result(pl, det_res, rgb888p_size, display_size):
    pl.osd_img.clear()
    if not det_res:
        return

    for det in det_res:
        # aicube.ocr_post_process 返回的 det 结构是 [裁剪的图像, 坐标列表]
        # 我们只需要坐标列表 det[1] 来绘制边框
        box_coords = det[1]
        for i in range(4):
            x1 = box_coords[(i * 2)] / rgb888p_size[0] * display_size[0]
            y1 = box_coords[(i * 2 + 1)] / rgb888p_size[1] * display_size[1]
            x2 = box_coords[((i + 1) * 2) % 8] / rgb888p_size[0] * display_size[0]
            y2 = box_coords[((i + 1) * 2 + 1) % 8] / rgb888p_size[1] * display_size[1]
            # 绘制红色的检测框
            pl.osd_img.draw_line((int(x1), int(y1), int(x2), int(y2)), color=(255, 0, 0, 255), thickness=5)


if __name__=="__main__":
    # === 参数配置 ===
    ocr_det_kmodel_path="/sdcard/examples/kmodel/ocr_det_int16.kmodel"

    # AI处理和模型输入分辨率
    rgb888p_size=[640,360]
    ocr_det_input_size=[640,640]

    # 检测阈值
    mask_threshold=0.25
    box_threshold=0.3

    # === 显示模式设置 ===
    if select_display == 1:
        display_mode = "hdmi"
        print("显示模式: HDMI")
    elif select_display == 2:
        display_mode = "lcd"
        print("显示模式: LCD")
    elif select_display == 3:
        display_mode = "hdmi"
        print("显示模式: IDE 缓冲区 (已回退到 HDMI)")
    else:
        raise ValueError("无效的显示模式选择，请在 1, 2, 3 中选择。")

    # === 初始化流程 ===
    # 初始化PipeLine，它负责管理摄像头和显示
    pl=PipeLine(rgb888p_size=rgb888p_size,display_mode=display_mode)
    pl.create()
    display_size=pl.get_display_size()

    # 初始化OCR检测器 (不再需要总调度器 OCRDetRec)
    ocr_det = OCRDetectionApp(ocr_det_kmodel_path, model_input_size=ocr_det_input_size, mask_threshold=mask_threshold, box_threshold=box_threshold, rgb888p_size=rgb888p_size, display_size=display_size)
    ocr_det.config_preprocess()

    # === 主循环 ===
    try:
        while True:
            with ScopedTiming("total",1):
                # 1. 获取摄像头当前帧
                img = pl.get_frame()

                # 2. 对当前帧进行文本检测 (不再有识别步骤)
                det_results = ocr_det.run(img)

                # 3. 绘制检测结果 (只绘制红框)
                draw_detection_result(pl, det_results, rgb888p_size, display_size)

                # 4. 展示最终画面
                pl.show_image()

                gc.collect()
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        # === 资源释放 ===
        ocr_det.deinit()
        pl.destroy()
        print("程序已退出，资源已释放。")
