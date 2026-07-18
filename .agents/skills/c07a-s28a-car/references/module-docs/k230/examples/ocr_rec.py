# ==================================================================
# == 请在这里选择显示模式 (1: HDMI, 2: LCD, 3: IDE 缓冲区) ==
# ==================================================================
select_display = 3

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

# 自定义OCR识别任务类
class OCRRecognitionApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,dict_path,rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.dict_path=dict_path
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.dict_word=None
        self.read_dict()
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.RGB_packed,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    def config_preprocess(self,input_image_size=None,input_np=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            top,bottom,left,right,_=letterbox_pad_param(ai2d_input_size,self.model_input_size)
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [0,0,0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([input_np.shape[0],input_np.shape[1],input_np.shape[2],input_np.shape[3]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            preds = np.argmax(results[0], axis=2).reshape((-1))
            output_txt = ""
            for i in range(len(preds)):
                if preds[i] != (len(self.dict_word) - 1) and (not (i > 0 and preds[i - 1] == preds[i])):
                    output_txt = output_txt + self.dict_word[preds[i]]
            return output_txt

    def read_dict(self):
        if self.dict_path!="":
            # In CanMV, the 'gb2312' encoding might be necessary for Chinese characters
            with open(self.dict_path, 'r', encoding='gb2312') as file:
                line_one = file.read(100000)
                line_list = line_one.split("\n")
            self.dict_word = {num: char.strip() for num, char in enumerate(line_list)}


class OCRDetRec:
    def __init__(self,ocr_det_kmodel,ocr_rec_kmodel,det_input_size,rec_input_size,dict_path,mask_threshold=0.25,box_threshold=0.3,rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        self.ocr_det_kmodel=ocr_det_kmodel
        self.ocr_rec_kmodel=ocr_rec_kmodel
        self.det_input_size=det_input_size
        self.rec_input_size=rec_input_size
        self.dict_path=dict_path
        self.mask_threshold=mask_threshold
        self.box_threshold=box_threshold
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ocr_det=OCRDetectionApp(self.ocr_det_kmodel,model_input_size=self.det_input_size,mask_threshold=self.mask_threshold,box_threshold=self.box_threshold,rgb888p_size=self.rgb888p_size,display_size=self.display_size,debug_mode=0)
        self.ocr_rec=OCRRecognitionApp(self.ocr_rec_kmodel,model_input_size=self.rec_input_size,dict_path=self.dict_path,rgb888p_size=self.rgb888p_size,display_size=self.display_size)
        self.ocr_det.config_preprocess()

    def run(self,input_np):
        det_res=self.ocr_det.run(input_np)
        boxes=[]
        ocr_res=[]
        for det in det_res:
            self.ocr_rec.config_preprocess(input_image_size=[det[0].shape[2],det[0].shape[1]],input_np=det[0])
            ocr_str=self.ocr_rec.run(det[0])
            ocr_res.append(ocr_str)
            boxes.append(det[1])
            gc.collect()
        return boxes,ocr_res

    def draw_result(self,pl,det_res,rec_res):
        pl.osd_img.clear()
        if det_res:
            for j in range(len(det_res)):
                # Draw bounding box
                for i in range(4):
                    x1 = det_res[j][(i * 2)] / self.rgb888p_size[0] * self.display_size[0]
                    y1 = det_res[j][(i * 2 + 1)] / self.rgb888p_size[1] * self.display_size[1]
                    x2 = det_res[j][((i + 1) * 2) % 8] / self.rgb888p_size[0] * self.display_size[0]
                    y2 = det_res[j][((i + 1) * 2 + 1) % 8] / self.rgb888p_size[1] * self.display_size[1]
                    pl.osd_img.draw_line((int(x1), int(y1), int(x2), int(y2)), color=(255, 0, 0, 255),thickness=5)

                # Prepare text coordinates, ensuring they are on-screen
                text_x = det_res[j][0] / self.rgb888p_size[0] * self.display_size[0]
                text_y = det_res[j][1] / self.rgb888p_size[1] * self.display_size[1] - 32
                if text_y < 0: text_y = 0

                # Draw recognized text
                pl.osd_img.draw_string_advanced(int(text_x),int(text_y),32,rec_res[j],color=(0,0,255))


if __name__=="__main__":
    # AI model paths
    ocr_det_kmodel_path="/sdcard/examples/kmodel/ocr_det_int16.kmodel"
    ocr_rec_kmodel_path="/sdcard/examples/kmodel/ocr_rec_int16.kmodel"
    dict_path="/sdcard/examples/utils/dict.txt"

    # AI model parameters
    rgb888p_size=[640,360]
    ocr_det_input_size=[640,640]
    ocr_rec_input_size=[512,32]
    mask_threshold=0.25
    box_threshold=0.3

    # 1. Set display mode based on the selection at the top
    if select_display == 1:
        display_mode = "hdmi"
        print("Display Mode: HDMI")
    elif select_display == 2:
        display_mode = "lcd"
        print("Display Mode: LCD")
    elif select_display == 3:
        display_mode = "ide" # Fallback to HDMI
        print("Display Mode: IDE Buffer (Fallback to HDMI)")
    else:
        raise ValueError("Invalid display mode. Please choose 1, 2, or 3.")

    # 2. Initialize the media pipeline
    pl=PipeLine(rgb888p_size=rgb888p_size,display_mode=display_mode)
    pl.create()
    display_size=pl.get_display_size()

    # 3. Initialize the OCR main class
    ocr=OCRDetRec(ocr_det_kmodel_path,ocr_rec_kmodel_path,det_input_size=ocr_det_input_size,rec_input_size=ocr_rec_input_size,dict_path=dict_path,mask_threshold=mask_threshold,box_threshold=box_threshold,rgb888p_size=rgb888p_size,display_size=display_size)

    # 4. Start the main loop
    try:
        while True:
            with ScopedTiming("total",1):
                img=pl.get_frame()
                boxes, rec_results = ocr.run(img)
                ocr.draw_result(pl, boxes, rec_results)
                pl.show_image()
                gc.collect()
    except Exception as e:
        print(f"An exception occurred: {e}")
    finally:
        ocr.ocr_det.deinit()
        ocr.ocr_rec.deinit()
        pl.destroy()
        print("Program exited and resources released.")

