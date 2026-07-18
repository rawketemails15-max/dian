from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math, time
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aicube
from media.media import *
from media.display import *
select_display = 3

def init_display(select_display):
    if select_display == 1:
        width, height = 640, 480
        Display.init(Display.LT9611, width=width, height=height, to_ide=True)
        print(f"初始化HDMI显示，分辨率：{width}x{height}")
    elif select_display == 2:
        width, height = 800, 480
        Display.init(Display.ST7701, width=width, height=height, to_ide=True)
        print(f"初始化LCD显示，分辨率：{width}x{height}")
    elif select_display == 3:
        width, height = 800, 480
        Display.init(Display.VIRT, width=width, height=height, fps=100, to_ide=True)
        print(f"初始化IDE虚拟显示，分辨率：{width}x{height}")
    else:
        raise ValueError("select_display参数错误，只能是1、2或3")
    return width, height

def deinit_display():
    Display.deinit()
    print("释放显示资源")

class HandDetApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,anchors,confidence_threshold=0.2,nms_threshold=0.5,nms_option=False,strides=[8,16,32],rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.labels=labels
        self.model_input_size=model_input_size
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.anchors=anchors
        self.strides = strides
        self.nms_option = nms_option
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right,_ = center_pad_param(self.rgb888p_size,self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            dets = aicube.anchorbasedet_post_process(results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, len(self.labels), self.confidence_threshold, self.nms_threshold, self.anchors, self.nms_option)
            return dets

class HandKPClassApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.crop_params=[]
        self.debug_mode=debug_mode
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    def config_preprocess(self,det,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.crop_params = self.get_crop_param(det)
            self.ai2d.crop(self.crop_params[0],self.crop_params[1],self.crop_params[2],self.crop_params[3])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            results=results[0].reshape(results[0].shape[0]*results[0].shape[1])
            results_show = np.zeros(results.shape,dtype=np.int16)
            results_show[0::2] = results[0::2] * self.crop_params[3] + self.crop_params[0]
            results_show[1::2] = results[1::2] * self.crop_params[2] + self.crop_params[1]
            gesture=self.hk_gesture(results_show)
            return results_show,gesture

    def get_crop_param(self,det_box):
        x1, y1, x2, y2 = det_box[2],det_box[3],det_box[4],det_box[5]
        w,h= int(x2 - x1),int(y2 - y1)
        w_det = int(float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0])
        h_det = int(float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1])
        x_det = int(x1*self.display_size[0] // self.rgb888p_size[0])
        y_det = int(y1*self.display_size[1] // self.rgb888p_size[1])
        length = max(w, h)/2
        cx = (x1+x2)/2
        cy = (y1+y2)/2
        ratio_num = 1.26*length
        x1_kp = int(max(0,cx-ratio_num))
        y1_kp = int(max(0,cy-ratio_num))
        x2_kp = int(min(self.rgb888p_size[0]-1, cx+ratio_num))
        y2_kp = int(min(self.rgb888p_size[1]-1, cy+ratio_num))
        w_kp = int(x2_kp - x1_kp + 1)
        h_kp = int(y2_kp - y1_kp + 1)
        return [x1_kp, y1_kp, w_kp, h_kp]

    def hk_vector_2d_angle(self,v1,v2):
        with ScopedTiming("hk_vector_2d_angle",self.debug_mode > 0):
            v1_x,v1_y,v2_x,v2_y = v1[0],v1[1],v2[0],v2[1]
            v1_norm = np.sqrt(v1_x * v1_x+ v1_y * v1_y)
            v2_norm = np.sqrt(v2_x * v2_x + v2_y * v2_y)
            dot_product = v1_x * v2_x + v1_y * v2_y
            cos_angle = dot_product/(v1_norm*v2_norm)
            angle = np.acos(cos_angle)*180/np.pi
            return angle

    def hk_gesture(self,results):
        with ScopedTiming("hk_gesture",self.debug_mode > 0):
            angle_list = []
            for i in range(5):
                angle = self.hk_vector_2d_angle([(results[0]-results[i*8+4]), (results[1]-results[i*8+5])],[(results[i*8+6]-results[i*8+8]),(results[i*8+7]-results[i*8+9])])
                angle_list.append(angle)
            thr_angle,thr_angle_thumb,thr_angle_s,gesture_str = 65.,53.,49.,None
            if 65535. not in angle_list:
                if (angle_list[0]>thr_angle_thumb)  and (angle_list[1]>thr_angle) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "fist"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]<thr_angle_s) and (angle_list[4]<thr_angle_s):
                    gesture_str = "five"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "gun"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]<thr_angle_s):
                    gesture_str = "love"
                elif (angle_list[0]>5)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "one"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]>thr_angle) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]<thr_angle_s):
                    gesture_str = "six"
                elif (angle_list[0]>thr_angle_thumb)  and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]<thr_angle_s) and (angle_list[4]>thr_angle):
                    gesture_str = "three"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]>thr_angle) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "thumbUp"
                elif (angle_list[0]>thr_angle_thumb)  and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "yeah"
            return gesture_str

class DynamicGestureApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ai2d_resize=Ai2d(debug_mode)
        self.ai2d_resize.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.ai2d_crop=Ai2d(debug_mode)
        self.ai2d_crop.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.input_tensors=[]
        self.gesture_kmodel_input_shape = [[1, 3, 224, 224], [1,3,56,56], [1,4,28,28], [1,4,28,28], [1,8,14,14], [1,8,14,14], [1,8,14,14], [1,12,14,14], [1,12,14,14], [1,20,7,7], [1,20,7,7]]
        self.resize_shape = 256
        self.mean_values = np.array([0.485, 0.456, 0.406]).reshape((3,1,1))
        self.std_values = np.array([0.229, 0.224, 0.225]).reshape((3,1,1))
        self.first_data=None
        self.max_hist_len=20
        self.crop_params=self.get_crop_param()

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d_resize.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d_resize.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.crop_params[1],self.crop_params[0]])
            self.ai2d_crop.crop(self.crop_params[2],self.crop_params[3],self.crop_params[4],self.crop_params[5])
            self.ai2d_crop.build([1,3,self.crop_params[1],self.crop_params[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])
            inputs_num=self.get_kmodel_inputs_num()
            self.first_data = np.ones(self.gesture_kmodel_input_shape[0], dtype=np.float)
            for i in range(inputs_num):
                data = np.zeros(self.gesture_kmodel_input_shape[i], dtype=np.float)
                self.input_tensors.append(nn.from_numpy(data))

    def preprocess(self,input_np):
        resize_tensor=self.ai2d_resize.run(input_np)
        crop_output_tensor=self.ai2d_crop.run(resize_tensor.to_numpy())
        ai2d_output = crop_output_tensor.to_numpy()
        self.first_data[0] = ai2d_output[0].copy()
        self.first_data[0] = (self.first_data[0]*1.0/255 -self.mean_values)/self.std_values
        self.input_tensors[0]=nn.from_numpy(self.first_data)

    def run(self,input_np,his_logit,history):
        self.preprocess(input_np)
        outputs=self.inference(self.input_tensors)
        outputs_num=self.get_kmodel_outputs_num()
        for i in range(1,outputs_num):
            self.input_tensors[i]=nn.from_numpy(outputs[i])
        return self.postprocess(outputs,his_logit,history)

    def postprocess(self,results,his_logit, history):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            his_logit.append(results[0])
            avg_logit = sum(np.array(his_logit))
            idx_ = np.argmax(avg_logit)
            idx = self.gesture_process_output(idx_, history)
            if (idx_ != idx):
                his_logit_last = his_logit[-1]
                his_logit = []
                his_logit.append(his_logit_last)
            return idx, avg_logit

    def gesture_process_output(self,pred,history):
        if (pred == 7 or pred == 8 or pred == 21 or pred == 22 or pred == 3 ):
            pred = history[-1]
        if (pred == 0 or pred == 4 or pred == 6 or pred == 9 or pred == 14 or pred == 1 or pred == 19 or pred == 20 or pred == 23 or pred == 24) :
            pred = history[-1]
        if (pred == 0) :
            pred = 2
        if (pred != history[-1]) :
            if (len(history)>= 2) :
                if (history[-1] != history[len(history)-2]) :
                    pred = history[-1]
        history.append(pred)
        if (len(history) > self.max_hist_len) :
            history = history[-self.max_hist_len:]
        return history[-1]

    def get_crop_param(self):
        ori_w = self.rgb888p_size[0]
        ori_h = self.rgb888p_size[1]
        width = self.model_input_size[0]
        height = self.model_input_size[1]
        ratiow = float(self.resize_shape) / ori_w
        ratioh = float(self.resize_shape) / ori_h
        if ratiow < ratioh:
            ratio = ratioh
        else:
            ratio = ratiow
        new_w = int(ratio * ori_w)
        new_h = int(ratio * ori_h)
        top = int((new_h-height)/2)
        left = int((new_w-width)/2)
        return new_w,new_h,left,top,width,height

    def deinit(self):
        with ScopedTiming("deinit",self.debug_mode > 0):
            del self.kpu
            del self.ai2d_resize
            del self.ai2d_crop
            self.tensors.clear()
            del self.tensors
            gc.collect()
            nn.shrink_memory_pool()
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)

class DynamicGesture:
    def __init__(self,hand_det_kmodel,hand_kp_kmodel,gesture_kmodel,det_input_size,kp_input_size,gesture_input_size,labels,anchors,confidence_threshold=0.25,nms_threshold=0.3,nms_option=False,strides=[8,16,32],rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        self.hand_det_kmodel=hand_det_kmodel
        self.hand_kp_kmodel=hand_kp_kmodel
        self.gesture_kmodel=gesture_kmodel
        self.det_input_size=det_input_size
        self.kp_input_size=kp_input_size
        self.gesture_input_size=gesture_input_size
        self.labels=labels
        self.anchors=anchors
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.nms_option=nms_option
        self.strides=strides
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.bin_width = 150
        self.bin_height = 216
        shang_argb = np.fromfile("/sdcard/examples/utils/shang.bin", dtype=np.uint8)
        self.shang_argb = shang_argb.reshape((self.bin_height, self.bin_width, 4))
        xia_argb = np.fromfile("/sdcard/examples/utils/xia.bin", dtype=np.uint8)
        self.xia_argb = xia_argb.reshape((self.bin_height, self.bin_width, 4))
        zuo_argb = np.fromfile("/sdcard/examples/utils/zuo.bin", dtype=np.uint8)
        self.zuo_argb = zuo_argb.reshape((self.bin_width, self.bin_height, 4))
        you_argb = np.fromfile("/sdcard/examples/utils/you.bin", dtype=np.uint8)
        self.you_argb = you_argb.reshape((self.bin_width, self.bin_height, 4))
        self.TRIGGER = 0
        self.MIDDLE = 1
        self.UP = 2
        self.DOWN = 3
        self.LEFT = 4
        self.RIGHT = 5
        self.max_hist_len = 20
        self.debug_mode=debug_mode
        self.cur_state = self.TRIGGER
        self.pre_state = self.TRIGGER
        self.draw_state = self.TRIGGER
        self.vec_flag = []
        self.his_logit = []
        self.history = [2]
        self.s_start = time.time_ns()
        self.m_start=None
        self.hand_det=HandDetApp(self.hand_det_kmodel,self.labels,model_input_size=self.det_input_size,anchors=self.anchors,confidence_threshold=self.confidence_threshold,nms_threshold=self.nms_threshold,nms_option=self.nms_option,strides=self.strides,rgb888p_size=self.rgb888p_size,display_size=self.display_size,debug_mode=0)
        self.hand_kp=HandKPClassApp(self.hand_kp_kmodel,model_input_size=self.kp_input_size,rgb888p_size=self.rgb888p_size,display_size=self.display_size)
        self.dg=DynamicGestureApp(self.gesture_kmodel,model_input_size=self.gesture_input_size,rgb888p_size=self.rgb888p_size,display_size=self.display_size)
        self.hand_det.config_preprocess()
        self.dg.config_preprocess()

    def run(self,input_np):
        if self.cur_state == self.TRIGGER:
            det_boxes=self.hand_det.run(input_np)
            boxes=[]
            gesture_res=[]
            for det_box in det_boxes:
                x1, y1, x2, y2 = det_box[2],det_box[3],det_box[4],det_box[5]
                w,h= int(x2 - x1),int(y2 - y1)
                if (h<(0.1*self.rgb888p_size[1])):
                    continue
                if (w<(0.25*self.rgb888p_size[0]) and ((x1<(0.03*self.rgb888p_size[0])) or (x2>(0.97*self.rgb888p_size[0])))):
                    continue
                if (w<(0.15*self.rgb888p_size[0]) and ((x1<(0.01*self.rgb888p_size[0])) or (x2>(0.99*self.rgb888p_size[0])))):
                    continue
                self.hand_kp.config_preprocess(det_box)
                hk_results,gesture_str=self.hand_kp.run(input_np)
                boxes.append(det_box)
                gesture_res.append((hk_results,gesture_str))
            return boxes,gesture_res
        else:
            idx, avg_logit = self.dg.run(input_np, self.his_logit, self.history)
            return idx,avg_logit

    def draw_result(self,pl,output1,output2):
        pl.osd_img.clear()
        draw_img_np = np.zeros((self.display_size[1],self.display_size[0],4),dtype=np.uint8)
        draw_img=image.Image(self.display_size[0], self.display_size[1], image.ARGB8888,alloc=image.ALLOC_REF,data=draw_img_np)
        if self.cur_state == self.TRIGGER:
            for i in range(len(output1)):
                hk_results,gesture=output2[i][0],output2[i][1]
                if ((gesture == "five") or (gesture == "yeah")):
                    v_x = hk_results[24]-hk_results[0]
                    v_y = hk_results[25]-hk_results[1]
                    angle = self.hand_kp.hk_vector_2d_angle([v_x,v_y],[1.0,0.0])
                    if (v_y>0):
                        angle = 360-angle
                    if ((70.0<=angle) and (angle<110.0)):
                        if ((self.pre_state != self.UP) or (self.pre_state != self.MIDDLE)):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag)>10)or(self.pre_state == self.UP) or (self.pre_state == self.MIDDLE) or(self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_height,:self.bin_width,:] = self.shang_argb
                            self.cur_state = self.UP
                    elif ((110.0<=angle) and (angle<225.0)):
                        if (self.pre_state != self.RIGHT):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag)>10)or(self.pre_state == self.RIGHT)or(self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_width,:self.bin_height,:] = self.you_argb
                            self.cur_state = self.RIGHT
                    elif((225.0<=angle) and (angle<315.0)):
                        if (self.pre_state != self.DOWN):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag)>10)or(self.pre_state == self.DOWN)or(self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_height,:self.bin_width,:] = self.xia_argb
                            self.cur_state = self.DOWN
                    else:
                        if (self.pre_state != self.LEFT):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag)>10)or(self.pre_state == self.LEFT)or(self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_width,:self.bin_height,:] = self.zuo_argb
                            self.cur_state = self.LEFT
                    self.m_start = time.time_ns()
            self.his_logit = []
        else:
            idx,avg_logit=output1,output2[0]
            if (self.cur_state == self.UP):
                draw_img_np[:self.bin_height,:self.bin_width,:] = self.shang_argb
                if ((idx==15) or (idx==10)):
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.7) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 4))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.DOWN
                        self.history = [2]
                    self.pre_state = self.UP
                elif ((idx==25)or(idx==26)) :
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.MIDDLE
                        self.history = [2]
                    self.pre_state = self.MIDDLE
                else:
                    self.his_logit.clear()
            elif (self.cur_state == self.RIGHT):
                draw_img_np[:self.bin_width,:self.bin_height,:] = self.you_argb
                if  ((idx==16)or(idx==11)) :
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.RIGHT
                        self.history = [2]
                    self.pre_state = self.RIGHT
                else:
                    self.his_logit.clear()
            elif (self.cur_state == self.DOWN):
                draw_img_np[:self.bin_height,:self.bin_width,:] = self.xia_argb
                if  ((idx==18)or(idx==13)):
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.UP
                        self.history = [2]
                    self.pre_state = self.DOWN
                else:
                    self.his_logit.clear()
            elif (self.cur_state == self.LEFT):
                draw_img_np[:self.bin_width,:self.bin_height,:] = self.zuo_argb
                if ((idx==17)or(idx==12)):
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.LEFT
                        self.history = [2]
                    self.pre_state = self.LEFT
                else:
                    self.his_logit.clear()

            self.elapsed_time = round((time.time_ns() - self.m_start)/1000000)

            if ((self.cur_state != self.TRIGGER) and (self.elapsed_time>2000)):
                self.cur_state = self.TRIGGER
                self.pre_state = self.TRIGGER

        self.elapsed_ms_show = round((time.time_ns()-self.s_start)/1000000)
        if (self.elapsed_ms_show<1000):
            if (self.draw_state == self.UP):
                draw_img.draw_arrow(self.display_size[0]//2,self.display_size[1]//2,self.display_size[0]//2,self.display_size[1]//2-100, (255,170,190,230), thickness=13)
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"向上")
            elif (self.draw_state == self.RIGHT):
                draw_img.draw_arrow(self.display_size[0]//2,self.display_size[1]//2,self.display_size[0]//2-100,self.display_size[1]//2, (255,170,190,230), thickness=13)
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"向左")
            elif (self.draw_state == self.DOWN):
                draw_img.draw_arrow(self.display_size[0]//2,self.display_size[1]//2,self.display_size[0]//2,self.display_size[1]//2+100, (255,170,190,230), thickness=13)
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"向下")
            elif (self.draw_state == self.LEFT):
                draw_img.draw_arrow(self.display_size[0]//2,self.display_size[1]//2,self.display_size[0]//2+100,self.display_size[1]//2, (255,170,190,230), thickness=13)
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"向右")
            elif (self.draw_state == self.MIDDLE):
                draw_img.draw_circle(self.display_size[0]//2,self.display_size[1]//2,100, (255,170,190,230), thickness=2, fill=True)
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"中间")
        else:
            self.draw_state = self.TRIGGER
        pl.osd_img.copy_from(draw_img)

if __name__=="__main__":
    width, height = init_display(select_display)
    display_mode_map = {1:"hdmi", 2:"lcd", 3:"ide"}
    display_mode = display_mode_map.get(select_display, "lcd")
    rgb888p_size = [1920,1080]

    pl=PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size=pl.get_display_size()

    hand_det_kmodel_path="/sdcard/examples/kmodel/hand_det.kmodel"
    hand_kp_kmodel_path="/sdcard/examples/kmodel/handkp_det.kmodel"
    gesture_kmodel_path="/sdcard/examples/kmodel/gesture.kmodel"
    hand_det_input_size=[512,512]
    hand_kp_input_size=[256,256]
    gesture_input_size=[224,224]
    confidence_threshold=0.2
    nms_threshold=0.5
    labels=["hand"]
    anchors=[26,27,53,52,75,71,80,99,106,82,99,134,140,113,161,172,245,276]

    dg=DynamicGesture(
        hand_det_kmodel_path, hand_kp_kmodel_path, gesture_kmodel_path,
        det_input_size=hand_det_input_size, kp_input_size=hand_kp_input_size, gesture_input_size=gesture_input_size,
        labels=labels, anchors=anchors, confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold, nms_option=False, strides=[8,16,32],
        rgb888p_size=rgb888p_size, display_size=display_size
    )

    try:
        while True:
            with ScopedTiming("total",1):
                img=pl.get_frame()
                output1,output2=dg.run(img)
                dg.draw_result(pl,output1,output2)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        pass
    finally:
        dg.hand_det.deinit()
        dg.hand_kp.deinit()
        dg.dg.deinit()
        pl.destroy()
        deinit_display()
