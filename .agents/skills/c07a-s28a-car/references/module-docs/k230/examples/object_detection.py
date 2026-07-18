from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math, time
from media.media import *
from media.display import Display
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo
from machine import Pin
from machine import FPIOA

# ==================================================================
# 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示)
# ==================================================================
select_display = 2  # 1=HDMI, 2=LCD, 3=IDE虚拟显示

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

# ================== TrackCropApp ====================
class TrackCropApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,ratio_src_crop,center_xy_wh,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.CONTEXT_AMOUNT = 0.5
        self.ratio_src_crop = ratio_src_crop
        self.center_xy_wh=center_xy_wh
        self.pad_crop_params=[]
        self.ai2d_pad=Ai2d(debug_mode)
        self.ai2d_pad.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.ai2d_crop=Ai2d(debug_mode)
        self.ai2d_crop.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.need_pad=False

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            self.pad_crop_params= self.get_padding_crop_param()
            if (self.pad_crop_params[0]!=0 or self.pad_crop_params[1]!=0 or self.pad_crop_params[2]!=0 or self.pad_crop_params[3]!=0):
                self.need_pad=True
                self.ai2d_pad.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_pad.pad([0,0,0,0,self.pad_crop_params[0],self.pad_crop_params[1],self.pad_crop_params[2],self.pad_crop_params[3]],0,[114,114,114])
                output_size=[self.rgb888p_size[0]+self.pad_crop_params[2]+self.pad_crop_params[3],
                             self.rgb888p_size[1]+self.pad_crop_params[0]+self.pad_crop_params[1]]
                self.ai2d_pad.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                   [1,3,output_size[1],output_size[0]])

                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(self.pad_crop_params[4]),int(self.pad_crop_params[6]),
                                    int(self.pad_crop_params[5]-self.pad_crop_params[4]+1),
                                    int(self.pad_crop_params[7]-self.pad_crop_params[6]+1))
                self.ai2d_crop.build([1,3,output_size[1],output_size[0]],
                                    [1,3,self.model_input_size[1],self.model_input_size[0]])
            else:
                self.need_pad=False
                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(self.center_xy_wh[0]-self.pad_crop_params[8]/2.0),
                                    int(self.center_xy_wh[1]-self.pad_crop_params[8]/2.0),
                                    int(self.pad_crop_params[8]),
                                    int(self.pad_crop_params[8]))
                self.ai2d_crop.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                    [1,3,self.model_input_size[1],self.model_input_size[0]])

    def preprocess(self,input_np):
        if self.need_pad:
            pad_output=self.ai2d_pad.run(input_np).to_numpy()
            return [self.ai2d_crop.run(pad_output)]
        else:
            return [self.ai2d_crop.run(input_np)]

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            return results[0]

    def get_padding_crop_param(self):
        s_z = round(np.sqrt((self.center_xy_wh[2] + self.CONTEXT_AMOUNT * (self.center_xy_wh[2] + self.center_xy_wh[3])) *
                           (self.center_xy_wh[3] + self.CONTEXT_AMOUNT * (self.center_xy_wh[2] + self.center_xy_wh[3]))))
        c = (s_z + 1) / 2
        context_xmin = np.floor(self.center_xy_wh[0] - c + 0.5)
        context_xmax = int(context_xmin + s_z - 1)
        context_ymin = np.floor(self.center_xy_wh[1] - c + 0.5)
        context_ymax = int(context_ymin + s_z - 1)
        left_pad = int(max(0, -context_xmin))
        top_pad = int(max(0, -context_ymin))
        right_pad = int(max(0, int(context_xmax - self.rgb888p_size[0] + 1)))
        bottom_pad = int(max(0, int(context_ymax - self.rgb888p_size[1] + 1)))
        context_xmin += left_pad
        context_xmax += left_pad
        context_ymin += top_pad
        context_ymax += top_pad
        return [top_pad,bottom_pad,left_pad,right_pad,
                context_xmin,context_xmax,context_ymin,context_ymax,s_z]

    def deinit(self):
        with ScopedTiming("deinit",self.debug_mode > 0):
            del self.ai2d_pad
            del self.ai2d_crop
            super().deinit()

# ================== TrackSrcApp ====================
class TrackSrcApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,ratio_src_crop,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.pad_crop_params=[]
        self.CONTEXT_AMOUNT = 0.5
        self.ratio_src_crop = ratio_src_crop
        self.debug_mode=debug_mode
        self.ai2d_pad=Ai2d(debug_mode)
        self.ai2d_pad.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8,np.uint8)
        self.ai2d_crop=Ai2d(debug_mode)
        self.ai2d_crop.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8,np.uint8)
        self.need_pad=False

    def config_preprocess(self,center_xy_wh,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.pad_crop_params= self.get_padding_crop_param(center_xy_wh)
            if (self.pad_crop_params[0]!=0 or self.pad_crop_params[1]!=0 or self.pad_crop_params[2]!=0 or self.pad_crop_params[3]!=0):
                self.need_pad=True
                self.ai2d_pad.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_pad.pad([0,0,0,0,self.pad_crop_params[0],self.pad_crop_params[1],self.pad_crop_params[2],self.pad_crop_params[3]],0,[114,114,114])
                output_size=[self.rgb888p_size[0]+self.pad_crop_params[2]+self.pad_crop_params[3],
                             self.rgb888p_size[1]+self.pad_crop_params[0]+self.pad_crop_params[1]]

                self.ai2d_pad.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                   [1,3,output_size[1],output_size[0]])
                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(self.pad_crop_params[4]),int(self.pad_crop_params[6]),
                                    int(self.pad_crop_params[5]-self.pad_crop_params[4]+1),
                                    int(self.pad_crop_params[7]-self.pad_crop_params[6]+1))
                self.ai2d_crop.build([1,3,output_size[1],output_size[0]],
                                    [1,3,self.model_input_size[1],self.model_input_size[0]])
            else:
                self.need_pad=False
                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(center_xy_wh[0]-self.pad_crop_params[8]/2.0),
                                    int(center_xy_wh[1]-self.pad_crop_params[8]/2.0),
                                    int(self.pad_crop_params[8]),
                                    int(self.pad_crop_params[8]))
                self.ai2d_crop.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                    [1,3,self.model_input_size[1],self.model_input_size[0]])

    def preprocess(self,input_np):
        with ScopedTiming("preprocess",self.debug_mode>0):
            if self.need_pad:
                pad_output=self.ai2d_pad.run(input_np).to_numpy()
                return [self.ai2d_crop.run(pad_output)]
            else:
                return [self.ai2d_crop.run(input_np)]

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            return results[0]

    def get_padding_crop_param(self,center_xy_wh):
        s_z = round(np.sqrt((center_xy_wh[2] + self.CONTEXT_AMOUNT * (center_xy_wh[2] + center_xy_wh[3])) *
                            (center_xy_wh[3] + self.CONTEXT_AMOUNT * (center_xy_wh[2] + center_xy_wh[3])))) * self.ratio_src_crop
        c = (s_z + 1) / 2
        context_xmin = np.floor(center_xy_wh[0] - c + 0.5)
        context_xmax = int(context_xmin + s_z - 1)
        context_ymin = np.floor(center_xy_wh[1] - c + 0.5)
        context_ymax = int(context_ymin + s_z - 1)
        left_pad = int(max(0, -context_xmin))
        top_pad = int(max(0, -context_ymin))
        right_pad = int(max(0, int(context_xmax - self.rgb888p_size[0] + 1)))
        bottom_pad = int(max(0, int(context_ymax - self.rgb888p_size[1] + 1)))
        context_xmin += left_pad
        context_xmax += left_pad
        context_ymin += top_pad
        context_ymax += top_pad
        return [top_pad,bottom_pad,left_pad,right_pad,
                context_xmin,context_xmax,context_ymin,context_ymax,s_z]

    def deinit(self):
        with ScopedTiming("deinit",self.debug_mode > 0):
            del self.ai2d_pad
            del self.ai2d_crop
            super().deinit()

# ================== TrackerApp ====================
class TrackerApp(AIBase):
    def __init__(self,kmodel_path,crop_input_size,thresh,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.crop_input_size=crop_input_size
        self.thresh=thresh
        self.CONTEXT_AMOUNT = 0.5
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8,np.uint8)

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            pass

    def run(self,input_np_1,input_np_2,center_xy_wh):
        input_tensors=[]
        input_tensors.append(nn.from_numpy(input_np_1))
        input_tensors.append(nn.from_numpy(input_np_2))
        results=self.inference(input_tensors)
        return self.postprocess(results,center_xy_wh)

    def postprocess(self,results,center_xy_wh):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            det = aidemo.nanotracker_postprocess(results[0],results[1],
                                                 [self.rgb888p_size[1],self.rgb888p_size[0]],
                                                 self.thresh,center_xy_wh,self.crop_input_size[0],self.CONTEXT_AMOUNT)
            return det

# ================== NanoTracker ====================
class NanoTracker:
    def __init__(self,track_crop_kmodel,track_src_kmodel,tracker_kmodel,
                 crop_input_size,src_input_size,threshold=0.25,
                 rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        self.track_crop_kmodel=track_crop_kmodel
        self.track_src_kmodel=track_src_kmodel
        self.tracker_kmodel=tracker_kmodel
        self.crop_input_size=crop_input_size
        self.src_input_size=src_input_size
        self.threshold=threshold

        self.CONTEXT_AMOUNT=0.5
        self.ratio_src_crop = 0.0
        self.track_x1 = float(600)
        self.track_y1 = float(300)
        self.track_w = float(100)
        self.track_h = float(100)
        self.draw_mean=[]
        self.center_xy_wh = []
        self.track_boxes = []
        self.center_xy_wh_tmp = []
        self.track_boxes_tmp=[]
        self.crop_output=None
        self.src_output=None

        self.seconds = 8
        self.endtime = None  # 初始化为None，倒计时在外部按键按下后开始
        self.countdown_active = False # 新增标志：倒计时是否正在进行
        self.initial_setup_done = False # 新增标志：首次倒计时设置是否已完成

        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.init_param()

        self.track_crop=TrackCropApp(self.track_crop_kmodel,
                                     model_input_size=self.crop_input_size,
                                     ratio_src_crop=self.ratio_src_crop,
                                     center_xy_wh=self.center_xy_wh,
                                     rgb888p_size=self.rgb888p_size,
                                     display_size=self.display_size,
                                     debug_mode=0)
        self.track_src=TrackSrcApp(self.track_src_kmodel,
                                   model_input_size=self.src_input_size,
                                   ratio_src_crop=self.ratio_src_crop,
                                   rgb888p_size=self.rgb888p_size,
                                   display_size=self.display_size,
                                   debug_mode=0)
        self.tracker=TrackerApp(self.tracker_kmodel,
                                crop_input_size=self.crop_input_size,
                                thresh=self.threshold,
                                rgb888p_size=self.rgb888p_size,
                                display_size=self.display_size)
        # 初始的模板图像处理配置只需要进行一次
        self.track_crop.config_preprocess()

    def run(self,input_np, external_tracking_enabled):
        nowtime = time.time()

        if external_tracking_enabled and not self.initial_setup_done:
            # 用户想要开始追踪，且初始设置（倒计时）尚未完成
            if not self.countdown_active: # 如果倒计时尚未激活，则开始
                self.endtime = nowtime + self.seconds
                self.countdown_active = True
                print(f"倒计时开始 (首次追踪). 当前时间: {nowtime}, 结束时间: {self.endtime}")

            if self.countdown_active and nowtime <= self.endtime:
                # 倒计时进行中
                countdown = int(self.endtime - nowtime)
                # 在倒计时阶段，持续更新初始裁剪图像 (模板)
                self.crop_output=self.track_crop.run(input_np)
                return self.draw_mean, countdown # 返回初始框和倒计时
            elif self.countdown_active and nowtime > self.endtime:
                # 倒计时刚刚结束
                print("倒计时结束，开始追踪。")
                self.countdown_active = False
                self.initial_setup_done = True # 标记初始设置已完成
                # 倒计时结束后，立即进行一次追踪以更新状态
                self.track_src.config_preprocess(self.center_xy_wh)
                self.src_output=self.track_src.run(input_np)
                det=self.tracker.run(self.crop_output,self.src_output,self.center_xy_wh)
                return det, None
        elif self.initial_setup_done and external_tracking_enabled:
            # 初始设置已完成，且外部追踪开关为True (正常追踪模式)
            self.track_src.config_preprocess(self.center_xy_wh)
            self.src_output=self.track_src.run(input_np)
            det=self.tracker.run(self.crop_output,self.src_output,self.center_xy_wh)
            return det, None
        else:
            # 外部追踪开关未开启，或者初始设置未完成且外部追踪未开启（等待按键开始状态）
            # 在这些情况下，只显示初始框，不显示倒计时
            return self.draw_mean, None

    def draw_result(self,pl,box,countdown=None):
        pl.osd_img.clear()
        if not self.initial_setup_done: # 如果初始设置（倒计时）尚未完成
            pl.osd_img.draw_rectangle(box[0],box[1],box[2],box[3],color=(255,0,255,0),thickness=4)
            if countdown is not None: # 仅当倒计时激活时才显示倒计时文本
                pl.osd_img.draw_string_advanced(50, 50, 50, f"倒计时: {countdown} 秒", color=(255, 255, 0, 0))
        else: # 初始设置已完成，进入追踪模式
            self.track_boxes = box[0]
            self.center_xy_wh = box[1]
            track_bool = True
            if (len(self.track_boxes) != 0):
                track_bool = (self.track_boxes[0] > 10 and self.track_boxes[1] >10 and
                              self.track_boxes[0]+self.track_boxes[2]<self.rgb888p_size[0]-10 and
                              self.track_boxes[1]+self.track_boxes[3]<self.rgb888p_size[1]-10)
            else:
                track_bool = False

            if (len(self.center_xy_wh) != 0):
                track_bool = track_bool and self.center_xy_wh[2]*self.center_xy_wh[3]<40000
            else:
                track_bool = False

            if (track_bool):
                self.center_xy_wh_tmp = self.center_xy_wh
                self.track_boxes_tmp = self.track_boxes
                x1 = int(self.track_boxes[0]*self.display_size[0]/self.rgb888p_size[0])
                y1 = int(self.track_boxes[1]*self.display_size[1]/self.rgb888p_size[1])
                w = int(self.track_boxes[2]*self.display_size[0]/self.rgb888p_size[0])
                h = int(self.track_boxes[3]*self.display_size[1]/self.rgb888p_size[1])
                pl.osd_img.draw_rectangle(x1,y1,w,h,color=(255,255,0,0),thickness=4)
            else:
                self.center_xy_wh = self.center_xy_wh_tmp
                self.track_boxes = self.track_boxes_tmp
                x1 = int(self.track_boxes[0]*self.display_size[0]/self.rgb888p_size[0])
                y1 = int(self.track_boxes[1]*self.display_size[1]/self.rgb888p_size[1])
                w = int(self.track_boxes[2]*self.display_size[0]/self.rgb888p_size[0])
                h = int(self.track_boxes[3]*self.display_size[1]/self.rgb888p_size[1])
                pl.osd_img.draw_rectangle(x1,y1,w,h,color=(255,255,0,0),thickness=4)
                pl.osd_img.draw_string_advanced(x1,y1-50,32,"请远离摄像头，保持跟踪物体大小基本一致!",color=(255,255,0,0))
                pl.osd_img.draw_string_advanced(x1,y1-100,32,"请靠近中心!",color=(255,255,0,0))

    def init_param(self):
        self.ratio_src_crop = float(self.src_input_size[0])/float(self.crop_input_size[0])
        print(self.ratio_src_crop)
        if (self.track_x1 < 50 or self.track_y1 < 50 or
            self.track_x1+self.track_w >= self.rgb888p_size[0]-50 or
            self.track_y1+self.track_h >= self.rgb888p_size[1]-50):
            print("**剪切范围超出图像范围**")
        else:
            track_mean_x = self.track_x1 + self.track_w/2.0
            track_mean_y = self.track_y1 + self.track_h/2.0
            draw_mean_w = int(self.track_w/self.rgb888p_size[0]*self.display_size[0])
            draw_mean_h = int(self.track_h/self.rgb888p_size[1]*self.display_size[1])
            draw_mean_x = int(track_mean_x/self.rgb888p_size[0]*self.display_size[0] - draw_mean_w/2.0)
            draw_mean_y = int(track_mean_y/self.rgb888p_size[1]*self.display_size[1] - draw_mean_h/2.0)

            self.draw_mean=[draw_mean_x,draw_mean_y,draw_mean_w,draw_mean_h]
            self.center_xy_wh = [track_mean_x,track_mean_y,self.track_w,self.track_h]
            self.center_xy_wh_tmp = [track_mean_x,track_mean_y,self.track_w,self.track_h]

            self.track_boxes = [self.track_x1,self.track_y1,self.track_w,self.track_h,1]
            self.track_boxes_tmp = np.array([self.track_x1,self.track_y1,self.track_w,self.track_h,1])

# ============================ 主程序入口 =============================
if __name__=="__main__":
    display_mode_map = {1:"hdmi",2:"lcd",3:"ide"}
    display_mode = display_mode_map.get(select_display, "lcd")

    if select_display == 1:
        display_size = [1920,1080]
    elif select_display == 2:
        display_size = [800,480]
    else:
        display_size = [1280,720]

    init_display(select_display, display_size[0], display_size[1])

    rgb888p_size = [1280,720]
    track_crop_kmodel_path = "/sdcard/examples/kmodel/cropped_test127.kmodel"
    track_src_kmodel_path = "/sdcard/examples/kmodel/nanotrack_backbone_sim.kmodel"
    tracker_kmodel_path = "/sdcard/examples/kmodel/nanotracker_head_calib_k230.kmodel"
    track_crop_input_size = [127,127]
    track_src_input_size = [255,255]
    threshold = 0.1

    pl = PipeLine(rgb888p_size=rgb888p_size,display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    track = NanoTracker(track_crop_kmodel_path,
                        track_src_kmodel_path,
                        tracker_kmodel_path,
                        crop_input_size=track_crop_input_size,
                        src_input_size=track_src_input_size,
                        threshold=threshold,
                        rgb888p_size=rgb888p_size,
                        display_size=display_size)

    # 设置按键控制GPIO
    fpioa = FPIOA()
    fpioa.set_function(21, FPIOA.GPIO21)
    KEY = Pin(21, Pin.IN, Pin.PULL_UP)

    last_pressed_time_ms = 0
    debounce_ms = 50
    tracking_enabled = False  # 初始状态：未追踪，未倒计时

    try:
        while True:
            with ScopedTiming("total",1):
                img = pl.get_frame()

                current_time_ms = time.ticks_ms()
                if KEY.value() == 0: # 检测按键是否被按下
                    if time.ticks_diff(current_time_ms, last_pressed_time_ms) > debounce_ms:
                        tracking_enabled = not tracking_enabled # 切换追踪状态
                        mode_str = "开启追踪" if tracking_enabled else "暂停追踪"
                        print(f"按键触发，切换状态: {mode_str}")
                        last_pressed_time_ms = current_time_ms
                        while KEY.value() == 0: # 等待按键释放，防止重复触发
                            time.sleep_ms(10)

                # 将当前的追踪状态传递给 NanoTracker 的 run 方法
                output, countdown = track.run(img, tracking_enabled)

                # 调用 draw_result 方法，它会根据 NanoTracker 的内部状态来绘制相应的内容
                track.draw_result(pl, output, countdown)

                # 仅当追踪未启动或暂停时显示提示信息
                if not tracking_enabled:
                    pl.osd_img.draw_string_advanced(50, 50, 40, "请按下按键开始/暂停追踪", color=(255,255,0,0))

                pl.show_image()
                gc.collect()

    except KeyboardInterrupt:
        print("用户手动终止程序")
    finally:
        track.track_crop.deinit()
        track.track_src.deinit()
        track.tracker.deinit()
        pl.destroy()
        deinit_display()

