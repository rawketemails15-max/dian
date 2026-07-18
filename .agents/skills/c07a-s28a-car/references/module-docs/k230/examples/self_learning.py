from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math, uos
from media.media import *
from media.display import Display
import nncase_runtime as nn
import ulab.numpy as np
import image
import aicube
from machine import Pin
from machine import FPIOA
import time

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

class SelfLearningApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,labels,top_k,threshold,features_save_path,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.labels=labels
        self.features_save_path=features_save_path
        self.database_path=features_save_path+"features/"
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.threshold = threshold
        self.top_k = top_k
        self.features=[2,2]
        self.time_one=60
        self.time_all = 0
        self.time_now = 0
        self.category_index = 0
        self.crop_w = 400
        self.crop_h = 400
        self.crop_x = self.rgb888p_size[0] / 2.0 - self.crop_w / 2.0
        self.crop_y = self.rgb888p_size[1] / 2.0 - self.crop_h / 2.0
        self.crop_x_osd=0
        self.crop_y_osd=0
        self.crop_w_osd=0
        self.crop_h_osd=0
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.learning = False
        self.need_press_to_start = True
        self.data_init()

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.crop(int(self.crop_x),int(self.crop_y),int(self.crop_w),int(self.crop_h))
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            return results[0][0]

    def draw_result(self,pl,feature):
        pl.osd_img.clear()
        with ScopedTiming("display_draw",self.debug_mode >0):
            pl.osd_img.draw_rectangle(self.crop_x_osd,self.crop_y_osd, self.crop_w_osd, self.crop_h_osd, color=(255, 255, 0, 255), thickness = 4)
            if self.need_press_to_start:
                pl.osd_img.draw_string_advanced(50, 50, 40, "请按下按键开始学习", color=(255,255,0,0))
            else:
                if self.learning:
                    if (self.category_index < len(self.labels)):
                        pl.osd_img.draw_string_advanced(50, self.crop_y_osd-50, 30,
                            "请将待添加类别放入框内进行特征采集："+self.labels[self.category_index] + "_" + str(int(self.time_now-1) // self.time_one) + ".bin",
                            color=(255,255,0,0))
                        with open(self.database_path + self.labels[self.category_index] + "_" + str(int(self.time_now-1) // self.time_one) + ".bin", 'wb') as f:
                            f.write(feature.tobytes())
                        self.time_now += 1
                        if (self.time_now // self.time_one == self.features[self.category_index]):
                            self.category_index += 1
                            self.time_all -= self.time_now
                            self.time_now = 0
                            self.learning = False
                    else:
                        self.learning = False
                        pl.osd_img.draw_string_advanced(50, 50, 40, "所有类别采集完成", color=(255,255,0,0))
                else:
                    if self.category_index < len(self.labels):
                        pl.osd_img.draw_string_advanced(50, 50, 40, f"请按下按键开始学习类别：{self.labels[self.category_index]}", color=(255,255,0,0))
                    else:
                        results_learn = []
                        list_features = os.listdir(self.database_path)
                        for feature_name in list_features:
                            with open(self.database_path + feature_name, 'rb') as f:
                                data = f.read()
                            save_vec = np.frombuffer(data, dtype=np.float)
                            score = self.getSimilarity(feature, save_vec)
                            if (score > self.threshold):
                                res = feature_name.split("_")
                                is_same = False
                                for r in results_learn:
                                    if (r["category"] ==  res[0]):
                                        if (r["score"] < score):
                                            r["bin_file"] = feature_name
                                            r["score"] = score
                                        is_same = True
                                if (not is_same):
                                    if(len(results_learn) < self.top_k):
                                        evec = {}
                                        evec["category"] = res[0]
                                        evec["score"] = score
                                        evec["bin_file"] = feature_name
                                        results_learn.append( evec )
                                        results_learn = sorted(results_learn, key=lambda x: -x["score"])
                                    else:
                                        if( score <= results_learn[self.top_k-1]["score"] ):
                                            continue
                                        else:
                                            evec = {}
                                            evec["category"] = res[0]
                                            evec["score"] = score
                                            evec["bin_file"] = feature_name
                                            results_learn.append( evec )
                                            results_learn = sorted(results_learn, key=lambda x: -x["score"])
                                            results_learn.pop()
                        draw_y = 200
                        for r in results_learn:
                            pl.osd_img.draw_string_advanced(50, draw_y, 50, r["category"] + " : " + str(r["score"]), color=(255,255,0,0))
                            draw_y += 50

    def data_init(self):
        files=os.listdir(self.features_save_path)
        if "features" in files:
            features_files=os.listdir(self.database_path)
            for file in features_files:
                os.remove(self.database_path+file)
            os.rmdir(self.database_path)
        os.mkdir(self.database_path)
        self.crop_x_osd = int(self.crop_x / self.rgb888p_size[0] * self.display_size[0])
        self.crop_y_osd = int(self.crop_y / self.rgb888p_size[1] * self.display_size[1])
        self.crop_w_osd = int(self.crop_w / self.rgb888p_size[0] * self.display_size[0])
        self.crop_h_osd = int(self.crop_h / self.rgb888p_size[1] * self.display_size[1])
        for i in range(len(self.labels)):
            for j in range(self.features[i]):
                self.time_all += self.time_one

    def getSimilarity(self,output_vec,save_vec):
        tmp = sum(output_vec * save_vec)
        mold_out = np.sqrt(sum(output_vec * output_vec))
        mold_save = np.sqrt(sum(save_vec * save_vec))
        return tmp / (mold_out * mold_save)

    def start_learning(self):
        if self.category_index < len(self.labels):
            self.learning = True
            self.time_now = 0
            self.need_press_to_start = False

    def next_category(self):
        if self.category_index < len(self.labels):
            self.learning = True
            self.time_now = 0
            self.need_press_to_start = False

if __name__=="__main__":
    rgb888p_size=[1280,720]
    kmodel_path="/sdcard/examples/kmodel/recognition.kmodel"
    features_save_path="/sdcard/examples/utils/"
    model_input_size=[224,224]
    labels=["苹果","香蕉"]
    top_k=3
    threshold=0.5

    display_mode_map = {1:"hdmi", 2:"lcd", 3:"ide"}
    display_mode = display_mode_map.get(select_display, "hdmi")

    if display_mode == "hdmi":
        display_size = [1920,1080]
    elif display_mode == "lcd":
        display_size = [800,480]
    else:
        display_size = [1280,720]

    init_display(select_display, display_size[0], display_size[1])

    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()

    sl = SelfLearningApp(kmodel_path, model_input_size=model_input_size, labels=labels, top_k=top_k,
                         threshold=threshold, features_save_path=features_save_path,
                         rgb888p_size=rgb888p_size, display_size=display_size, debug_mode=0)

    sl.config_preprocess()

    fpioa = FPIOA()
    fpioa.set_function(21, FPIOA.GPIO21)
    KEY = Pin(21, Pin.IN, Pin.PULL_UP)

    last_pressed_time_ms = 0
    debounce_ms = 50

    try:
        while True:
            with ScopedTiming("total",1):
                img=pl.get_frame()
                res=sl.run(img)

                current_time_ms = time.ticks_ms()
                if KEY.value() == 0:
                    if time.ticks_diff(current_time_ms, last_pressed_time_ms) > debounce_ms:
                        if sl.need_press_to_start:
                            sl.start_learning()
                        elif not sl.learning and sl.category_index < len(sl.labels):
                            sl.next_category()
                        last_pressed_time_ms = current_time_ms
                        while KEY.value() == 0:
                            time.sleep_ms(10)

                sl.draw_result(pl,res)
                pl.show_image()
                gc.collect()

    except KeyboardInterrupt:
        print("用户中断程序，退出中...")
    finally:
        if os.path.exists(sl.database_path):
            try:
                features_files = os.listdir(sl.database_path)
                for file in features_files:
                    os.remove(sl.database_path + file)
                os.rmdir(sl.database_path)
            except Exception as e:
                print("清理文件夹错误:", e)
        sl.deinit()
        pl.destroy()
        deinit_display()
