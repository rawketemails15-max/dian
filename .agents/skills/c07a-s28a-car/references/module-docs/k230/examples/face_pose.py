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
# == 选择显示模式 (1: HDMI, 2: LCD, 3: IDE虚拟显示) ==
# ==================================================================
select_display = 1  # 1=HDMI，2=LCD，3=IDE虚拟显示，

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
        raise ValueError("select_display参数错误，应为 1 2 或 3")

def deinit_display():
    Display.deinit()
    print("释放显示资源")

class FaceDetApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,anchors,confidence_threshold=0.25,nms_threshold=0.3,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.anchors=anchors
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8,np.uint8)

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            top,bottom,left,right,_=letterbox_pad_param(self.rgb888p_size,self.model_input_size)
            self.ai2d.pad([0,0,0,0,top,bottom,left,right],0,[104,117,123])
            self.ai2d.resize(nn.interp_method.tf_bilinear,nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            res=aidemo.face_det_post_process(self.confidence_threshold,self.nms_threshold,self.model_input_size[0],self.anchors,self.rgb888p_size,results)
            if len(res)==0:
                return res
            else:
                return res[0]

class FacePoseApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.model_input_size=model_input_size
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8,np.uint8)

    def config_preprocess(self,det,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            matrix_dst = self.get_affine_matrix(det)
            self.ai2d.affine(nn.interp_method.cv2_bilinear,0,0,127,1,matrix_dst)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            R,eular = self.get_euler(results[0][0])
            return R,eular

    def get_affine_matrix(self,bbox):
        with ScopedTiming("get_affine_matrix", self.debug_mode > 1):
            factor = 2.7
            x1,y1,w,h = map(lambda x: int(round(x,0)),bbox[:4])
            edge_size = self.model_input_size[1]
            trans_distance = edge_size/2.0
            center_x = x1+w/2.0
            center_y = y1+h/2.0
            maximum_edge = factor*(h if h > w else w)
            scale = edge_size*2.0/maximum_edge
            cx = trans_distance - scale*center_x
            cy = trans_distance - scale*center_y
            affine_matrix = [scale,0,cx,0,scale,cy]
            return affine_matrix

    def rotation_matrix_to_euler_angles(self,R):
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        if sy < 1e-6:
            pitch = np.arctan2(-R[1, 2], R[1, 1]) * 180 / np.pi
            yaw = np.arctan2(-R[2, 0], sy) * 180 / np.pi
            roll = 0
        else:
            pitch = np.arctan2(R[2, 1], R[2, 2]) * 180 / np.pi
            yaw = np.arctan2(-R[2, 0], sy) * 180 / np.pi
            roll = np.arctan2(R[1, 0], R[0, 0]) * 180 / np.pi
        return [pitch,yaw,roll]

    def get_euler(self,data):
        R = data[:3,:3].copy()
        eular = self.rotation_matrix_to_euler_angles(R)
        return R,eular

class FacePose:
    def __init__(self,face_det_kmodel,face_pose_kmodel,det_input_size,pose_input_size,anchors,confidence_threshold=0.25,nms_threshold=0.3,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        self.face_det_kmodel=face_det_kmodel
        self.face_pose_kmodel=face_pose_kmodel
        self.det_input_size=det_input_size
        self.pose_input_size=pose_input_size
        self.anchors=anchors
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.face_det=FaceDetApp(self.face_det_kmodel,model_input_size=self.det_input_size,anchors=self.anchors,confidence_threshold=self.confidence_threshold,nms_threshold=self.nms_threshold,rgb888p_size=self.rgb888p_size,display_size=self.display_size,debug_mode=debug_mode)
        self.face_pose=FacePoseApp(self.face_pose_kmodel,model_input_size=self.pose_input_size,rgb888p_size=self.rgb888p_size,display_size=self.display_size)
        self.face_det.config_preprocess()

    def run(self,input_np):
        det_boxes=self.face_det.run(input_np)
        pose_res=[]
        for det_box in det_boxes:
            self.face_pose.config_preprocess(det_box)
            R,eular=self.face_pose.run(input_np)
            pose_res.append((R,eular))
        return det_boxes,pose_res

    def draw_result(self,pl,dets,pose_res):
        pl.osd_img.clear()
        if dets:
            draw_img_np = np.zeros((self.display_size[1],self.display_size[0],4),dtype=np.uint8)
            draw_img=image.Image(self.display_size[0], self.display_size[1], image.ARGB8888,alloc=image.ALLOC_REF,data=draw_img_np)
            line_color = np.array([255, 0, 0 ,255],dtype=np.uint8)    #bgra
            for i,det in enumerate(dets):
                projections,center_point = self.build_projection_matrix(det)
                R,euler = pose_res[i]
                first_points = []
                second_points = []
                for pp in range(8):
                    sum_x, sum_y = 0.0,0.0
                    for cc in range(3):
                        sum_x += projections[pp][cc] * R[cc][0]
                        sum_y += projections[pp][cc] * (-R[cc][1])
                    center_x, center_y = center_point[0], center_point[1]
                    x = (sum_x + center_x) / self.rgb888p_size[0] * self.display_size[0]
                    y = (sum_y + center_y) / self.rgb888p_size[1] * self.display_size[1]
                    x = max(0,min(x,self.display_size[0]))
                    y = max(0,min(y,self.display_size[1]))
                    if pp < 4:
                        first_points.append((x,y))
                    else:
                        second_points.append((x,y))
                first_points = np.array(first_points,dtype=np.float)
                aidemo.polylines(draw_img_np,first_points,True,line_color,2,8,0)
                second_points = np.array(second_points,dtype=np.float)
                aidemo.polylines(draw_img_np,second_points,True,line_color,2,8,0)
                for ll in range(4):
                    x0, y0 = int(first_points[ll][0]), int(first_points[ll][1])
                    x1, y1 = int(second_points[ll][0]), int(second_points[ll][1])
                    draw_img.draw_line(x0,y0,x1,y1,color=(255,0,0,255),thickness=2)
            pl.osd_img.copy_from(draw_img)

    def build_projection_matrix(self,det):
        x1, y1, w, h = map(lambda x: int(round(x,0)), det[:4])
        center_x = x1 + w/2.0
        center_y = y1 + h/2.0
        rear_width = 0.5 * w
        rear_height = 0.5 * h
        rear_depth = 0
        factor = np.sqrt(2.0)
        front_width = factor * rear_width
        front_height = factor * rear_height
        front_depth = factor * rear_width
        temp = [
            [-rear_width, -rear_height, rear_depth],
            [-rear_width, rear_height, rear_depth],
            [rear_width, rear_height, rear_depth],
            [rear_width, -rear_height, rear_depth],
            [-front_width, -front_height, front_depth],
            [-front_width, front_height, front_depth],
            [front_width, front_height, front_depth],
            [front_width, -front_height, front_depth]
        ]
        projections = np.array(temp)
        return projections, (center_x, center_y)


if __name__=="__main__":
    display_mode_map = {1:'hdmi', 2:'lcd', 3:'ide'}
    display_mode = display_mode_map.get(select_display, 'hdmi')

    if display_mode == 'hdmi':
        display_size = [1920,1080]
    elif display_mode == 'lcd':
        display_size = [800,480]
    else:
        display_size = [1280,720]

    rgb888p_size = [1280,720]

    face_det_kmodel_path = "/sdcard/examples/kmodel/face_detection_320.kmodel"
    face_pose_kmodel_path = "/sdcard/examples/kmodel/face_pose.kmodel"
    anchors_path = "/sdcard/examples/utils/prior_data_320.bin"

    face_det_input_size = [320,320]
    face_pose_input_size = [120,120]
    confidence_threshold = 0.5
    nms_threshold = 0.2
    anchor_len = 4200
    det_dim = 4

    anchors = np.fromfile(anchors_path, dtype=np.float)
    anchors = anchors.reshape((anchor_len, det_dim))

    init_display(select_display,width=display_size[0],height=display_size[1])

    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()

    fp = FacePose(face_det_kmodel_path,face_pose_kmodel_path,det_input_size=face_det_input_size,pose_input_size=face_pose_input_size,anchors=anchors,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,rgb888p_size=rgb888p_size,display_size=display_size)

    try:
        while True:
            with ScopedTiming("total",1):
                img = pl.get_frame()
                det_boxes,pose_res = fp.run(img)
                fp.draw_result(pl,det_boxes,pose_res)
                pl.show_image()
                gc.collect()
    except KeyboardInterrupt:
        print("用户终止程序")

    finally:
        fp.face_det.deinit()
        fp.face_pose.deinit()
        pl.destroy()
        deinit_display()
