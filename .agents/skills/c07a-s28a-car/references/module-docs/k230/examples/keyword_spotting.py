from libs.PipeLine import ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from media.pyaudio import *                     # 音频模块
from media.media import *                       # 软件抽象模块
import media.wave as wave                       # wav音频处理模块
import nncase_runtime as nn                     # nncase运行模块
import ulab.numpy as np                         # numpy类似操作
import aidemo                                   # aidemo模块
import time                                     # 时间统计
import struct                                   # 字节字符转换模块
import gc                                       # 垃圾回收模块
import os, sys                                  # 操作系统接口模块


# 自定义关键词唤醒类，继承自AIBase基类
class KWSApp(AIBase):
    def __init__(self, kmodel_path, threshold, debug_mode=0):
        super().__init__(kmodel_path)  # 调用基类的构造函数
        self.kmodel_path = kmodel_path  # 模型文件路径
        self.threshold = threshold
        self.debug_mode = debug_mode  # 是否开启调试模式
        self.cache_np = np.zeros((1, 256, 105), dtype=np.float)

    # 自定义预处理，返回模型输入tensor列表
    def preprocess(self, pcm_data):
        pcm_data_list = []
        # 获取音频流数据
        for i in range(0, len(pcm_data), 2):
            # 每两个字节组成一个有符号整数，转为浮点数采样值
            int_pcm_data = struct.unpack("<h", pcm_data[i:i + 2])[0]
            float_pcm_data = float(int_pcm_data)
            pcm_data_list.append(float_pcm_data)
        print("Preprocess first 10 samples:", pcm_data_list[:10])  # 打印前10个采样点，确认音频数据
        # 将pcm数据处理为模型输入的特征向量
        mp_feats = aidemo.kws_preprocess(fp, pcm_data_list)[0]
        mp_feats_np = np.array(mp_feats).reshape((1, 30, 40))
        audio_input_tensor = nn.from_numpy(mp_feats_np)
        cache_input_tensor = nn.from_numpy(self.cache_np)
        return [audio_input_tensor, cache_input_tensor]

    # 自定义当前任务的后处理，results是模型输出array列表
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            logits_np = results[0]
            self.cache_np= results[1]
            max_logits = np.max(logits_np, axis=1)[0]
            max_p = np.max(max_logits)
            idx = np.argmax(max_logits)
            print(f"max_p={max_p}, idx={idx}")   # 打印当前置信度和类别预测
            if max_p > self.threshold and idx == 1:
                return 1
            else:
                return 0



if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    nn.shrink_memory_pool()
    # 设置模型路径和参数
    kmodel_path = "/sdcard/examples/kmodel/kws.kmodel"
    THRESH = 0.05
    SAMPLE_RATE = 16000
    CHANNELS = 1
    FORMAT = paInt16
    CHUNK = int(0.3 * SAMPLE_RATE)   # 0.3秒采样大小，4800点
    reply_wav_file = "/sdcard/examples/utils/wozai.wav"

    # 初始化音频预处理接口
    fp = aidemo.kws_fp_create()
    # 初始化PyAudio对象及MediaManager
    p = PyAudio()
    p.initialize(CHUNK)
    MediaManager.init()

    # 打开音频输入流
    input_stream = p.open(format=FORMAT, channels=CHANNELS, rate=SAMPLE_RATE, input=True, frames_per_buffer=CHUNK)
    input_stream.volume(vol=100)  # 设置麦克风音量最大

    # 打开输出流，用于播放回复音频
    output_stream = p.open(format=FORMAT, channels=CHANNELS, rate=SAMPLE_RATE, output=True, frames_per_buffer=CHUNK)

    # 创建自定义关键词唤醒实例
    kws = KWSApp(kmodel_path, threshold=THRESH, debug_mode=0)

    try:
        while True:
            os.exitpoint()  # 检查程序退出信号
            with ScopedTiming("total", 1):
                pcm_data = input_stream.read()
                print(f"Captured audio length: {len(pcm_data)} bytes")   # 打印采集数据长度
                print(f"First 10 bytes: {pcm_data[:10]}")

                res = kws.run(pcm_data)
                if res:
                    print("====Detected XiaonanXiaonan!====")
                    wf = wave.open(reply_wav_file, "rb")
                    wav_data = wf.read_frames(CHUNK)
                    while wav_data:
                        output_stream.write(wav_data)
                        wav_data = wf.read_frames(CHUNK)
                    time.sleep(1)  # 播放后缓冲
                    wf.close()
                else:
                    print("Deactivated!")
                gc.collect()  # 垃圾回收
    except Exception as e:
        sys.print_exception(e)
    finally:
        input_stream.stop_stream()
        output_stream.stop_stream()
        input_stream.close()
        output_stream.close()
        p.terminate()
        MediaManager.deinit()
        aidemo.kws_fp_destroy(fp)
        kws.deinit()
