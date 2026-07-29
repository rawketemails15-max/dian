# ============================================================
# K230 自建热点 + H.264 RTSP 低延迟无线图传
#
# 适用固件：
#   CanMV v1.4-19
#   k230_canmv_hiwonder
#
# 电脑播放地址：
#   rtsp://K230_IP:8554/ball
# VLC直连热点建议强制RTSP/TCP：
#   vlc --rtsp-tcp --network-caching=100 rtsp://K230_IP:8554/ball
#
# 开机脱机运行：
#   将本文件保存到开发板 /sdcard/main.py
#
# 低延迟参数：
#   640x480
#   30 FPS
#   1500 kbps
#   GOP = 10
#   VENC缓冲区 = 4
# ============================================================

import network
import time
import os
import gc
import _thread
import uctypes

from media.vencoder import *
from media.sensor import *
from media.media import *

# RTSP模块延迟到AP客户端接入后再加载，避免冷启动时过早初始化网络栈
mm = None


# ============================================================
# Wi-Fi 热点配置
# ============================================================

AP_SSID = "K230_AP"
AP_PASSWORD = "12345678"
AP_READY_TIMEOUT_MS = 10000
AP_POLL_INTERVAL_MS = 200
BOOT_SETTLE_MS = 3000
AP_CLIENT_POLL_MS = 250
AP_CLIENT_SETTLE_MS = 2000
AP_STATUS_FALLBACK_SETTLE_MS = 8000
SCRIPT_VERSION = "rtsp-offline-v2"

# 脱机启动时无法看IDE终端，将关键阶段追加到SD卡
BOOT_LOG_PATH = "/sdcard/wifi_rtsp.log"
BOOT_LOG_MAX_BYTES = 32768


# ============================================================
# RTSP 配置
# ============================================================

RTSP_PORT = 8554
RTSP_SESSION = "ball"


# ============================================================
# 视频低延迟配置
# ============================================================

VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480

# 摄像头输入和H.264输出帧率
VIDEO_FPS = 30

# 单位：kbps
# 640x480建议先使用1200～1800
VIDEO_BIT_RATE = 1500

# 每10帧一个关键帧
# 30FPS下约每0.33秒一个关键帧
VIDEO_GOP = 10

# 原来是8，降低到4以减少编码排队
VENC_BUFFER_COUNT = 4

# B.py已经验证本板摄像头挂在设备2
SENSOR_ID = 2

# GetStream最长阻塞时间，单位ms
GET_STREAM_TIMEOUT_MS = 1000

# 启动后必须在此时间内取得第一帧，否则判定媒体链路启动失败
FIRST_FRAME_TIMEOUT_MS = 6000

# 已经出过帧后，连续超过此时间无新帧则重建整个RTSP媒体链路
STREAM_STALL_TIMEOUT_MS = 5000

# 主循环健康统计和自动恢复参数
HEALTH_PRINT_INTERVAL_MS = 2000
THREAD_STOP_TIMEOUT_MS = 3000
RESTART_DELAY_MS = 2000


# ============================================================
# 异常显示
# Hiwonder固件没有 sys.print_exception()
# ============================================================

def show_exception(error, position=""):
    print()
    print("----------------------------------------")

    if position:
        print("异常位置:", position)

    try:
        print("异常类型:", type(error).__name__)
    except Exception:
        print("异常类型: 无法读取")

    try:
        print("异常内容:", error)
    except Exception:
        print("异常内容: 无法读取")

    print("----------------------------------------")
    print()


# ============================================================
# 脱机关键事件日志
# ============================================================

def log_event(message):
    line = "[%dms] %s" % (
        time.ticks_ms(),
        message
    )
    print(line)

    try:
        mode = "a"

        try:
            if os.stat(BOOT_LOG_PATH)[6] >= BOOT_LOG_MAX_BYTES:
                mode = "w"
        except Exception:
            pass

        with open(BOOT_LOG_PATH, mode) as log_file:
            log_file.write(line)
            log_file.write("\n")
    except Exception:
        # 日志失败不能影响图传主流程
        pass


# ============================================================
# 等待AP客户端接入
# ============================================================

def get_ap_station_count(ap):
    try:
        stations = ap.status("stations")

        if stations is None:
            return 0

        try:
            return len(stations)
        except Exception:
            return 1 if stations else 0

    except Exception:
        # 部分Hiwonder固件只在AP模式实现isconnected()
        try:
            return 1 if ap.isconnected() else 0
        except Exception:
            return None


def wait_for_ap_client(ap):
    print("等待电脑/手机连接K230热点后再启动RTSP……")

    while True:
        station_count = get_ap_station_count(ap)

        if station_count is None:
            # 无法查询站点的旧固件只能采用保守固定等待
            log_event(
                "固件不支持查询AP客户端，等待%dms"
                % AP_STATUS_FALLBACK_SETTLE_MS
            )
            time.sleep_ms(
                AP_STATUS_FALLBACK_SETTLE_MS
            )
            return

        if station_count > 0:
            log_event(
                "检测到AP客户端%d个，等待网络路由稳定"
                % station_count
            )
            time.sleep_ms(AP_CLIENT_SETTLE_MS)
            return

        try:
            os.exitpoint()
        except Exception:
            pass

        time.sleep_ms(AP_CLIENT_POLL_MS)


def ensure_multimedia_loaded():
    global mm

    if mm is None:
        log_event("网络链路已就绪，开始加载multimedia RTSP模块")
        import multimedia as multimedia_module
        mm = multimedia_module


# ============================================================
# 创建K230 Wi-Fi热点
# ============================================================

def start_ap():
    print("正在创建K230 Wi-Fi热点……")

    ap = network.WLAN(network.AP_IF)

    # 不同定制固件对active()支持略有差异
    try:
        ap.active(True)
    except Exception:
        pass

    # 不同固件的config参数形式不完全一致，不添加channel参数
    try:
        ap.config(
            ssid=AP_SSID,
            key=AP_PASSWORD
        )
    except Exception:
        ap.config("ssid", AP_SSID)
        ap.config("key", AP_PASSWORD)

    # 不能只固定等待几秒：必须确认AP已经拿到有效地址
    wait_started_ms = time.ticks_ms()
    ip_config = None
    ap_ip = None

    while True:
        try:
            ip_config = ap.ifconfig()
            ap_ip = ip_config[0]
        except Exception:
            ip_config = None
            ap_ip = None

        if ap_ip and ap_ip != "0.0.0.0":
            break

        if (
            time.ticks_diff(
                time.ticks_ms(),
                wait_started_ms
            )
            >= AP_READY_TIMEOUT_MS
        ):
            raise RuntimeError(
                "AP热点在%dms内未取得有效IP"
                % AP_READY_TIMEOUT_MS
            )

        time.sleep_ms(AP_POLL_INTERVAL_MS)

    print()
    print("========================================")
    print("K230热点创建完成")
    print("热点名称:", AP_SSID)
    print("热点密码:", AP_PASSWORD)

    try:
        print("AP状态:", ap.status())
    except Exception as error:
        print("读取AP状态失败:", error)

    try:
        print("AP信息:", ap.info())
    except Exception as error:
        print("读取AP信息失败:", error)

    print("IP配置:", ip_config)
    print("K230 IP:", ap_ip)
    print("========================================")
    print()

    log_event(
        "AP就绪 ssid=%s ip=%s"
        % (
            AP_SSID,
            ap_ip
        )
    )

    return ap, ap_ip


# ============================================================
# RTSP低延迟摄像头服务器
# ============================================================

class RtspCameraServer:

    def __init__(
        self,
        session_name="ball",
        port=8554,
        width=640,
        height=480
    ):
        ensure_multimedia_loaded()

        self.session_name = session_name
        self.port = port

        # H.264编码宽度按16像素对齐
        self.width = ALIGN_UP(width, 16)
        self.height = height

        self.video_type = mm.multi_media_type.media_h264
        self.enable_audio = False

        # v1.4必须显式指定VENC通道
        self.venc_chn = VENC_CHN_ID_0

        self.rtsp_server = mm.rtsp_server()

        self.sensor = None
        self.encoder = None
        self.link = None

        self.media_initialized = False
        self.encoder_created = False
        self.encoder_started = False
        self.sensor_started = False

        self.rtsp_initialized = False
        self.rtsp_started = False
        self.session_created = False

        self.stream_thread_started = False
        self.start_stream = False
        self.runthread_over = True

        self.server_started = False

        # 编码线程健康状态。主线程只读取这些简单值，
        # 避免在固件的轻量线程实现中额外引入锁。
        self.first_frame_ready = False
        self.encoded_frames = 0
        self.encoded_bytes = 0
        self.last_stream_ms = 0
        self.get_stream_failures = 0
        self.thread_error = None

        self.health_sample_ms = time.ticks_ms()
        self.health_sample_frames = 0
        self.health_sample_bytes = 0

    # --------------------------------------------------------
    # 初始化摄像头、编码器和媒体缓冲区
    # --------------------------------------------------------

    def _init_video_stream(self):
        print("正在初始化摄像头……")

        self.sensor = Sensor(id=SENSOR_ID)
        self.sensor.reset()

        self.sensor.set_framesize(
            width=self.width,
            height=self.height,
            alignment=12
        )

        # H.264硬件编码使用YUV420SP
        self.sensor.set_pixformat(Sensor.YUV420SP)

        print(
            "摄像头设备/分辨率:",
            SENSOR_ID,
            "/",
            self.width,
            "x",
            self.height
        )

        print("正在初始化H.264编码器……")

        self.encoder = Encoder()

        # 必须在MediaManager.init()之前调用
        # 缓冲区由原来的8降低到4
        self.encoder.SetOutBufs(
            self.venc_chn,
            VENC_BUFFER_COUNT,
            self.width,
            self.height
        )

        # 摄像头直接绑定到VENC通道
        self.link = MediaManager.link(
            self.sensor.bind_info()["src"],
            (
                VIDEO_ENCODE_MOD_ID,
                VENC_DEV_ID,
                self.venc_chn
            )
        )

        print("正在初始化媒体缓冲区……")

        MediaManager.init()
        self.media_initialized = True

        # 参数顺序：
        # payloadType
        # profile
        # picWidth
        # picHeight
        # bit_rate
        # gopLen
        # src_frame_rate
        # dst_frame_rate
        encoder_attribute = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,

            # Baseline通常比Main更适合实时低延迟播放
            self.encoder.H264_PROFILE_BASELINE,

            self.width,
            self.height,

            VIDEO_BIT_RATE,
            VIDEO_GOP,
            VIDEO_FPS,
            VIDEO_FPS
        )

        self.encoder.Create(
            self.venc_chn,
            encoder_attribute
        )

        self.encoder_created = True

        print("H.264编码器初始化完成")
        print("视频帧率:", VIDEO_FPS, "FPS")
        print("视频码率:", VIDEO_BIT_RATE, "kbps")
        print("GOP长度:", VIDEO_GOP)
        print("编码缓冲区数量:", VENC_BUFFER_COUNT)

    # --------------------------------------------------------
    # 启动摄像头和编码器
    # --------------------------------------------------------

    def _start_video_stream(self):
        print("正在启动H.264编码器……")

        self.encoder.Start(self.venc_chn)
        self.encoder_started = True

        print("正在启动摄像头……")

        self.sensor.run()
        self.sensor_started = True

        print("摄像头和编码器启动完成")

    # --------------------------------------------------------
    # 创建并启动RTSP服务器
    # --------------------------------------------------------

    def _start_rtsp_server(self):
        print("正在初始化RTSP服务……")

        self.rtsp_server.rtspserver_init(self.port)
        self.rtsp_initialized = True

        self.rtsp_server.rtspserver_createsession(
            self.session_name,
            self.video_type,
            self.enable_audio
        )

        self.session_created = True

        self.rtsp_server.rtspserver_start()
        self.rtsp_started = True

        print("RTSP服务启动完成")

    # --------------------------------------------------------
    # 启动整个图传系统
    # --------------------------------------------------------

    def start(self):
        if self.server_started:
            print("RTSP图传服务已经在运行")
            return

        self.first_frame_ready = False
        self.encoded_frames = 0
        self.encoded_bytes = 0
        self.last_stream_ms = 0
        self.get_stream_failures = 0
        self.thread_error = None
        self.health_sample_ms = time.ticks_ms()
        self.health_sample_frames = 0
        self.health_sample_bytes = 0
        self.runthread_over = False

        try:
            # 1. 初始化摄像头和编码器
            self._init_video_stream()

            # 2. 初始化RTSP服务器
            self._start_rtsp_server()

            # 3. 启动编码器和摄像头
            self._start_video_stream()

            # 4. 启动发送线程
            self.start_stream = True
            self.stream_thread_started = True

            _thread.start_new_thread(
                self._stream_thread,
                ()
            )

            # 端口成功监听不等于视频链路正常。
            # 必须确认VENC确实产生并送出了第一帧H.264。
            wait_started_ms = time.ticks_ms()

            while not self.first_frame_ready:
                if self.runthread_over:
                    detail = self.thread_error
                    if not detail:
                        detail = "编码线程提前退出"
                    raise RuntimeError(detail)

                if (
                    time.ticks_diff(
                        time.ticks_ms(),
                        wait_started_ms
                    )
                    >= FIRST_FRAME_TIMEOUT_MS
                ):
                    raise RuntimeError(
                        "%dms内没有取得H.264首帧，GetStream失败次数=%d"
                        % (
                            FIRST_FRAME_TIMEOUT_MS,
                            self.get_stream_failures
                        )
                    )

                time.sleep_ms(50)

            self.health_sample_ms = time.ticks_ms()
            self.health_sample_frames = self.encoded_frames
            self.health_sample_bytes = self.encoded_bytes
            self.server_started = True
            print(
                "已取得H.264首帧，字节数:",
                self.encoded_bytes
            )
            print("RTSP低延迟图传系统启动成功")

        except BaseException as error:
            show_exception(
                error,
                "RtspCameraServer.start"
            )

            self.start_stream = False

            if self.stream_thread_started:
                self._wait_stream_thread(
                    THREAD_STOP_TIMEOUT_MS
                )
            else:
                self.runthread_over = True

            self._release_resources()

            raise error

    # --------------------------------------------------------
    # RTSP视频发送线程
    # --------------------------------------------------------

    def _stream_thread(self):
        stream_data = StreamData()

        try:
            while self.start_stream:
                # 允许CanMV IDE停止程序
                try:
                    os.exitpoint()
                except Exception:
                    pass

                stream_received = False

                try:
                    # 使用超时模式，避免停止时永久阻塞
                    result = self.encoder.GetStream(
                        self.venc_chn,
                        stream_data,
                        GET_STREAM_TIMEOUT_MS
                    )

                    # 非0表示没有成功取得码流
                    if result != 0:
                        self.get_stream_failures += 1
                        continue

                    stream_received = True
                    frame_bytes = 0

                    # 每一帧可能包含多个编码数据包
                    for pack_index in range(
                        stream_data.pack_cnt
                    ):
                        packet_size = (
                            stream_data.data_size[pack_index]
                        )

                        if packet_size <= 0:
                            continue

                        packet = bytes(
                            uctypes.bytearray_at(
                                stream_data.data[pack_index],
                                packet_size
                            )
                        )

                        # v1.4官方RTSP示例使用1000作为时间戳参数
                        self.rtsp_server.rtspserver_sendvideodata(
                            self.session_name,
                            packet,
                            packet_size,
                            1000
                        )

                        frame_bytes += packet_size

                        # 正式运行不要逐帧print
                        # 否则串口输出会增加卡顿
                        #
                        # print(
                        #     "大小:",
                        #     packet_size,
                        #     "类型:",
                        #     stream_data.stream_type[pack_index]
                        # )

                    if frame_bytes > 0:
                        self.encoded_frames += 1
                        self.encoded_bytes += frame_bytes
                        self.last_stream_ms = time.ticks_ms()
                        self.first_frame_ready = True

                finally:
                    if stream_received:
                        try:
                            self.encoder.ReleaseStream(
                                self.venc_chn,
                                stream_data
                            )
                        except Exception as error:
                            print("释放编码帧失败:", error)

        except BaseException as error:
            try:
                self.thread_error = (
                    "%s: %s"
                    % (
                        type(error).__name__,
                        error
                    )
                )
            except Exception:
                self.thread_error = "编码线程发生未知异常"

            show_exception(
                error,
                "RTSP视频发送线程"
            )

            self.start_stream = False

        finally:
            self.runthread_over = True
            print("RTSP视频发送线程已退出")

    # --------------------------------------------------------
    # 等待编码线程退出
    # --------------------------------------------------------

    def _wait_stream_thread(self, timeout_ms):
        if not self.stream_thread_started:
            return True

        wait_started_ms = time.ticks_ms()

        while not self.runthread_over:
            if (
                time.ticks_diff(
                    time.ticks_ms(),
                    wait_started_ms
                )
                >= timeout_ms
            ):
                return False

            time.sleep_ms(50)

        return True

    # --------------------------------------------------------
    # 健康状态
    # --------------------------------------------------------

    def health_error(self):
        if self.thread_error:
            return self.thread_error

        if self.stream_thread_started and self.runthread_over:
            return "编码线程已经退出"

        if not self.first_frame_ready:
            return "尚未取得H.264首帧"

        if self.last_stream_ms:
            silent_ms = time.ticks_diff(
                time.ticks_ms(),
                self.last_stream_ms
            )

            if silent_ms > STREAM_STALL_TIMEOUT_MS:
                return (
                    "编码码流已中断%dms"
                    % silent_ms
                )

        return None

    def is_healthy(self):
        if not self.server_started:
            return False

        return self.health_error() is None

    def print_health(self):
        now = time.ticks_ms()
        elapsed_ms = time.ticks_diff(
            now,
            self.health_sample_ms
        )

        if elapsed_ms <= 0:
            return

        frame_delta = (
            self.encoded_frames
            - self.health_sample_frames
        )
        byte_delta = (
            self.encoded_bytes
            - self.health_sample_bytes
        )

        fps = (
            frame_delta
            * 1000.0
            / elapsed_ms
        )

        # byte * 8 / ms 正好是kbit/s
        kbps = (
            byte_delta
            * 8.0
            / elapsed_ms
        )

        print(
            "[HEALTH] fps=%.1f bitrate=%.0fkbps frames=%d bytes=%d "
            "get_fail=%d thread=%s"
            % (
                fps,
                kbps,
                self.encoded_frames,
                self.encoded_bytes,
                self.get_stream_failures,
                "RUN" if not self.runthread_over else "STOP"
            )
        )

        self.health_sample_ms = now
        self.health_sample_frames = self.encoded_frames
        self.health_sample_bytes = self.encoded_bytes

    # --------------------------------------------------------
    # 获取RTSP地址
    # --------------------------------------------------------

    def get_rtsp_url(self):
        try:
            return (
                self.rtsp_server
                .rtspserver_getrtspurl(
                    self.session_name
                )
            )

        except Exception:
            try:
                return (
                    self.rtsp_server
                    .rtspserver_getrtspurl()
                )

            except Exception:
                return None

    # --------------------------------------------------------
    # 释放全部资源
    # --------------------------------------------------------

    def _release_resources(self):
        print("正在释放图传资源……")

        # 1. 停止摄像头
        if (
            self.sensor_started
            and self.sensor is not None
        ):
            try:
                self.sensor.stop()
                print("摄像头已停止")
            except Exception as error:
                print("停止摄像头失败:", error)

            self.sensor_started = False

        # 2. 解除摄像头与VENC绑定
        if self.link is not None:
            try:
                link_object = self.link
                self.link = None
                del link_object

                print("摄像头与编码器绑定已解除")
            except Exception as error:
                print("解除媒体绑定失败:", error)

        # 3. 停止编码器
        if (
            self.encoder_started
            and self.encoder is not None
        ):
            try:
                self.encoder.Stop(
                    self.venc_chn
                )

                print("H.264编码器已停止")
            except Exception as error:
                print("停止编码器失败:", error)

            self.encoder_started = False

        # 4. 销毁编码器
        if (
            self.encoder_created
            and self.encoder is not None
        ):
            try:
                self.encoder.Destroy(
                    self.venc_chn
                )

                print("H.264编码器已销毁")
            except Exception as error:
                print("销毁编码器失败:", error)

            self.encoder_created = False

        # 5. 释放媒体缓冲区
        if self.media_initialized:
            try:
                MediaManager.deinit()
                print("媒体缓冲区已释放")
            except Exception as error:
                print("释放媒体缓冲区失败:", error)

            self.media_initialized = False

        # 6. 停止RTSP服务器
        if self.rtsp_started:
            try:
                self.rtsp_server.rtspserver_stop()
                print("RTSP服务已停止")
            except Exception as error:
                print("停止RTSP服务失败:", error)

            self.rtsp_started = False

        # 7. 销毁RTSP会话
        if self.session_created:
            try:
                self.rtsp_server.rtspserver_destroysession(
                    self.session_name
                )

                print("RTSP会话已销毁")
            except Exception as error:
                print("销毁RTSP会话失败:", error)

            self.session_created = False

        # 8. 反初始化RTSP服务
        if self.rtsp_initialized:
            try:
                self.rtsp_server.rtspserver_deinit()
                print("RTSP服务已反初始化")
            except Exception as error:
                print("反初始化RTSP服务失败:", error)

            self.rtsp_initialized = False

        self.sensor = None
        self.encoder = None
        self.start_stream = False
        self.stream_thread_started = False
        self.server_started = False

        gc.collect()

        print("图传资源释放完成")

    # --------------------------------------------------------
    # 停止整个图传系统
    # --------------------------------------------------------

    def stop(self):
        print("正在停止RTSP图传系统……")

        self.start_stream = False

        # 等待发送线程结束
        if self.stream_thread_started:
            if not self._wait_stream_thread(
                THREAD_STOP_TIMEOUT_MS
            ):
                print(
                    "警告：发送线程未在规定时间内退出"
                )

        self._release_resources()

        print("RTSP图传系统已经停止")


# ============================================================
# 主程序
# ============================================================

def main():
    ap = None
    rtsp = None
    ap_ip = None
    restart_count = 0
    last_health_print_ms = 0

    try:
        log_event(
            "wifi.py/main.py开始执行 version=%s"
            % SCRIPT_VERSION
        )
        print(
            "冷启动等待%dms，让Wi-Fi和媒体驱动完成初始化……"
            % BOOT_SETTLE_MS
        )
        time.sleep_ms(BOOT_SETTLE_MS)

        # 允许CanMV IDE中断程序
        try:
            os.exitpoint(
                os.EXITPOINT_ENABLE
            )
        except Exception:
            pass

        # 1. 创建Wi-Fi热点
        ap, ap_ip = start_ap()

        # 使用AP地址手动构造URL，避免多网卡时模块返回错误接口地址
        manual_url = (
            "rtsp://{}:{}/{}"
            .format(
                ap_ip,
                RTSP_PORT,
                RTSP_SESSION
            )
        )

        # 永久监督RTSP媒体链路。线程退出或码流停止时自动完整重建。
        while True:
            try:
                os.exitpoint()
            except Exception:
                pass

            if rtsp is None:
                try:
                    print()
                    if restart_count:
                        print(
                            "正在第%d次重建RTSP媒体链路……"
                            % restart_count
                        )

                    # 冷启动时RTSP不能早于首个AP客户端初始化。
                    # 在线运行时电脑通常已经连上热点，因此不会暴露此时序问题。
                    wait_for_ap_client(ap)

                    rtsp = RtspCameraServer(
                        session_name=RTSP_SESSION,
                        port=RTSP_PORT,
                        width=VIDEO_WIDTH,
                        height=VIDEO_HEIGHT
                    )

                    # start()只有在实际送出首帧后才会成功返回
                    rtsp.start()

                    official_url = rtsp.get_rtsp_url()

                    print()
                    print("========================================")
                    print("K230低延迟无线图传启动成功")
                    print("编码首帧自检: PASS")
                    print()
                    print("电脑连接热点:")
                    print("热点名称:", AP_SSID)
                    print("热点密码:", AP_PASSWORD)
                    print()
                    print("VLC网络串流地址:")
                    print(manual_url)
                    print()
                    print("VLC必须优先使用RTSP over TCP:")
                    print(
                        'vlc --rtsp-tcp --network-caching=100 "%s"'
                        % manual_url
                    )
                    print(
                        "说明: 8554/TCP连通只代表控制端口可用，"
                        "不代表默认UDP视频通道可用"
                    )

                    if official_url:
                        print("模块返回地址:", official_url)

                    print()
                    print("当前视频参数:")
                    print(
                        VIDEO_WIDTH,
                        "x",
                        VIDEO_HEIGHT,
                        "@",
                        VIDEO_FPS,
                        "FPS"
                    )
                    print("码率:", VIDEO_BIT_RATE, "kbps")
                    print("GOP:", VIDEO_GOP)
                    print("VENC缓冲:", VENC_BUFFER_COUNT)
                    print("自动恢复次数:", restart_count)
                    print()
                    print("按Ctrl+C或IDE停止按钮结束")
                    print("========================================")
                    print()

                    log_event(
                        "RTSP首帧自检通过 url=%s restart=%d"
                        % (
                            manual_url,
                            restart_count
                        )
                    )
                    last_health_print_ms = time.ticks_ms()

                except KeyboardInterrupt:
                    raise

                except BaseException as error:
                    try:
                        log_event(
                            "RTSP启动失败 restart=%d error=%s"
                            % (
                                restart_count,
                                error
                            )
                        )
                    except Exception:
                        pass

                    show_exception(
                        error,
                        "main.启动RTSP"
                    )

                    if rtsp is not None:
                        try:
                            rtsp.stop()
                        except BaseException as stop_error:
                            show_exception(
                                stop_error,
                                "main.启动失败后的清理"
                            )

                    rtsp = None
                    restart_count += 1
                    print(
                        "%dms后自动重试……"
                        % RESTART_DELAY_MS
                    )

                    delay_started_ms = time.ticks_ms()
                    while (
                        time.ticks_diff(
                            time.ticks_ms(),
                            delay_started_ms
                        )
                        < RESTART_DELAY_MS
                    ):
                        try:
                            os.exitpoint()
                        except Exception:
                            pass

                        time.sleep_ms(100)

                    continue

            health_problem = rtsp.health_error()

            if health_problem:
                print()
                print(
                    "[RECOVERY] 检测到媒体链路异常:",
                    health_problem
                )
                log_event(
                    "媒体链路异常 restart=%d reason=%s"
                    % (
                        restart_count,
                        health_problem
                    )
                )

                try:
                    rtsp.stop()
                except BaseException as error:
                    show_exception(
                        error,
                        "main.异常链路清理"
                    )

                rtsp = None
                restart_count += 1
                print(
                    "%dms后重建RTSP媒体链路……"
                    % RESTART_DELAY_MS
                )

                delay_started_ms = time.ticks_ms()
                while (
                    time.ticks_diff(
                        time.ticks_ms(),
                        delay_started_ms
                    )
                    < RESTART_DELAY_MS
                ):
                    try:
                        os.exitpoint()
                    except Exception:
                        pass

                    time.sleep_ms(100)

                continue

            now = time.ticks_ms()
            if (
                time.ticks_diff(
                    now,
                    last_health_print_ms
                )
                >= HEALTH_PRINT_INTERVAL_MS
            ):
                rtsp.print_health()
                last_health_print_ms = now

            time.sleep_ms(100)

    except KeyboardInterrupt:
        log_event("用户停止程序")
        print("用户停止程序")

    except BaseException as error:
        try:
            log_event(
                "main异常: %s"
                % error
            )
        except Exception:
            pass

        show_exception(
            error,
            "main"
        )

    finally:
        if rtsp is not None:
            try:
                rtsp.stop()
            except BaseException as error:
                show_exception(
                    error,
                    "main.finally.rtsp.stop"
                )

        ap = None
        gc.collect()

        log_event("程序结束")
        print("程序结束")


# ============================================================
# 程序入口
# ============================================================

if __name__ == "__main__":
    main()
