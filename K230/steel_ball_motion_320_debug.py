# ============================================================
# K230 钢珠识别 + LCD标记 + 原始画面H.264 RTSP无线图传
#
# 适用固件：
#   CanMV v1.4-19
#   k230_canmv_hiwonder
#
# 三路摄像头输出：
#   通道0：800x480 YUV420 -> ST7701 LCD视频层
#   通道1：640x480 YUV420 -> H.264 VENC -> RTSP（无识别标记）
#   通道2：320x320 RGB888P -> KPU钢珠识别
#
# 电脑播放地址：
#   rtsp://K230_IP:8554/ball
# VLC直连热点建议强制RTSP/TCP：
#   vlc --rtsp-tcp --network-caching=100 rtsp://K230_IP:8554/ball
# ============================================================

import network
import socket
import time
import os
import gc
import _thread
import uctypes

from media.vencoder import *
from media.sensor import *
from media.display import Display
from media.media import *

import nncase_runtime as nn
import ulab.numpy as np
import image


# RTSP模块延迟到AP客户端接入后再加载，避免冷启动时过早初始化网络栈。
mm = None


# ============================================================
# Wi-Fi热点配置
# ============================================================

AP_SSID = "K230_AP"
AP_PASSWORD = "12345678"
AP_READY_TIMEOUT_MS = 10000
AP_POLL_INTERVAL_MS = 200
BOOT_SETTLE_MS = 3000
AP_CLIENT_POLL_MS = 250
AP_CLIENT_SETTLE_MS = 2000
AP_STATUS_FALLBACK_SETTLE_MS = 8000
AP_CLIENT_WAIT_TIMEOUT_MS = 15000

SCRIPT_VERSION = "steel-ball-full-v3"
BOOT_LOG_PATH = "/sdcard/wifi_rtsp.log"
BOOT_LOG_MAX_BYTES = 32768


# ============================================================
# RTSP与H.264配置
# ============================================================

RTSP_PORT = 8554
RTSP_SESSION = "ball"

VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 30
VIDEO_BIT_RATE = 1500
VIDEO_GOP = 10
VENC_BUFFER_COUNT = 4

GET_STREAM_TIMEOUT_MS = 1000
FIRST_FRAME_TIMEOUT_MS = 6000
STREAM_STALL_TIMEOUT_MS = 15000
HEALTH_PRINT_INTERVAL_MS = 2000
THREAD_STOP_TIMEOUT_MS = 3000
RESTART_DELAY_MS = 2000


# ============================================================
# 摄像头、显示和AI配置
# ============================================================

# 已在原wifi.py中验证本板摄像头挂在设备2。
SENSOR_ID = 2
SENSOR_INPUT_WIDTH = 1280
SENSOR_INPUT_HEIGHT = 960

DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480

KMODEL_PATH = (
    "/sdcard/examples/kmodel/"
    "steel_ball_single_320_lowlatency.kmodel"
)
AI_FRAME_SIZE = [320, 320]

# 首次锁定采用较高阈值；锁定后允许较低阈值，以适应运动模糊。
ACQUIRE_CONFIDENCE = 0.18
TRACK_CONFIDENCE = 0.05
MAX_CANDIDATES = 12
MAX_TRACK_JUMP = 70.0
LOST_FRAMES_TO_UNLOCK = 3
MIN_BALL_SIZE = 4.0
MAX_BALL_SIZE = 80.0
MIN_ASPECT_RATIO = 0.50
MAX_ASPECT_RATIO = 2.00

MARKER_SIZE = 10
MARKER_COLOR = (255, 0, 255, 0)
GC_INTERVAL = 180
DISPLAY_MARKER_RETRY_MS = 1000


# ============================================================
# Coordinate output and optional two-axis servo control
# ============================================================

# Every valid/predicted ball position is broadcast to the controller PC:
# BALL,x,y,vx,vy,confidence,timestamp_us
UDP_ENABLED = True
UDP_PORT = 9000

# Keep this False until the servos use an external regulated 5 V supply and
# the external supply GND is connected to K230 GND. Set True after wiring.
SERVO_ENABLED = False

# K230 PWM0/PWM2. Both belong to the same 0..2 clock group and can share
# the required 50 Hz servo frequency. GPIO43/PWM1 is avoided because the
# Hiwonder board uses it for the buzzer.
SERVO_X_PIN = 42
SERVO_Y_PIN = 46
SERVO_X_ENABLED = True
SERVO_Y_ENABLED = True
SERVO_FREQUENCY = 50

# Mechanical calibration. Start with this conservative +/-12 degree range.
SERVO_X_CENTER_DEG = 90.0
SERVO_Y_CENTER_DEG = 90.0
SERVO_X_MIN_DEG = 78.0
SERVO_X_MAX_DEG = 102.0
SERVO_Y_MIN_DEG = 78.0
SERVO_Y_MAX_DEG = 102.0
SERVO_X_INVERT = False
SERVO_Y_INVERT = False

# Position/velocity feedback. Error and velocity are normalized by the
# 320x320 image size, so these values are easy to tune on the mechanism.
SERVO_X_KP = 10.0
SERVO_Y_KP = 10.0
SERVO_X_KD = 1.0
SERVO_Y_KD = 1.0
SERVO_TARGET_X = 160.0
SERVO_TARGET_Y = 160.0
SERVO_UPDATE_INTERVAL_MS = 20
SERVO_LOST_HOLD_MS = 300
SERVO_LOST_CENTER_MS = 1200

# One predicted frame prevents an isolated YOLO miss from freezing the servo.
PREDICT_SINGLE_MISSED_FRAME = True
MAX_PREDICT_SPEED = 900.0


last_marker_update_ms = 0
display_marker_enabled = True
display_marker_error_reported = False
display_marker_last_error_ms = 0


# ============================================================
# 通用辅助函数
# ============================================================

def show_exception(error, position=""):
    """兼容没有sys.print_exception()的Hiwonder固件。"""
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


def log_event(message):
    """把关键启动和恢复事件同时写到终端与SD卡。"""
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
        # 日志失败不能影响识别或图传主流程。
        pass


def delay_with_exitpoint(delay_ms):
    """可被IDE停止按钮打断的毫秒延时。"""
    delay_started_ms = time.ticks_ms()

    while (
        time.ticks_diff(
            time.ticks_ms(),
            delay_started_ms
        )
        < delay_ms
    ):
        try:
            os.exitpoint()
        except KeyboardInterrupt:
            raise
        except Exception:
            pass

        time.sleep_ms(100)


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
        # 部分Hiwonder固件只在AP模式实现isconnected()。
        try:
            return 1 if ap.isconnected() else 0
        except Exception:
            return None


def wait_for_ap_client(ap):
    print("等待电脑/手机连接K230热点后再启动识别与RTSP……")
    wait_started_ms = time.ticks_ms()

    while True:
        station_count = get_ap_station_count(ap)

        if station_count is None:
            log_event(
                "固件不支持查询AP客户端，等待%dms"
                % AP_STATUS_FALLBACK_SETTLE_MS
            )
            delay_with_exitpoint(
                AP_STATUS_FALLBACK_SETTLE_MS
            )
            return False

        if station_count > 0:
            log_event(
                "检测到AP客户端%d个，等待网络路由稳定"
                % station_count
            )
            delay_with_exitpoint(AP_CLIENT_SETTLE_MS)
            return True

        if (
            time.ticks_diff(
                time.ticks_ms(),
                wait_started_ms
            )
            >= AP_CLIENT_WAIT_TIMEOUT_MS
        ):
            log_event(
                "等待AP客户端超时，先启动LCD、识别和舵机；"
                "电脑稍后仍可连接RTSP"
            )
            return False

        try:
            os.exitpoint()
        except KeyboardInterrupt:
            raise
        except Exception:
            pass

        time.sleep_ms(AP_CLIENT_POLL_MS)


def ensure_multimedia_loaded():
    global mm

    if mm is None:
        log_event("网络链路已就绪，开始加载multimedia RTSP模块")
        import multimedia as multimedia_module
        mm = multimedia_module


def start_ap():
    print("正在创建K230 Wi-Fi热点……")

    ap = network.WLAN(network.AP_IF)

    try:
        ap.active(True)
    except Exception:
        pass

    try:
        ap.config(
            ssid=AP_SSID,
            key=AP_PASSWORD
        )
    except Exception:
        ap.config("ssid", AP_SSID)
        ap.config("key", AP_PASSWORD)

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
# Wireless coordinates and local two-axis servo output
# ============================================================

class UdpBallSender:
    def __init__(self, ap_ip):
        self.sock = None
        self.sent = 0
        self.failures = 0

        if not UDP_ENABLED:
            self.target = None
            return

        octets = ap_ip.split(".")
        if len(octets) == 4:
            broadcast_ip = (
                octets[0] + "." + octets[1] + "."
                + octets[2] + ".255"
            )
        else:
            broadcast_ip = "255.255.255.255"

        self.target = (broadcast_ip, UDP_PORT)

        try:
            self.sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            )
            self.sock.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_BROADCAST,
                1
            )
            print(
                "UDP坐标: %s:%d"
                % (broadcast_ip, UDP_PORT)
            )
        except Exception as error:
            self.sock = None
            print("UDP坐标输出关闭:", error)

    def send(self, motion):
        if self.sock is None or motion is None:
            return

        message = (
            "BALL,%.2f,%.2f,%.2f,%.2f,%.3f,%d\n"
            % (
                motion[0],
                motion[1],
                motion[2],
                motion[3],
                motion[4],
                motion[5]
            )
        )

        try:
            self.sock.sendto(
                message.encode(),
                self.target
            )
            self.sent += 1
        except Exception:
            # Network loss must never interrupt inference or servo control.
            self.failures += 1

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None


class BallServoController:
    """Low-latency PD position controller for two 50 Hz PWM servos."""

    def __init__(self):
        self.enabled = False
        self.pwm_x = None
        self.pwm_y = None
        self.last_update_ms = 0
        self.last_seen_ms = 0
        self.angle_x = SERVO_X_CENTER_DEG
        self.angle_y = SERVO_Y_CENTER_DEG

        if not SERVO_ENABLED:
            print("舵机PWM: 已关闭（接好独立5V和共地后设置SERVO_ENABLED=True）")
            return

        try:
            from machine import PWM

            if SERVO_X_ENABLED:
                self.pwm_x = PWM(
                    SERVO_X_PIN,
                    freq=SERVO_FREQUENCY
                )

            if SERVO_Y_ENABLED:
                self.pwm_y = PWM(
                    SERVO_Y_PIN,
                    freq=SERVO_FREQUENCY
                )

            self._write_angles(
                SERVO_X_CENTER_DEG,
                SERVO_Y_CENTER_DEG
            )
            self.enabled = True
            print(
                "舵机PWM已启动: X=GPIO%d Y=GPIO%d %dHz"
                % (
                    SERVO_X_PIN,
                    SERVO_Y_PIN,
                    SERVO_FREQUENCY
                )
            )
        except Exception as error:
            print("舵机PWM初始化失败，视觉功能继续:", error)
            self.close(center_first=False)

    @staticmethod
    def _clamp(value, minimum, maximum):
        if value < minimum:
            return minimum
        if value > maximum:
            return maximum
        return value

    @staticmethod
    def _angle_to_duty_u16(angle):
        # 0 degree -> 0.5 ms, 90 -> 1.5 ms, 180 -> 2.5 ms.
        pulse_us = 500.0 + angle * (2000.0 / 180.0)
        return int(pulse_us * 65535.0 / 20000.0)

    def _write_angles(self, angle_x, angle_y):
        angle_x = self._clamp(
            angle_x,
            SERVO_X_MIN_DEG,
            SERVO_X_MAX_DEG
        )
        angle_y = self._clamp(
            angle_y,
            SERVO_Y_MIN_DEG,
            SERVO_Y_MAX_DEG
        )

        if self.pwm_x is not None:
            self.pwm_x.duty_u16(
                self._angle_to_duty_u16(angle_x)
            )

        if self.pwm_y is not None:
            self.pwm_y.duty_u16(
                self._angle_to_duty_u16(angle_y)
            )

        self.angle_x = angle_x
        self.angle_y = angle_y

    def update(self, motion):
        now_ms = time.ticks_ms()

        if motion is not None:
            self.last_seen_ms = now_ms
            error_x = (
                motion[0] - SERVO_TARGET_X
            ) / float(AI_FRAME_SIZE[0])
            error_y = (
                motion[1] - SERVO_TARGET_Y
            ) / float(AI_FRAME_SIZE[1])
            velocity_x = motion[2] / float(AI_FRAME_SIZE[0])
            velocity_y = motion[3] / float(AI_FRAME_SIZE[1])

            correction_x = (
                SERVO_X_KP * error_x
                + SERVO_X_KD * velocity_x
            )
            correction_y = (
                SERVO_Y_KP * error_y
                + SERVO_Y_KD * velocity_y
            )

            if SERVO_X_INVERT:
                correction_x = -correction_x
            if SERVO_Y_INVERT:
                correction_y = -correction_y

            target_x = SERVO_X_CENTER_DEG + correction_x
            target_y = SERVO_Y_CENTER_DEG + correction_y
        else:
            lost_ms = time.ticks_diff(
                now_ms,
                self.last_seen_ms
            )

            if (
                self.last_seen_ms
                and lost_ms < SERVO_LOST_HOLD_MS
            ):
                return self.angle_x, self.angle_y

            if (
                self.last_seen_ms
                and lost_ms < SERVO_LOST_CENTER_MS
            ):
                # Keep the last safe command while YOLO tries to reacquire.
                return self.angle_x, self.angle_y

            target_x = SERVO_X_CENTER_DEG
            target_y = SERVO_Y_CENTER_DEG

        if (
            time.ticks_diff(now_ms, self.last_update_ms)
            < SERVO_UPDATE_INTERVAL_MS
        ):
            return self.angle_x, self.angle_y

        self.last_update_ms = now_ms

        if self.enabled:
            try:
                self._write_angles(target_x, target_y)
            except Exception as error:
                print("舵机PWM运行失败，自动停用:", error)
                self.close(center_first=False)
        else:
            self.angle_x = self._clamp(
                target_x,
                SERVO_X_MIN_DEG,
                SERVO_X_MAX_DEG
            )
            self.angle_y = self._clamp(
                target_y,
                SERVO_Y_MIN_DEG,
                SERVO_Y_MAX_DEG
            )

        return self.angle_x, self.angle_y

    def close(self, center_first=True):
        if center_first and self.enabled:
            try:
                self._write_angles(
                    SERVO_X_CENTER_DEG,
                    SERVO_Y_CENTER_DEG
                )
                time.sleep_ms(150)
            except Exception:
                pass

        if self.pwm_x is not None:
            try:
                self.pwm_x.deinit()
            except Exception:
                pass

        if self.pwm_y is not None:
            try:
                self.pwm_y.deinit()
            except Exception:
                pass

        self.pwm_x = None
        self.pwm_y = None
        self.enabled = False


# ============================================================
# 钢珠识别与运动跟踪
# ============================================================

class SingleBallTracker:
    def __init__(self, kmodel_path):
        self.kpu = nn.kpu()
        self.kpu.load_kmodel(kmodel_path)
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_velocity_x = 0.0
        self.last_velocity_y = 0.0
        self.last_time_us = 0
        self.has_last_position = False
        self.lost_frames = 0

    @staticmethod
    def _is_pipe_inner_wall(pixels, x, y):
        x = int(x)
        y = int(y)

        if (
            x < 0
            or y < 0
            or x >= AI_FRAME_SIZE[0]
            or y >= AI_FRAME_SIZE[1]
        ):
            return False

        red = int(pixels[0, y, x])
        green = int(pixels[1, y, x])
        blue = int(pixels[2, y, x])

        return (
            red >= 35
            and green >= 30
            and red - blue >= 10
            and green - blue >= 5
            and red - green >= -5
        )

    def _has_pipe_context(
        self,
        pixels,
        center_x,
        center_y,
        width,
        height
    ):
        radius_x = int(width * 0.65) + 3
        radius_y = int(height * 0.65) + 3

        sample_points = (
            (center_x - radius_x, center_y),
            (center_x + radius_x, center_y),
            (center_x, center_y - radius_y),
            (center_x, center_y + radius_y),
            (
                center_x - radius_x,
                center_y - radius_y
            ),
            (
                center_x + radius_x,
                center_y - radius_y
            ),
            (
                center_x - radius_x,
                center_y + radius_y
            ),
            (
                center_x + radius_x,
                center_y + radius_y
            ),
        )

        pipe_hits = 0

        for x, y in sample_points:
            if self._is_pipe_inner_wall(pixels, x, y):
                pipe_hits += 1

        return pipe_hits >= 2

    def _select_candidate(self, output_data, input_np):
        scores = output_data[4, :]
        candidate_indices = np.argsort(
            scores,
            axis=0
        )[::-1]
        pixels = (
            input_np[0]
            if len(input_np.shape) == 4
            else input_np
        )
        minimum_score = (
            TRACK_CONFIDENCE
            if self.has_last_position
            else ACQUIRE_CONFIDENCE
        )

        checked = 0

        for candidate_index in candidate_indices:
            if checked >= MAX_CANDIDATES:
                break

            checked += 1
            index = int(candidate_index)
            score = float(scores[index])

            if score < minimum_score:
                break

            center_x = float(output_data[0, index])
            center_y = float(output_data[1, index])
            width = float(output_data[2, index])
            height = float(output_data[3, index])

            if (
                width < MIN_BALL_SIZE
                or height < MIN_BALL_SIZE
                or width > MAX_BALL_SIZE
                or height > MAX_BALL_SIZE
            ):
                continue

            aspect_ratio = width / height

            if (
                aspect_ratio < MIN_ASPECT_RATIO
                or aspect_ratio > MAX_ASPECT_RATIO
            ):
                continue

            if self.has_last_position:
                delta_x = center_x - self.last_x
                delta_y = center_y - self.last_y

                if (
                    delta_x * delta_x
                    + delta_y * delta_y
                    > MAX_TRACK_JUMP * MAX_TRACK_JUMP
                ):
                    continue

            if not self._has_pipe_context(
                pixels,
                center_x,
                center_y,
                width,
                height
            ):
                continue

            return center_x, center_y, score

        return None

    def run(self, input_np):
        input_tensor = nn.from_numpy(input_np)
        self.kpu.set_input_tensor(0, input_tensor)
        self.kpu.run()

        output_tensor = self.kpu.get_output_tensor(0)
        result = output_tensor.to_numpy()

        del output_tensor
        del input_tensor

        if len(result.shape) == 3:
            result = result.reshape(
                (
                    result.shape[0] * result.shape[1],
                    result.shape[2]
                )
            )

        if result.shape[0] <= result.shape[1]:
            output_data = result
        else:
            output_data = result.transpose()

        candidate = self._select_candidate(
            output_data,
            input_np
        )

        if candidate is None:
            self.lost_frames += 1

            if (
                PREDICT_SINGLE_MISSED_FRAME
                and self.has_last_position
                and self.lost_frames == 1
            ):
                now_us = time.ticks_us()
                delta_us = time.ticks_diff(
                    now_us,
                    self.last_time_us
                )

                if delta_us > 0:
                    velocity_x = max(
                        -MAX_PREDICT_SPEED,
                        min(
                            self.last_velocity_x,
                            MAX_PREDICT_SPEED
                        )
                    )
                    velocity_y = max(
                        -MAX_PREDICT_SPEED,
                        min(
                            self.last_velocity_y,
                            MAX_PREDICT_SPEED
                        )
                    )
                    delta_seconds = (
                        float(delta_us) / 1000000.0
                    )
                    predicted_x = max(
                        0.0,
                        min(
                            self.last_x
                            + velocity_x * delta_seconds,
                            float(AI_FRAME_SIZE[0] - 1)
                        )
                    )
                    predicted_y = max(
                        0.0,
                        min(
                            self.last_y
                            + velocity_y * delta_seconds,
                            float(AI_FRAME_SIZE[1] - 1)
                        )
                    )
                    return (
                        predicted_x,
                        predicted_y,
                        velocity_x,
                        velocity_y,
                        0.0,
                        now_us
                    )

            if self.lost_frames >= LOST_FRAMES_TO_UNLOCK:
                self.has_last_position = False

            return None

        center_x, center_y, best_score = candidate
        now_us = time.ticks_us()

        velocity_x = 0.0
        velocity_y = 0.0

        if self.has_last_position:
            delta_us = time.ticks_diff(
                now_us,
                self.last_time_us
            )

            if delta_us > 0:
                scale = 1000000.0 / float(delta_us)
                velocity_x = (
                    center_x - self.last_x
                ) * scale
                velocity_y = (
                    center_y - self.last_y
                ) * scale

        self.last_x = center_x
        self.last_y = center_y
        self.last_velocity_x = velocity_x
        self.last_velocity_y = velocity_y
        self.last_time_us = now_us
        self.has_last_position = True
        self.lost_frames = 0

        return (
            center_x,
            center_y,
            velocity_x,
            velocity_y,
            best_score,
            now_us
        )

    def deinit(self):
        del self.kpu
        gc.collect()
        nn.shrink_memory_pool()
        time.sleep_ms(50)


# ============================================================
# 统一三通道媒体管线
# ============================================================

class CombinedMediaPipeline:
    """统一拥有Sensor、Display、MediaManager、VENC和RTSP。"""

    def __init__(
        self,
        session_name=RTSP_SESSION,
        port=RTSP_PORT
    ):
        ensure_multimedia_loaded()

        self.session_name = session_name
        self.port = port
        self.video_type = mm.multi_media_type.media_h264
        self.enable_audio = False

        self.video_width = ALIGN_UP(VIDEO_WIDTH, 16)
        self.video_height = VIDEO_HEIGHT
        self.venc_chn = VENC_CHN_ID_0

        self.rtsp_server = mm.rtsp_server()
        self.sensor = None
        self.encoder = None
        self.encoder_link = None
        self.osd_img = None
        self.cur_frame = None

        self.display_initialized = False
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

        self.first_frame_ready = False
        self.encoded_frames = 0
        self.encoded_bytes = 0
        self.last_stream_ms = 0
        self.get_stream_failures = 0
        self.rtsp_send_failures = 0
        self.last_rtsp_send_error_ms = 0
        self.thread_error = None

        self.health_sample_ms = time.ticks_ms()
        self.health_sample_frames = 0
        self.health_sample_bytes = 0

    def _configure_media(self):
        print("正在初始化三通道摄像头媒体管线……")

        try:
            nn.shrink_memory_pool()
        except Exception:
            pass

        self.sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_INPUT_WIDTH,
            height=SENSOR_INPUT_HEIGHT,
            fps=VIDEO_FPS
        )
        self.sensor.reset()

        print("正在初始化ST7701显示屏……")
        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            osd_num=1,
            to_ide=False
        )
        self.display_initialized = True

        # 通道0只负责LCD底图。
        self.sensor.set_framesize(
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0
        )
        self.sensor.set_pixformat(
            PIXEL_FORMAT_YUV_SEMIPLANAR_420,
            chn=CAM_CHN_ID_0
        )

        display_bind_info = self.sensor.bind_info(
            x=0,
            y=0,
            chn=CAM_CHN_ID_0
        )
        Display.bind_layer(
            **display_bind_info,
            layer=Display.LAYER_VIDEO1
        )

        # 通道1只负责原始画面硬件编码，不经过OSD。
        self.sensor.set_framesize(
            width=self.video_width,
            height=self.video_height,
            chn=CAM_CHN_ID_1,
            alignment=12
        )
        self.sensor.set_pixformat(
            PIXEL_FORMAT_YUV_SEMIPLANAR_420,
            chn=CAM_CHN_ID_1
        )

        # 通道2只负责KPU输入。
        self.sensor.set_framesize(
            width=AI_FRAME_SIZE[0],
            height=AI_FRAME_SIZE[1],
            chn=CAM_CHN_ID_2
        )
        self.sensor.set_pixformat(
            PIXEL_FORMAT_RGB_888_PLANAR,
            chn=CAM_CHN_ID_2
        )

        self.osd_img = image.Image(
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            image.ARGB8888
        )

        print("正在初始化H.264编码器……")
        self.encoder = Encoder()
        self.encoder.SetOutBufs(
            self.venc_chn,
            VENC_BUFFER_COUNT,
            self.video_width,
            self.video_height
        )

        self.encoder_link = MediaManager.link(
            self.sensor.bind_info(
                chn=CAM_CHN_ID_1
            )["src"],
            (
                VIDEO_ENCODE_MOD_ID,
                VENC_DEV_ID,
                self.venc_chn
            )
        )

        print("正在初始化媒体缓冲区……")
        MediaManager.init()
        self.media_initialized = True

        # Display.show_image()首次调用时才会从公共VB池取得OSD显示
        # 缓冲；ST7701横屏还会额外取得一个旋转缓冲。三路Sensor和
        # VENC全部启动后再申请，在部分Hiwonder固件上会因为公共池
        # 已被媒体模块占用而报“get display buffer failed”。
        # 因此必须在Encoder.Create/Sensor.run之前用透明帧预留缓冲。
        self.osd_img.clear()
        Display.show_image(
            self.osd_img,
            0,
            0,
            Display.LAYER_OSD3
        )
        print("LCD OSD显示与旋转缓冲区预留完成")

        encoder_attribute = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_BASELINE,
            self.video_width,
            self.video_height,
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

        print("三通道媒体管线配置完成")
        print(
            "LCD通道:",
            DISPLAY_WIDTH,
            "x",
            DISPLAY_HEIGHT
        )
        print(
            "RTSP原始画面通道:",
            self.video_width,
            "x",
            self.video_height
        )
        print(
            "AI通道:",
            AI_FRAME_SIZE[0],
            "x",
            AI_FRAME_SIZE[1]
        )

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

    def _start_media_stream(self):
        print("正在启动H.264编码器……")
        self.encoder.Start(self.venc_chn)
        self.encoder_started = True

        print("正在启动摄像头……")
        self.sensor.run()
        self.sensor_started = True

        print("摄像头、LCD和编码器启动完成")

    def start(self):
        if self.server_started:
            return

        self.first_frame_ready = False
        self.encoded_frames = 0
        self.encoded_bytes = 0
        self.last_stream_ms = 0
        self.get_stream_failures = 0
        self.rtsp_send_failures = 0
        self.last_rtsp_send_error_ms = 0
        self.thread_error = None
        self.health_sample_ms = time.ticks_ms()
        self.health_sample_frames = 0
        self.health_sample_bytes = 0
        self.runthread_over = False

        try:
            self._configure_media()
            self._start_rtsp_server()
            self._start_media_stream()

            self.start_stream = True
            self.stream_thread_started = True

            _thread.start_new_thread(
                self._stream_thread,
                ()
            )

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
                        "%dms内没有取得H.264首帧，"
                        "GetStream失败次数=%d"
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

        except BaseException as error:
            self.start_stream = False

            if self.stream_thread_started:
                self._wait_stream_thread(
                    THREAD_STOP_TIMEOUT_MS
                )
            else:
                self.runthread_over = True

            self._release_resources()
            raise error

    def _stream_thread(self):
        stream_data = StreamData()

        try:
            while self.start_stream:
                try:
                    os.exitpoint()
                except Exception:
                    pass

                stream_received = False

                try:
                    result = self.encoder.GetStream(
                        self.venc_chn,
                        stream_data,
                        GET_STREAM_TIMEOUT_MS
                    )

                    if result != 0:
                        self.get_stream_failures += 1
                        continue

                    stream_received = True
                    frame_bytes = 0

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

                        frame_bytes += packet_size
                        try:
                            self.rtsp_server.rtspserver_sendvideodata(
                                self.session_name,
                                packet,
                                packet_size,
                                1000
                            )
                        except Exception as error:
                            # VLC closing or Wi-Fi roaming may reset the RTSP
                            # socket. Keep draining VENC so LCD/AI/servo never
                            # stop; the RTSP server accepts a later reconnect.
                            self.rtsp_send_failures += 1
                            now_ms = time.ticks_ms()
                            if (
                                not self.last_rtsp_send_error_ms
                                or time.ticks_diff(
                                    now_ms,
                                    self.last_rtsp_send_error_ms
                                )
                                >= HEALTH_PRINT_INTERVAL_MS
                            ):
                                self.last_rtsp_send_error_ms = now_ms
                                print(
                                    "RTSP客户端暂时断开，"
                                    "LCD/识别/舵机继续:",
                                    error
                                )

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

    def stop_stream_thread(self):
        """先停止编码读取线程，供主程序在释放KPU前调用。"""
        self.start_stream = False

        if self.stream_thread_started:
            if not self._wait_stream_thread(
                THREAD_STOP_TIMEOUT_MS
            ):
                print(
                    "警告：RTSP发送线程未在规定时间内退出"
                )

    def get_frame(self):
        self.cur_frame = self.sensor.snapshot(
            chn=CAM_CHN_ID_2
        )
        return self.cur_frame.to_numpy_ref()

    def show_image(self):
        Display.show_image(
            self.osd_img,
            0,
            0,
            Display.LAYER_OSD3
        )

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
        kbps = (
            byte_delta
            * 8.0
            / elapsed_ms
        )

        print(
            "[HEALTH] fps=%.1f bitrate=%.0fkbps "
            "frames=%d bytes=%d get_fail=%d send_fail=%d thread=%s"
            % (
                fps,
                kbps,
                self.encoded_frames,
                self.encoded_bytes,
                self.get_stream_failures,
                self.rtsp_send_failures,
                (
                    "RUN"
                    if not self.runthread_over
                    else "STOP"
                )
            )
        )

        self.health_sample_ms = now
        self.health_sample_frames = self.encoded_frames
        self.health_sample_bytes = self.encoded_bytes

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

    def _release_resources(self):
        print("正在释放识别与图传资源……")

        # 清理阶段禁止新的IDE退出点异常打断资源释放。
        try:
            os.exitpoint(
                os.EXITPOINT_ENABLE_SLEEP
            )
        except Exception:
            pass

        self.cur_frame = None
        self.osd_img = None
        gc.collect()

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

        if self.encoder_link is not None:
            try:
                link_object = self.encoder_link
                self.encoder_link = None
                del link_object
                gc.collect()
                print("摄像头通道1与编码器绑定已解除")
            except Exception as error:
                print("解除编码器绑定失败:", error)

        if (
            self.encoder_started
            and self.encoder is not None
        ):
            try:
                self.encoder.Stop(self.venc_chn)
                print("H.264编码器已停止")
            except Exception as error:
                print("停止编码器失败:", error)

            self.encoder_started = False

        if (
            self.encoder_created
            and self.encoder is not None
        ):
            try:
                self.encoder.Destroy(self.venc_chn)
                print("H.264编码器已销毁")
            except Exception as error:
                print("销毁编码器失败:", error)

            self.encoder_created = False

        if self.display_initialized:
            try:
                Display.deinit()
                print("显示屏已反初始化")
            except Exception as error:
                print("反初始化显示屏失败:", error)

            self.display_initialized = False

        if self.media_initialized:
            try:
                MediaManager.deinit()
                print("媒体缓冲区已释放")
            except Exception as error:
                print("释放媒体缓冲区失败:", error)

            self.media_initialized = False

        if self.rtsp_started:
            try:
                self.rtsp_server.rtspserver_stop()
                print("RTSP服务已停止")
            except Exception as error:
                print("停止RTSP服务失败:", error)

            self.rtsp_started = False

        if self.session_created:
            try:
                self.rtsp_server.rtspserver_destroysession(
                    self.session_name
                )
                print("RTSP会话已销毁")
            except Exception as error:
                print("销毁RTSP会话失败:", error)

            self.session_created = False

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
        self.runthread_over = True

        gc.collect()
        print("识别与图传资源释放完成")

    def stop(self):
        self.stop_stream_thread()
        self._release_resources()


# ============================================================
# LCD识别标记
# ============================================================

def draw_marker(pipeline, motion):
    global last_marker_update_ms
    global display_marker_enabled
    global display_marker_error_reported
    global display_marker_last_error_ms

    if not display_marker_enabled:
        if (
            time.ticks_diff(
                time.ticks_ms(),
                display_marker_last_error_ms
            )
            < DISPLAY_MARKER_RETRY_MS
        ):
            return
        display_marker_enabled = True

    pipeline.osd_img.clear()

    if motion is not None:
        center_x = int(
            motion[0]
            * DISPLAY_WIDTH
            // AI_FRAME_SIZE[0]
        )
        center_y = int(
            motion[1]
            * DISPLAY_HEIGHT
            // AI_FRAME_SIZE[1]
        )

        left = center_x - MARKER_SIZE
        top = center_y - MARKER_SIZE

        if left < 0:
            left = 0

        if top < 0:
            top = 0

        marker_width = MARKER_SIZE * 2
        marker_height = MARKER_SIZE * 2

        if left + marker_width >= DISPLAY_WIDTH:
            marker_width = DISPLAY_WIDTH - left - 1

        if top + marker_height >= DISPLAY_HEIGHT:
            marker_height = DISPLAY_HEIGHT - top - 1

        pipeline.osd_img.draw_rectangle(
            left,
            top,
            marker_width,
            marker_height,
            color=MARKER_COLOR,
            thickness=2
        )

    try:
        pipeline.show_image()
    except RuntimeError as error:
        # The camera video layer is hardware-bound and remains visible.
        # A transient OSD/VB failure must not tear down LCD, AI, RTSP or PWM.
        display_marker_enabled = False
        display_marker_last_error_ms = time.ticks_ms()
        if not display_marker_error_reported:
            display_marker_error_reported = True
            print(
                "LCD检测框OSD缓冲异常，将自动重试；"
                "摄像头/识别/图传/舵机继续:",
                error
            )


def print_startup_summary(
    pipeline,
    manual_url,
    restart_count
):
    official_url = pipeline.get_rtsp_url()

    print()
    print("========================================")
    print("钢珠识别与原始画面无线图传启动成功")
    print("编码首帧自检: PASS")
    print()
    print("LCD:")
    print(
        DISPLAY_WIDTH,
        "x",
        DISPLAY_HEIGHT,
        "钢珠识别标记"
    )
    print()
    print("RTSP:")
    print(
        VIDEO_WIDTH,
        "x",
        VIDEO_HEIGHT,
        "@",
        VIDEO_FPS,
        "FPS，无识别标记"
    )
    print("码率:", VIDEO_BIT_RATE, "kbps")
    print("GOP:", VIDEO_GOP)
    print("VENC缓冲:", VENC_BUFFER_COUNT)
    print("UDP坐标端口:", UDP_PORT)
    print(
        "舵机PWM:",
        "已启用" if SERVO_ENABLED else "安全关闭"
    )
    print()
    print("电脑连接热点:")
    print("热点名称:", AP_SSID)
    print("热点密码:", AP_PASSWORD)
    print()
    print("VLC网络串流地址:")
    print(manual_url)
    print()
    print("VLC建议命令:")
    print(
        'vlc --rtsp-tcp --network-caching=100 "%s"'
        % manual_url
    )

    if official_url:
        print("模块返回地址:", official_url)

    print("自动恢复次数:", restart_count)
    print("按Ctrl+C或IDE停止按钮结束")
    print("========================================")
    print()


# ============================================================
# 主程序
# ============================================================

def main():
    global display_marker_enabled
    global display_marker_error_reported
    global display_marker_last_error_ms

    ap = None
    ap_ip = None
    pipeline = None
    tracker = None
    udp = None
    servo = None
    restart_count = 0

    try:
        log_event(
            "合并程序开始执行 version=%s"
            % SCRIPT_VERSION
        )
        print(
            "冷启动等待%dms，让Wi-Fi和媒体驱动完成初始化……"
            % BOOT_SETTLE_MS
        )
        time.sleep_ms(BOOT_SETTLE_MS)

        try:
            os.exitpoint(
                os.EXITPOINT_ENABLE
            )
        except Exception:
            pass

        ap, ap_ip = start_ap()
        udp = UdpBallSender(ap_ip)
        servo = BallServoController()
        wait_for_ap_client(ap)

        manual_url = (
            "rtsp://{}:{}/{}"
            .format(
                ap_ip,
                RTSP_PORT,
                RTSP_SESSION
            )
        )

        while True:
            pipeline = None
            tracker = None
            frame_count = 0
            total_frame_count = 0
            detected_count = 0
            last_motion = None
            servo_angles = (
                SERVO_X_CENTER_DEG,
                SERVO_Y_CENTER_DEG
            )
            last_health_print_ms = 0

            try:
                # _release_resources()会在清理期间切换为SLEEP；
                # 每次新建媒体管线前重新允许IDE停止程序。
                try:
                    os.exitpoint(
                        os.EXITPOINT_ENABLE
                    )
                except Exception:
                    pass

                if restart_count:
                    print()
                    print(
                        "正在第%d次重建识别与RTSP媒体链路……"
                        % restart_count
                    )

                pipeline = CombinedMediaPipeline(
                    session_name=RTSP_SESSION,
                    port=RTSP_PORT
                )
                pipeline.start()
                display_marker_enabled = True
                display_marker_error_reported = False
                display_marker_last_error_ms = 0

                tracker = SingleBallTracker(
                    KMODEL_PATH
                )

                print_startup_summary(
                    pipeline,
                    manual_url,
                    restart_count
                )

                log_event(
                    "识别和RTSP首帧自检通过 "
                    "url=%s restart=%d"
                    % (
                        manual_url,
                        restart_count
                    )
                )
                last_health_print_ms = time.ticks_ms()

                while True:
                    try:
                        os.exitpoint()
                    except KeyboardInterrupt:
                        raise
                    except Exception:
                        pass

                    input_np = pipeline.get_frame()
                    motion = tracker.run(input_np)
                    draw_marker(pipeline, motion)
                    udp.send(motion)
                    servo_angles = servo.update(motion)

                    frame_count += 1
                    total_frame_count += 1
                    if motion is not None:
                        detected_count += 1
                        last_motion = motion

                    if total_frame_count % GC_INTERVAL == 0:
                        gc.collect()

                    health_problem = pipeline.health_error()

                    if health_problem:
                        raise RuntimeError(
                            "媒体链路异常: %s"
                            % health_problem
                        )

                    now = time.ticks_ms()

                    if (
                        time.ticks_diff(
                            now,
                            last_health_print_ms
                        )
                        >= HEALTH_PRINT_INTERVAL_MS
                    ):
                        pipeline.print_health()
                        if last_motion is None:
                            print(
                                "[CONTROL] ball=LOST "
                                "servo=(%.1f,%.1f) udp=%d fail=%d"
                                % (
                                    servo_angles[0],
                                    servo_angles[1],
                                    udp.sent,
                                    udp.failures
                                )
                            )
                        else:
                            print(
                                "[CONTROL] detected=%d/%d "
                                "ball=(%.1f,%.1f) "
                                "velocity=(%.1f,%.1f) "
                                "servo=(%.1f,%.1f) udp=%d"
                                % (
                                    detected_count,
                                    frame_count,
                                    last_motion[0],
                                    last_motion[1],
                                    last_motion[2],
                                    last_motion[3],
                                    servo_angles[0],
                                    servo_angles[1],
                                    udp.sent
                                )
                            )
                        frame_count = 0
                        detected_count = 0
                        last_motion = None
                        last_health_print_ms = now

            except KeyboardInterrupt:
                raise

            except BaseException as error:
                try:
                    log_event(
                        "识别或媒体链路失败 "
                        "restart=%d error=%s"
                        % (
                            restart_count,
                            error
                        )
                    )
                except Exception:
                    pass

                show_exception(
                    error,
                    "main.识别与RTSP运行"
                )
                restart_count += 1

            finally:
                # 先停止RTSP线程，再释放KPU，最后统一释放媒体资源。
                if pipeline is not None:
                    try:
                        pipeline.stop_stream_thread()
                    except BaseException as error:
                        show_exception(
                            error,
                            "main.stop_stream_thread"
                        )

                if tracker is not None:
                    try:
                        tracker.deinit()
                    except BaseException as error:
                        show_exception(
                            error,
                            "main.tracker.deinit"
                        )

                tracker = None

                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except BaseException as error:
                        show_exception(
                            error,
                            "main.pipeline.stop"
                        )

                pipeline = None
                gc.collect()

            print(
                "%dms后重建识别与RTSP媒体链路……"
                % RESTART_DELAY_MS
            )
            delay_with_exitpoint(RESTART_DELAY_MS)

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
        # 正常情况下内层finally已完成清理；以下代码保证启动中断也能释放。
        if pipeline is not None:
            try:
                pipeline.stop_stream_thread()
            except BaseException as error:
                show_exception(
                    error,
                    "main.finally.stop_stream_thread"
                )

        if tracker is not None:
            try:
                tracker.deinit()
            except BaseException as error:
                show_exception(
                    error,
                    "main.finally.tracker.deinit"
                )

        if pipeline is not None:
            try:
                pipeline.stop()
            except BaseException as error:
                show_exception(
                    error,
                    "main.finally.pipeline.stop"
                )

        if servo is not None:
            servo.close()

        if udp is not None:
            udp.close()

        tracker = None
        pipeline = None
        servo = None
        udp = None
        ap = None
        gc.collect()

        log_event("程序结束")
        print("程序结束")


if __name__ == "__main__":
    main()
