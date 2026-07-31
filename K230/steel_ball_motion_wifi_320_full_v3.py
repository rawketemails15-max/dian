# ============================================================
# K230 钢珠识别 + LCD标记 + 原始画面H.264 RTSP无线图传
#
# 适用固件：
#   CanMV v1.4-19
#   k230_canmv_hiwonder
#
# 三路摄像头输出：
#   通道0：800x480 YUV420 -> ST7701 LCD视频层
#   通道1：低延迟预设分辨率 YUV420 -> H.264 VENC -> RTSP（无识别标记）
#   通道2：320x320 RGB888P -> KPU钢珠识别
#
# 电脑播放地址：
#   rtsp://K230_IP:8554/ball
# VLC直连热点最低延迟优先使用UDP：
#   vlc --network-caching=50 --live-caching=50 --clock-jitter=0
#       --clock-synchro=0 --drop-late-frames --skip-frames
#       rtsp://K230_IP:8554/ball
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
from machine import FPIOA, TOUCH, UART


# 媒体管线在AP就绪后立即运行；RTSP等电脑或手机接入热点后再启用。
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
AP_CLIENT_SETTLE_MS = 500
AP_STATUS_FALLBACK_SETTLE_MS = 8000
SCRIPT_VERSION = "steel-ball-full-v3-touch-target-uart-ack-v1"
BOOT_LOG_PATH = "/sdcard/wifi_rtsp.log"
BOOT_LOG_MAX_BYTES = 32768


# ============================================================
# RTSP与H.264配置
# ============================================================

RTSP_PORT = 8554
RTSP_SESSION = "ball"

# 只改这一项即可切换图传档位：
#   0 = 原始稳定参数
#   1 = 安全低延迟（当前默认，保持已验证分辨率/帧率/缓冲数）
#   2 = 激进低延迟（确认档位1能运行后再试）
RTSP_LOW_LATENCY_PRESET = 2

if RTSP_LOW_LATENCY_PRESET == 0:
    VIDEO_WIDTH = 640
    VIDEO_HEIGHT = 480
    VIDEO_FPS = 30
    VIDEO_BIT_RATE = 1500
    VIDEO_GOP = 10
    VENC_BUFFER_COUNT = 4
    DISPLAY_MARKER_INTERVAL_MS = 33
elif RTSP_LOW_LATENCY_PRESET == 1:
    VIDEO_WIDTH = 640
    VIDEO_HEIGHT = 480
    VIDEO_FPS = 30
    VIDEO_BIT_RATE = 1200
    VIDEO_GOP = 5
    VENC_BUFFER_COUNT = 4
    DISPLAY_MARKER_INTERVAL_MS = 66
elif RTSP_LOW_LATENCY_PRESET == 2:
    VIDEO_WIDTH = 480
    VIDEO_HEIGHT = 360
    VIDEO_FPS = 20
    VIDEO_BIT_RATE = 650
    VIDEO_GOP = 4
    VENC_BUFFER_COUNT = 2
    DISPLAY_MARKER_INTERVAL_MS = 100
else:
    raise ValueError("RTSP_LOW_LATENCY_PRESET必须是0、1或2")

# Sensor/KPU继续以30 FPS运行；只让VENC按VIDEO_FPS抽帧，
# 避免为了图传降帧而降低钢珠识别和UART控制频率。
SENSOR_FPS = 30

GET_STREAM_TIMEOUT_MS = 200
FIRST_FRAME_TIMEOUT_MS = 6000
STREAM_STALL_TIMEOUT_MS = 5000
HEALTH_PRINT_INTERVAL_MS = 2000
THREAD_STOP_TIMEOUT_MS = 3000
RESTART_DELAY_MS = 2000
RTSP_SLOW_SEND_MS = 100
# K230 VENC的pts以微秒计，原生RTSP接口的timestamp以毫秒计。
VENC_PTS_UNITS_PER_RTSP_MS = 1000


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
CENTER_LINE_COLOR = (255, 255, 0, 0)
OSD_TEXT_COLOR = (255, 255, 255, 255)
GC_INTERVAL = 300
DISPLAY_MARKER_RETRY_MS = 1000


# ============================================================
# Coordinate output and optional two-axis servo control
# ============================================================

# Every valid/predicted ball position is broadcast to the controller PC:
# BALL,x,y,vx,vy,confidence,timestamp_us
UDP_ENABLED = True
UDP_PORT = 9000

# K230 UART1: IO3/TX -> MSPM0 PB7/RX, IO4/RX <- MSPM0 PB6/TX.
UART_TX_IO = 3
UART_RX_IO = 4
UART_BAUD_RATE = 115200
UART_SEND_INTERVAL_MS = 33
UART_PROTOCOL_VERSION = 0x01
UART_MESSAGE_BALL_X = 0x03
UART_MESSAGE_TARGET_X = 0x04
UART_MESSAGE_STATUS = 0x83
UART_STATUS_PAYLOAD_SIZE = 45
UART_STATUS_FRAME_SIZE = 52
UART_TARGET_SEND_INTERVAL_MS = 33
UART_TARGET_RETRY_MS = 100
UART_TARGET_HEARTBEAT_MS = 500
UART_STATUS_STALE_MS = 300

BALL_TARGET_DEFAULT_X = 171
BALL_TARGET_DEFAULT_X_Q4 = BALL_TARGET_DEFAULT_X * 16
BALL_TARGET_MIN_X_Q4 = 0
BALL_TARGET_MAX_X_Q4 = 319 * 16

TOUCH_LINE_HIT_HALF_WIDTH = 30
TOUCH_RELEASE_EMPTY_FRAMES = 2
TOUCH_TARGET_DEADBAND_Q4 = 8
TOUCH_RETRY_MS = 1000

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
    """Return the AP station count, or None when the firmware cannot report it."""
    try:
        stations = ap.status("stations")

        if stations is None:
            return 0

        try:
            return len(stations)
        except Exception:
            return 1 if stations else 0

    except Exception:
        # Some Hiwonder firmware builds only expose isconnected() in AP mode.
        try:
            return 1 if ap.isconnected() else 0
        except Exception:
            return None


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


def clamp_integer(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def display_x_to_target_q4(display_x):
    display_x = clamp_integer(
        int(display_x),
        0,
        DISPLAY_WIDTH - 1
    )
    target_q4 = (
        display_x * AI_FRAME_SIZE[0] * 16
        + DISPLAY_WIDTH // 2
    ) // DISPLAY_WIDTH
    return clamp_integer(
        target_q4,
        BALL_TARGET_MIN_X_Q4,
        BALL_TARGET_MAX_X_Q4
    )


def target_q4_to_display_x(target_q4):
    target_q4 = clamp_integer(
        int(target_q4),
        BALL_TARGET_MIN_X_Q4,
        BALL_TARGET_MAX_X_Q4
    )
    display_x = (
        target_q4 * DISPLAY_WIDTH
        + AI_FRAME_SIZE[0] * 8
    ) // (AI_FRAME_SIZE[0] * 16)
    return clamp_integer(
        display_x,
        0,
        DISPLAY_WIDTH - 1
    )


def crc16_ccitt_false(data, start, length):
    crc = 0xFFFF

    for index in range(start, start + length):
        crc ^= data[index] << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_target_frame(sequence, target_x_q4):
    target_x_q4 = clamp_integer(
        int(target_x_q4),
        BALL_TARGET_MIN_X_Q4,
        BALL_TARGET_MAX_X_Q4
    )
    frame = bytearray(11)
    frame[0] = 0xA5
    frame[1] = 0x5A
    frame[2] = UART_PROTOCOL_VERSION
    frame[3] = UART_MESSAGE_TARGET_X
    frame[4] = 4
    frame[5] = sequence & 0xFF
    frame[6] = (sequence >> 8) & 0xFF
    frame[7] = target_x_q4 & 0xFF
    frame[8] = (target_x_q4 >> 8) & 0xFF
    crc = crc16_ccitt_false(frame, 2, 7)
    frame[9] = crc & 0xFF
    frame[10] = (crc >> 8) & 0xFF
    return frame


class DraggableTargetLine:
    """Capture one touch gesture and map LCD X to the 320-pixel target."""

    STATE_IDLE = 0
    STATE_DRAGGING = 1
    STATE_IGNORED = 2

    def __init__(self, target_x_q4=BALL_TARGET_DEFAULT_X_Q4):
        self.target_x_q4 = clamp_integer(
            int(target_x_q4),
            BALL_TARGET_MIN_X_Q4,
            BALL_TARGET_MAX_X_Q4
        )
        self.state = self.STATE_IDLE
        self.dragging = False
        self.empty_frames = 0

    def display_x(self):
        return target_q4_to_display_x(self.target_x_q4)

    def update(self, points):
        changed = False
        committed = False
        touching = bool(points)

        if touching:
            touch_x = clamp_integer(
                int(points[0].x),
                0,
                DISPLAY_WIDTH - 1
            )
            self.empty_frames = 0

            if self.state == self.STATE_IDLE:
                if (
                    abs(touch_x - self.display_x())
                    <= TOUCH_LINE_HIT_HALF_WIDTH
                ):
                    self.state = self.STATE_DRAGGING
                    self.dragging = True
                else:
                    self.state = self.STATE_IGNORED
                    self.dragging = False

            if self.state == self.STATE_DRAGGING:
                new_target_q4 = display_x_to_target_q4(
                    touch_x
                )
                if (
                    abs(new_target_q4 - self.target_x_q4)
                    >= TOUCH_TARGET_DEADBAND_Q4
                ):
                    self.target_x_q4 = new_target_q4
                    changed = True
            return changed, committed

        if self.state != self.STATE_IDLE:
            self.empty_frames += 1
            if self.empty_frames >= TOUCH_RELEASE_EMPTY_FRAMES:
                committed = self.state == self.STATE_DRAGGING
                self.state = self.STATE_IDLE
                self.dragging = False
                self.empty_frames = 0
        return changed, committed


class MspStatusParser:
    """Incrementally parse the existing 52-byte MSPM0 status frame."""

    def __init__(self):
        self.frame = bytearray(UART_STATUS_FRAME_SIZE)
        self.index = 0
        self.raw_bytes = 0
        self.valid_frames = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.sequence = 0
        self.target_x_q4 = None
        self.last_status_ms = 0

    def _resync(self, value=0):
        if value == 0xA5:
            self.frame[0] = value
            self.index = 1
        else:
            self.index = 0

    def feed(self, data, now_ms):
        if not data:
            return

        for value in data:
            self.raw_bytes += 1
            if self.index == 0:
                if value == 0xA5:
                    self.frame[0] = value
                    self.index = 1
                continue
            if self.index == 1:
                if value == 0x5A:
                    self.frame[1] = value
                    self.index = 2
                else:
                    self._resync(value)
                continue

            self.frame[self.index] = value
            self.index += 1
            if self.index == 5:
                if (
                    self.frame[2] != UART_PROTOCOL_VERSION
                    or self.frame[3] != UART_MESSAGE_STATUS
                    or self.frame[4] != UART_STATUS_PAYLOAD_SIZE
                ):
                    self.format_errors += 1
                    self._resync(value)
                    continue

            if self.index == UART_STATUS_FRAME_SIZE:
                received_crc = (
                    self.frame[50]
                    | (self.frame[51] << 8)
                )
                calculated_crc = crc16_ccitt_false(
                    self.frame,
                    2,
                    48
                )
                if received_crc == calculated_crc:
                    self.sequence = (
                        self.frame[5]
                        | (self.frame[6] << 8)
                    )
                    self.target_x_q4 = (
                        self.frame[41]
                        | (self.frame[42] << 8)
                    )
                    self.last_status_ms = now_ms
                    self.valid_frames += 1
                else:
                    self.crc_errors += 1
                self._resync(value)


class BallUartSender:
    """Bidirectional MSPM0 link for vision, target and status ACK frames."""

    def __init__(self):
        self.uart = None
        self.vision_sequence = 0
        self.target_sequence = 0
        self.last_send_ms = time.ticks_add(
            time.ticks_ms(),
            -UART_SEND_INTERVAL_MS
        )
        self.last_target_send_ms = time.ticks_add(
            time.ticks_ms(),
            -UART_TARGET_HEARTBEAT_MS
        )
        self.start_ms = time.ticks_ms()
        self.requested_target_x_q4 = BALL_TARGET_DEFAULT_X_Q4
        self.target_pending = True
        self.target_force = True
        self.status_parser = MspStatusParser()
        self.sent = 0
        self.target_sent = 0
        self.failures = 0
        self.rx_failures = 0

        try:
            fpioa = FPIOA()
            fpioa.set_function(
                UART_TX_IO,
                FPIOA.UART1_TXD,
                ie=0,
                oe=1,
                pu=0,
                pd=0
            )
            fpioa.set_function(
                UART_RX_IO,
                FPIOA.UART1_RXD,
                ie=1,
                oe=0,
                pu=1,
                pd=0,
                st=1
            )
            try:
                fpioa.help(UART_TX_IO)
                fpioa.help(UART_RX_IO)
            except Exception as error:
                print("UART1 FPIOA状态读取失败:", error)
            self.uart = UART(
                UART.UART1,
                baudrate=UART_BAUD_RATE,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE,
                timeout=2
            )
            print("UART1 ready: IO3 TX, IO4 RX, 115200 8N1")
        except Exception as error:
            self.uart = None
            print("UART1坐标输出关闭:", error)

    def _write_frame(self, frame, is_target=False):
        if self.uart is None:
            return False
        try:
            self.uart.write(frame)
            if is_target:
                self.target_sent += 1
            else:
                self.sent += 1
            return True
        except Exception:
            self.failures += 1
            return False

    def send(self, motion):
        now_ms = time.ticks_ms()
        if (
            time.ticks_diff(now_ms, self.last_send_ms)
            < UART_SEND_INTERVAL_MS
        ):
            return
        self.last_send_ms = now_ms

        flags = 0
        confidence = 0
        x_q4 = 0
        velocity_x = 0

        if motion is not None:
            flags = 0x01
            if motion[4] <= 0.0:
                flags |= 0x02
            confidence = clamp_integer(
                int(motion[4] * 255.0 + 0.5),
                0,
                255
            )
            x_q4 = clamp_integer(
                int(motion[0] * 16.0 + 0.5),
                0,
                0xFFFF
            )
            velocity_x = clamp_integer(
                int(motion[2]),
                -32768,
                32767
            )

        frame = bytearray(15)
        frame[0] = 0xA5
        frame[1] = 0x5A
        frame[2] = UART_PROTOCOL_VERSION
        frame[3] = UART_MESSAGE_BALL_X
        frame[4] = 8
        frame[5] = self.vision_sequence & 0xFF
        frame[6] = (self.vision_sequence >> 8) & 0xFF
        frame[7] = flags
        frame[8] = confidence
        frame[9] = x_q4 & 0xFF
        frame[10] = (x_q4 >> 8) & 0xFF
        frame[11] = velocity_x & 0xFF
        frame[12] = (velocity_x >> 8) & 0xFF
        crc = crc16_ccitt_false(frame, 2, 11)
        frame[13] = crc & 0xFF
        frame[14] = (crc >> 8) & 0xFF

        self.vision_sequence = (
            self.vision_sequence + 1
        ) & 0xFFFF
        self._write_frame(frame)

    def set_target(self, target_x_q4, force=False):
        target_x_q4 = clamp_integer(
            int(target_x_q4),
            BALL_TARGET_MIN_X_Q4,
            BALL_TARGET_MAX_X_Q4
        )
        if target_x_q4 != self.requested_target_x_q4:
            self.requested_target_x_q4 = target_x_q4
            self.target_pending = True
        if force:
            self.target_pending = True
            self.target_force = True

    def request_target_resend(self):
        self.target_pending = True
        self.target_force = True

    def _send_target_frame(self, now_ms):
        frame = build_target_frame(
            self.target_sequence,
            self.requested_target_x_q4
        )

        self.target_sequence = (
            self.target_sequence + 1
        ) & 0xFFFF
        self.last_target_send_ms = now_ms
        self.target_pending = False
        self.target_force = False
        self._write_frame(frame, is_target=True)

    def target_acknowledged(self, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        parser = self.status_parser
        if parser.last_status_ms == 0:
            return False
        if (
            time.ticks_diff(now_ms, parser.last_status_ms)
            > UART_STATUS_STALE_MS
        ):
            return False
        return parser.target_x_q4 == self.requested_target_x_q4

    def target_ack_text(self, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        parser = self.status_parser
        if self.uart is None:
            return "LINK LOST"
        if parser.last_status_ms == 0:
            if (
                time.ticks_diff(now_ms, self.start_ms)
                > UART_STATUS_STALE_MS
            ):
                return "LINK LOST"
            return "WAIT"
        if (
            time.ticks_diff(now_ms, parser.last_status_ms)
            > UART_STATUS_STALE_MS
        ):
            return "LINK LOST"
        if parser.target_x_q4 == self.requested_target_x_q4:
            return "ACK"
        return "WAIT"

    def service_target(self, force=False):
        now_ms = time.ticks_ms()
        if force:
            self.target_pending = True
            self.target_force = True

        elapsed_ms = time.ticks_diff(
            now_ms,
            self.last_target_send_ms
        )
        should_send = self.target_force
        if (
            not should_send
            and self.target_pending
            and elapsed_ms >= UART_TARGET_SEND_INTERVAL_MS
        ):
            should_send = True
        if not should_send and not self.target_acknowledged(now_ms):
            should_send = elapsed_ms >= UART_TARGET_RETRY_MS
        if not should_send and self.target_acknowledged(now_ms):
            should_send = elapsed_ms >= UART_TARGET_HEARTBEAT_MS

        if should_send:
            self._send_target_frame(now_ms)

    def poll_status(self):
        if self.uart is None:
            return
        try:
            data = self.uart.read()
            if data:
                self.status_parser.feed(
                    data,
                    time.ticks_ms()
                )
        except Exception:
            self.rx_failures += 1

    def close(self):
        if self.uart is not None:
            try:
                self.uart.deinit()
            except Exception:
                pass
        self.uart = None


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
        self.rtsp_feed_enabled = False
        self.rtsp_wait_started_ms = 0
        self.rtsp_client_seen_ms = None
        self.rtsp_last_poll_ms = 0
        self.rtsp_status_fallback_logged = False

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
        self.rtsp_sent_frames = 0
        self.rtsp_discarded_frames = 0
        self.rtsp_timestamp_origin_ms = 0
        self.rtsp_last_timestamp_ms = 0
        self.rtsp_last_send_duration_ms = 0
        self.rtsp_max_send_duration_ms = 0
        self.rtsp_slow_send_count = 0
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
            fps=SENSOR_FPS
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
            SENSOR_FPS,
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

    def poll_rtsp_client(self, ap):
        """Non-blocking AP-client gate for RTSP while vision keeps running."""
        now_ms = time.ticks_ms()

        if (
            time.ticks_diff(
                now_ms,
                self.rtsp_last_poll_ms
            )
            < AP_CLIENT_POLL_MS
        ):
            return False

        self.rtsp_last_poll_ms = now_ms
        station_count = get_ap_station_count(ap)

        if station_count is None:
            if not self.rtsp_status_fallback_logged:
                self.rtsp_status_fallback_logged = True
                print(
                    "固件无法查询热点客户端，"
                    "%dms后按兼容模式启用RTSP"
                    % AP_STATUS_FALLBACK_SETTLE_MS
                )
                log_event(
                    "固件无法查询AP客户端，等待%dms后启用RTSP"
                    % AP_STATUS_FALLBACK_SETTLE_MS
                )

            if (
                not self.rtsp_feed_enabled
                and time.ticks_diff(
                    now_ms,
                    self.rtsp_wait_started_ms
                )
                >= AP_STATUS_FALLBACK_SETTLE_MS
            ):
                if not self.rtsp_started:
                    self._start_rtsp_server()
                self.rtsp_feed_enabled = True
                print("RTSP兼容模式已启用，从当前编码帧开始发送")
                log_event("RTSP兼容模式已启用，从当前编码帧开始发送")
                return True

            return False

        self.rtsp_status_fallback_logged = False

        if station_count <= 0:
            self.rtsp_client_seen_ms = None

            if self.rtsp_feed_enabled:
                # The encoder thread keeps draining VENC while disconnected.
                # This prevents old H.264 frames from accumulating for replay.
                self.rtsp_feed_enabled = False
                print("热点客户端已断开，暂停RTSP投递并丢弃旧编码帧")
                log_event("热点客户端已断开，RTSP暂停投递")

            return False

        if self.rtsp_feed_enabled:
            return False

        if self.rtsp_client_seen_ms is None:
            self.rtsp_client_seen_ms = now_ms
            print(
                "检测到热点客户端%d个，等待网络稳定……"
                % station_count
            )
            log_event(
                "检测到AP客户端%d个，等待网络稳定"
                % station_count
            )
            return False

        if (
            time.ticks_diff(
                now_ms,
                self.rtsp_client_seen_ms
            )
            < AP_CLIENT_SETTLE_MS
        ):
            return False

        if not self.rtsp_started:
            self._start_rtsp_server()

        # Set this only after the RTSP session is completely ready. The
        # encoder thread cannot feed a half-initialized network service.
        self.rtsp_feed_enabled = True
        official_url = self.get_rtsp_url()
        print("RTSP已启用，从当前编码帧开始发送")
        if official_url:
            print("RTSP地址:", official_url)
        log_event(
            "RTSP已启用，从当前编码帧开始发送 url=%s"
            % (official_url if official_url else "unknown")
        )
        return True

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
        self.rtsp_sent_frames = 0
        self.rtsp_discarded_frames = 0
        self.rtsp_timestamp_origin_ms = 0
        self.rtsp_last_timestamp_ms = 0
        self.rtsp_last_send_duration_ms = 0
        self.rtsp_max_send_duration_ms = 0
        self.rtsp_slow_send_count = 0
        self.last_rtsp_send_error_ms = 0
        self.thread_error = None
        self.health_sample_ms = time.ticks_ms()
        self.health_sample_frames = 0
        self.health_sample_bytes = 0
        self.runthread_over = False
        self.rtsp_feed_enabled = False
        self.rtsp_wait_started_ms = 0
        self.rtsp_client_seen_ms = None
        self.rtsp_last_poll_ms = 0
        self.rtsp_status_fallback_logged = False

        try:
            self._configure_media()
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
            self.rtsp_timestamp_origin_ms = self.health_sample_ms
            self.rtsp_wait_started_ms = self.health_sample_ms
            self.rtsp_last_poll_ms = time.ticks_add(
                self.rtsp_wait_started_ms,
                -AP_CLIENT_POLL_MS
            )
            self.server_started = True

            print("已取得H.264编码首帧，字节数:", self.encoded_bytes)
            print("RTSP等待热点客户端，等待期间编码帧将立即释放")

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
                    frame_sent = False
                    send_this_frame = self.rtsp_feed_enabled
                    frame_pts = 0

                    # v1.4-19固件没有在本板运行时验证过
                    # STREAM_TYPE_HEADER/I名称，不能让编码线程依赖它们。
                    # 使用短GOP，让播放器最多等待很短时间便取得下一I帧。
                    for pack_index in range(
                        stream_data.pack_cnt
                    ):
                        packet_size = (
                            stream_data.data_size[pack_index]
                        )

                        if packet_size <= 0:
                            continue

                        frame_bytes += packet_size
                        packet_pts = stream_data.pts[pack_index]
                        if not frame_pts and packet_pts > 0:
                            frame_pts = packet_pts

                    frame_timestamp_ms = 0
                    send_started_ms = 0
                    send_failed = False

                    if send_this_frame:
                        now_ms = time.ticks_ms()

                        if frame_pts > 0:
                            # VENC PTS与正在发送的编码帧严格对应；旧版固定
                            # 1000会破坏播放器的实时播放时钟。
                            frame_timestamp_ms = (
                                frame_pts
                                // VENC_PTS_UNITS_PER_RTSP_MS
                            )
                        else:
                            # 极少数固件/首包没有PTS时仍保持单调毫秒时钟。
                            frame_timestamp_ms = time.ticks_diff(
                                now_ms,
                                self.rtsp_timestamp_origin_ms
                            )

                        if frame_timestamp_ms < 0:
                            frame_timestamp_ms = 0

                        if (
                            frame_timestamp_ms
                            <= self.rtsp_last_timestamp_ms
                        ):
                            frame_timestamp_ms = (
                                self.rtsp_last_timestamp_ms + 1
                            )

                        send_started_ms = now_ms

                    for pack_index in range(
                        stream_data.pack_cnt
                    ):
                        packet_size = (
                            stream_data.data_size[pack_index]
                        )

                        if packet_size <= 0:
                            continue

                        if send_this_frame and not send_failed:
                            packet = bytes(
                                uctypes.bytearray_at(
                                    stream_data.data[pack_index],
                                    packet_size
                                )
                            )
                            try:
                                send_result = (
                                    self.rtsp_server
                                    .rtspserver_sendvideodata(
                                        self.session_name,
                                        packet,
                                        packet_size,
                                        frame_timestamp_ms
                                    )
                                )
                                if send_result not in (None, 0):
                                    raise RuntimeError(
                                        "RTSP返回错误码%d"
                                        % send_result
                                    )
                                frame_sent = True
                            except Exception as error:
                                # Keep draining VENC on a socket failure so
                                # LCD, recognition and UART never stop.
                                send_failed = True
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
                                        "RTSP发送暂时失败，"
                                        "LCD/识别/UART继续:",
                                        error
                                    )

                    if (
                        frame_sent
                        and not send_failed
                    ):
                        self.rtsp_last_timestamp_ms = (
                            frame_timestamp_ms
                        )
                        send_duration_ms = time.ticks_diff(
                            time.ticks_ms(),
                            send_started_ms
                        )
                        self.rtsp_last_send_duration_ms = (
                            send_duration_ms
                        )

                        if (
                            send_duration_ms
                            > self.rtsp_max_send_duration_ms
                        ):
                            self.rtsp_max_send_duration_ms = (
                                send_duration_ms
                            )

                        if send_duration_ms >= RTSP_SLOW_SEND_MS:
                            self.rtsp_slow_send_count += 1

                    if frame_bytes > 0:
                        self.encoded_frames += 1
                        self.encoded_bytes += frame_bytes
                        self.last_stream_ms = time.ticks_ms()
                        self.first_frame_ready = True
                        if frame_sent and not send_failed:
                            self.rtsp_sent_frames += 1
                        else:
                            self.rtsp_discarded_frames += 1

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
            "frames=%d bytes=%d rtsp_sent=%d dropped=%d "
            "get_fail=%d send_fail=%d "
            "send_ms=%d max_send_ms=%d slow=%d "
            "rtsp=%s thread=%s"
            % (
                fps,
                kbps,
                self.encoded_frames,
                self.encoded_bytes,
                self.rtsp_sent_frames,
                self.rtsp_discarded_frames,
                self.get_stream_failures,
                self.rtsp_send_failures,
                self.rtsp_last_send_duration_ms,
                self.rtsp_max_send_duration_ms,
                self.rtsp_slow_send_count,
                (
                    "SEND"
                    if self.rtsp_feed_enabled
                    else (
                        "READY"
                        if self.rtsp_started
                        else "WAIT"
                    )
                ),
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
        if not self.rtsp_started:
            return None

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
        self.rtsp_feed_enabled = False

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
        self.rtsp_client_seen_ms = None

        gc.collect()
        print("识别与图传资源释放完成")

    def stop(self):
        self.stop_stream_thread()
        self._release_resources()


# ============================================================
# LCD识别标记
# ============================================================

def draw_marker(
    pipeline,
    motion,
    uart_sender=None,
    target_line=None,
    force=False
):
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

    now_ms = time.ticks_ms()
    if (
        not force
        and last_marker_update_ms
        and time.ticks_diff(
            now_ms,
            last_marker_update_ms
        )
        < DISPLAY_MARKER_INTERVAL_MS
    ):
        return
    last_marker_update_ms = now_ms

    pipeline.osd_img.clear()
    if target_line is None:
        target_x_q4 = BALL_TARGET_DEFAULT_X_Q4
        display_center_x = target_q4_to_display_x(
            target_x_q4
        )
        line_thickness = 4
        dragging = False
    else:
        target_x_q4 = target_line.target_x_q4
        display_center_x = target_line.display_x()
        line_thickness = 6 if target_line.dragging else 4
        dragging = target_line.dragging
    target_x = target_x_q4 / 16.0

    pipeline.osd_img.draw_line(
        display_center_x,
        0,
        display_center_x,
        DISPLAY_HEIGHT - 1,
        color=CENTER_LINE_COLOR,
        thickness=line_thickness
    )

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
        pipeline.osd_img.draw_string_advanced(
            8,
            8,
            22,
            "X:%.1f T:%.1f ERR:%+.1f%s%s"
            % (
                motion[0],
                target_x,
                target_x - motion[0],
                " PRED" if motion[4] <= 0.0 else "",
                " DRAG" if dragging else ""
            ),
            color=OSD_TEXT_COLOR
        )
    else:
        pipeline.osd_img.draw_string_advanced(
            8,
            8,
            22,
            "BALL LOST T:%.1f%s"
            % (
                target_x,
                " DRAG" if dragging else ""
            ),
            color=OSD_TEXT_COLOR
        )

    if uart_sender is None or uart_sender.uart is None:
        uart_text = "UART:OFF LINK LOST"
    else:
        uart_text = "VTX:%d TTX:%d %s E:%d/%d" % (
            uart_sender.sent,
            uart_sender.target_sent,
            uart_sender.target_ack_text(now_ms),
            uart_sender.failures,
            uart_sender.status_parser.crc_errors
        )
    pipeline.osd_img.draw_string_advanced(
        8,
        38,
        20,
        uart_text,
        color=OSD_TEXT_COLOR
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
    print("钢珠识别与H.264编码管线启动成功")
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
    print("低延迟档位:", RTSP_LOW_LATENCY_PRESET)
    print(
        VIDEO_WIDTH,
        "x",
        VIDEO_HEIGHT,
        "@",
        VIDEO_FPS,
        "FPS，无识别标记"
    )
    print("Sensor/识别帧率:", SENSOR_FPS, "FPS")
    print("码率:", VIDEO_BIT_RATE, "kbps")
    print("GOP:", VIDEO_GOP)
    print("VENC缓冲:", VENC_BUFFER_COUNT)
    print("RTSP时间戳: VENC PTS转毫秒，缺失时单调时钟回退")
    print("LCD标记刷新间隔:", DISPLAY_MARKER_INTERVAL_MS, "ms")
    print(
        "RTSP状态:",
        (
            "已启用，发送当前帧"
            if pipeline.rtsp_feed_enabled
            else "等待热点客户端，旧编码帧不缓存"
        )
    )
    print("UDP坐标端口:", UDP_PORT)
    print(
        "UART坐标:",
        "IO3 TX -> PB7 RX, 115200 8N1, 约30Hz"
    )
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
    print("VLC最低延迟命令（UDP优先）:")
    print(
        'vlc --no-audio --network-caching=50 --live-caching=50 '
        '--clock-jitter=0 --clock-synchro=0 --drop-late-frames '
        '--skip-frames "%s"'
        % manual_url
    )
    print("VLC抗丢包命令（UDP花屏时改TCP）:")
    print(
        'vlc --no-audio --rtsp-tcp --network-caching=100 '
        '--live-caching=100 --drop-late-frames --skip-frames "%s"'
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
    uart_sender = None
    servo = None
    touch = None
    target_line = DraggableTargetLine()
    last_touch_init_ms = time.ticks_add(
        time.ticks_ms(),
        -TOUCH_RETRY_MS
    )
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
        uart_sender = BallUartSender()
        uart_sender.set_target(
            target_line.target_x_q4,
            force=True
        )
        servo = BallServoController()
        log_event(
            "AP已就绪，立即启动LCD、识别、UART与RTSP；"
            "客户端可稍后连接"
        )

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
                pipeline.poll_rtsp_client(ap)
                if touch is None:
                    last_touch_init_ms = time.ticks_ms()
                    try:
                        touch = TOUCH(0)
                        print(
                            "TOUCH(0) ready: drag red line "
                            "within +/-30 LCD pixels"
                        )
                    except Exception as error:
                        touch = None
                        print("触摸目标线关闭:", error)
                uart_sender.request_target_resend()
                display_marker_enabled = True
                display_marker_error_reported = False
                display_marker_last_error_ms = 0

                tracker = SingleBallTracker(
                    KMODEL_PATH
                )
                pipeline.poll_rtsp_client(ap)

                print_startup_summary(
                    pipeline,
                    manual_url,
                    restart_count
                )

                log_event(
                    "识别和H.264编码首帧自检通过 "
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
                    points = ()
                    if (
                        touch is None
                        and time.ticks_diff(
                            time.ticks_ms(),
                            last_touch_init_ms
                        ) >= TOUCH_RETRY_MS
                    ):
                        last_touch_init_ms = time.ticks_ms()
                        try:
                            touch = TOUCH(0)
                            print("TOUCH(0)重新初始化成功")
                        except Exception:
                            touch = None
                    if touch is not None:
                        try:
                            points = touch.read(1)
                        except Exception as error:
                            print("触摸读取失败，1秒后重试:", error)
                            touch = None
                            last_touch_init_ms = time.ticks_ms()
                    target_changed, target_committed = (
                        target_line.update(points)
                    )
                    if target_changed:
                        uart_sender.set_target(
                            target_line.target_x_q4
                        )
                    if target_committed:
                        uart_sender.set_target(
                            target_line.target_x_q4,
                            force=True
                        )
                    uart_sender.poll_status()
                    uart_sender.service_target(
                        force=target_committed
                    )
                    uart_sender.send(motion)
                    draw_marker(
                        pipeline,
                        motion,
                        uart_sender,
                        target_line,
                        force=target_changed or target_committed
                    )
                    udp.send(motion)
                    servo_angles = servo.update(motion)
                    pipeline.poll_rtsp_client(ap)

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
                        print(
                            "[TARGET] x=%.1f %s "
                            "target_tx=%d status_rx=%d "
                            "crc=%d format=%d"
                            % (
                                target_line.target_x_q4 / 16.0,
                                uart_sender.target_ack_text(now),
                                uart_sender.target_sent,
                                uart_sender.status_parser.valid_frames,
                                uart_sender.status_parser.crc_errors,
                                uart_sender.status_parser.format_errors
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

        if uart_sender is not None:
            uart_sender.close()

        tracker = None
        pipeline = None
        servo = None
        udp = None
        uart_sender = None
        ap = None
        gc.collect()

        log_event("程序结束")
        print("程序结束")


if __name__ == "__main__":
    main()
