"""
K230 steel-ball vision V2: YOLOv8 + draggable target + MSPM0 link.

This is the canonical K230 entry point for the vehicle-mounted ball system.
The proven three-channel media, Wi-Fi, UDP, RTSP and binary UART transport
remain in the fallback module; this file supplies the V2 detector, coordinate
mapping, motion estimation, touch target and per-inference-frame OSD.
"""

import gc
import os
import sys
import time

from libs.YOLO import YOLOv8
from machine import TOUCH

import steel_ball_motion_wifi_320_full_v3 as platform


# ---------------------------------------------------------------------------
# V2 detector and coordinate spaces
# ---------------------------------------------------------------------------

SCRIPT_VERSION = "li-v2-full-control-1"
MODEL_PATH = "/sdcard/best_v2.kmodel"
DETECTION_SIZE = [640, 360]
MODEL_INPUT_SIZE = [320, 320]
DISPLAY_SIZE = [800, 480]
CONTROL_SIZE = [320, 320]

LABELS = ["ball"]
CONF_THRESHOLD = 0.30
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 20
MAX_BALL_SPEED_PX_S = 900.0
PREDICT_SINGLE_MISSED_FRAME = True

TARGET_DEFAULT_X = 171
TARGET_MIN_X_Q4 = 0
TARGET_MAX_X_Q4 = 319 * 16
TOUCH_LINE_HIT_HALF_WIDTH = 30
TOUCH_RELEASE_EMPTY_FRAMES = 2
TOUCH_TARGET_DEADBAND_Q4 = 8
TOUCH_RETRY_MS = 1000

GC_INTERVAL_FRAMES = 300
LATENCY_PRINT_INTERVAL_MS = 2000
OSD_RETRY_MS = 1000


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def display_x_to_control_x(display_x, display_width=DISPLAY_SIZE[0]):
    """Map a YOLO/LCD X coordinate to the independent 320-pixel control axis."""
    if display_width <= 0:
        return 0.0
    display_x = clamp(float(display_x), 0.0, float(display_width - 1))
    return clamp(
        display_x * CONTROL_SIZE[0] / float(display_width),
        0.0,
        float(CONTROL_SIZE[0] - 1),
    )


def display_y_to_control_y(display_y, display_height=DISPLAY_SIZE[1]):
    if display_height <= 0:
        return 0.0
    display_y = clamp(float(display_y), 0.0, float(display_height - 1))
    return clamp(
        display_y * CONTROL_SIZE[1] / float(display_height),
        0.0,
        float(CONTROL_SIZE[1] - 1),
    )


def display_x_to_target_q4(display_x, display_width=DISPLAY_SIZE[0]):
    """Use the agreed round(touchX * 320 * 16 / 800) mapping and clamp."""
    if display_width <= 0:
        return TARGET_MIN_X_Q4
    display_x = clamp(int(display_x), 0, display_width - 1)
    target_q4 = (
        display_x * CONTROL_SIZE[0] * 16 + display_width // 2
    ) // display_width
    return int(clamp(target_q4, TARGET_MIN_X_Q4, TARGET_MAX_X_Q4))


def target_q4_to_display_x(target_q4, display_width=DISPLAY_SIZE[0]):
    target_q4 = int(clamp(
        int(target_q4),
        TARGET_MIN_X_Q4,
        TARGET_MAX_X_Q4,
    ))
    display_x = (
        target_q4 * display_width + CONTROL_SIZE[0] * 8
    ) // (CONTROL_SIZE[0] * 16)
    return int(clamp(display_x, 0, display_width - 1))


def get_best_ball(result):
    """Return the highest-confidence class-0 detection in display coordinates."""
    if result is None:
        return None

    try:
        if len(result) < 3:
            return None
        boxes = result[0]
        class_ids = result[1]
        scores = result[2]
        count = min(len(boxes), len(class_ids), len(scores))
    except Exception:
        return None

    best_ball = None
    best_score = -1.0

    for index in range(count):
        try:
            box = boxes[index]
            if len(box) < 4:
                continue
            box_x = int(box[0])
            box_y = int(box[1])
            box_w = int(box[2])
            box_h = int(box[3])
            class_id = int(class_ids[index])
            score = float(scores[index])
        except Exception:
            continue

        if (
            class_id != 0
            or score < CONF_THRESHOLD
            or box_w <= 0
            or box_h <= 0
            or score <= best_score
        ):
            continue

        best_score = score
        best_ball = (
            box_x + box_w // 2,
            box_y + box_h // 2,
            box_x,
            box_y,
            box_w,
            box_h,
            score,
        )

    return best_ball


class MotionEstimator:
    """Convert real detections to control coordinates and real-time velocity."""

    def __init__(self, display_width=800, display_height=480):
        self.display_width = display_width
        self.display_height = display_height
        self.locked = False
        self.missed_frames = 0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_velocity_x = 0.0
        self.last_velocity_y = 0.0
        self.last_real_us = 0

    def reset(self):
        self.locked = False
        self.missed_frames = 0
        self.last_velocity_x = 0.0
        self.last_velocity_y = 0.0
        self.last_real_us = 0

    def update(self, best_ball, now_us):
        if best_ball is not None:
            x = display_x_to_control_x(
                best_ball[0],
                self.display_width,
            )
            y = display_y_to_control_y(
                best_ball[1],
                self.display_height,
            )
            confidence = float(best_ball[6])
            velocity_x = 0.0
            velocity_y = 0.0

            # A reacquired detection is a new track. Never derive velocity
            # across a missing frame because that creates false high speed.
            if self.locked and self.missed_frames == 0:
                elapsed_us = time.ticks_diff(now_us, self.last_real_us)
                if elapsed_us > 0:
                    scale = 1000000.0 / float(elapsed_us)
                    velocity_x = clamp(
                        (x - self.last_x) * scale,
                        -MAX_BALL_SPEED_PX_S,
                        MAX_BALL_SPEED_PX_S,
                    )
                    velocity_y = clamp(
                        (y - self.last_y) * scale,
                        -MAX_BALL_SPEED_PX_S,
                        MAX_BALL_SPEED_PX_S,
                    )

            self.locked = True
            self.missed_frames = 0
            self.last_x = x
            self.last_y = y
            self.last_velocity_x = velocity_x
            self.last_velocity_y = velocity_y
            self.last_real_us = now_us
            return (
                x,
                y,
                velocity_x,
                velocity_y,
                confidence,
                now_us,
            )

        if (
            self.locked
            and self.missed_frames == 0
            and PREDICT_SINGLE_MISSED_FRAME
        ):
            self.missed_frames = 1
            elapsed_us = max(
                0,
                time.ticks_diff(now_us, self.last_real_us),
            )
            elapsed_s = elapsed_us / 1000000.0
            predicted_x = clamp(
                self.last_x + self.last_velocity_x * elapsed_s,
                0.0,
                float(CONTROL_SIZE[0] - 1),
            )
            predicted_y = clamp(
                self.last_y + self.last_velocity_y * elapsed_s,
                0.0,
                float(CONTROL_SIZE[1] - 1),
            )
            return (
                predicted_x,
                predicted_y,
                self.last_velocity_x,
                self.last_velocity_y,
                0.0,
                now_us,
            )

        # The second missing frame is explicitly invalid. Reacquisition will
        # start with zero velocity.
        self.locked = False
        self.missed_frames += 1
        self.last_velocity_x = 0.0
        self.last_velocity_y = 0.0
        return None


class DraggableTargetLine:
    STATE_IDLE = 0
    STATE_DRAGGING = 1
    STATE_IGNORED = 2

    def __init__(
        self,
        target_x_q4=TARGET_DEFAULT_X * 16,
        display_width=DISPLAY_SIZE[0],
    ):
        self.display_width = display_width
        self.target_x_q4 = int(clamp(
            int(target_x_q4),
            TARGET_MIN_X_Q4,
            TARGET_MAX_X_Q4,
        ))
        self.state = self.STATE_IDLE
        self.dragging = False
        self.empty_frames = 0

    def display_x(self):
        return target_q4_to_display_x(
            self.target_x_q4,
            self.display_width,
        )

    def lock_to(self, target_x_q4):
        self.target_x_q4 = int(clamp(
            int(target_x_q4),
            TARGET_MIN_X_Q4,
            TARGET_MAX_X_Q4,
        ))
        self.state = self.STATE_IDLE
        self.dragging = False
        self.empty_frames = 0

    def update(self, points):
        changed = False
        committed = False

        if points:
            touch_x = int(clamp(
                int(points[0].x),
                0,
                self.display_width - 1,
            ))
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
                    touch_x,
                    self.display_width,
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


class LatencyStats:
    NAMES = ("capture", "infer", "service", "osd", "loop")

    def __init__(self):
        self.window_started_ms = time.ticks_ms()
        self.count = 0
        self.total = {}
        self.maximum = {}
        self._clear_values()

    def _clear_values(self):
        for name in self.NAMES:
            self.total[name] = 0
            self.maximum[name] = 0

    def add(self, name, elapsed_us):
        elapsed_us = max(0, int(elapsed_us))
        self.total[name] += elapsed_us
        if elapsed_us > self.maximum[name]:
            self.maximum[name] = elapsed_us

    def finish_loop(self, now_ms):
        self.count += 1
        elapsed_ms = time.ticks_diff(now_ms, self.window_started_ms)
        if elapsed_ms < LATENCY_PRINT_INTERVAL_MS:
            return

        count = max(1, self.count)
        fps = self.count * 1000.0 / max(1, elapsed_ms)
        values = []
        for name in self.NAMES:
            values.append(
                "%s=%.1f/%0.1fms"
                % (
                    name,
                    self.total[name] / count / 1000.0,
                    self.maximum[name] / 1000.0,
                )
            )
        print("[LATENCY] fps=%.1f %s" % (fps, " ".join(values)))

        self.window_started_ms = now_ms
        self.count = 0
        self._clear_values()


class OsdRenderer:
    """Draw once per inference frame; retry one second after an OSD failure."""

    def __init__(self):
        self.enabled = True
        self.last_error_ms = 0

    def draw(
        self,
        pipeline,
        yolo,
        result,
        motion,
        target_line,
        uart_sender,
    ):
        now_ms = time.ticks_ms()
        if (
            not self.enabled
            and time.ticks_diff(now_ms, self.last_error_ms) < OSD_RETRY_MS
        ):
            return False

        self.enabled = True
        try:
            osd = pipeline.osd_img
            osd.clear()
            yolo.draw_result(result, osd)

            line_x = target_line.display_x()
            line_width = 6 if target_line.dragging else 4
            osd.draw_line(
                line_x,
                0,
                line_x,
                DISPLAY_SIZE[1] - 1,
                color=(255, 255, 0, 0),
                thickness=line_width,
            )

            if motion is None:
                ball_text = "BALL LOST"
                error_text = "E=--"
                mode_text = "LOST"
            else:
                ball_display_x = int(
                    motion[0] * DISPLAY_SIZE[0] / CONTROL_SIZE[0]
                )
                ball_display_y = int(
                    motion[1] * DISPLAY_SIZE[1] / CONTROL_SIZE[1]
                )
                predicted = motion[4] <= 0.0
                marker_color = (
                    (255, 255, 255, 0)
                    if predicted
                    else (255, 0, 255, 0)
                )
                osd.draw_cross(
                    ball_display_x,
                    ball_display_y,
                    color=marker_color,
                    size=12,
                    thickness=2,
                )
                error = target_line.target_x_q4 / 16.0 - motion[0]
                ball_text = "BALL X=%.1f V=%.1f" % (
                    motion[0],
                    motion[2],
                )
                error_text = "T=%.1f E=%+.1f" % (
                    target_line.target_x_q4 / 16.0,
                    error,
                )
                mode_text = "PRED" if predicted else "REAL"

            if target_line.dragging:
                mode_text += " DRAG"

            osd.draw_string_advanced(
                10,
                8,
                22,
                ball_text,
                color=(255, 255, 255, 255),
            )
            osd.draw_string_advanced(
                10,
                36,
                22,
                error_text,
                color=(255, 255, 255, 0),
            )
            osd.draw_string_advanced(
                10,
                64,
                20,
                "%s %s TX=%d RX=%d"
                % (
                    mode_text,
                    uart_sender.target_ack_text(now_ms),
                    uart_sender.target_sent,
                    uart_sender.status_parser.valid_frames,
                ),
                color=(255, 255, 255, 255),
            )
            pipeline.show_image()
            return True
        except Exception as error:
            self.enabled = False
            self.last_error_ms = now_ms
            print("OSD刷新失败，1秒后重试:", error)
            return False


def create_touch():
    try:
        touch = TOUCH(0)
        print("TOUCH(0) ready: drag within +/-30 LCD pixels of red line")
        return touch
    except Exception as error:
        print("触摸目标线暂不可用，1秒后重试:", error)
        return None


def create_yolo():
    yolo = YOLOv8(
        task_type="detect",
        mode="video",
        kmodel_path=MODEL_PATH,
        labels=LABELS,
        rgb888p_size=DETECTION_SIZE,
        model_input_size=MODEL_INPUT_SIZE,
        display_size=DISPLAY_SIZE,
        conf_thresh=CONF_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=MAX_BOXES_NUM,
        debug_mode=0,
    )
    yolo.config_preprocess()
    return yolo


def print_startup_summary(pipeline, ap_ip, restart_count):
    print("=" * 60)
    print("K230 V2完整视觉:", SCRIPT_VERSION)
    print("模型:", MODEL_PATH)
    print("检测通道: 640x360 RGB888P -> 320x320 model")
    print("LCD: 800x480, 每个推理帧刷新OSD")
    if pipeline.network_enabled:
        print(
            "RTSP: rtsp://%s:%d/%s (%dx%d @ %d FPS)"
            % (
                ap_ip,
                platform.RTSP_PORT,
                platform.RTSP_SESSION,
                pipeline.video_width,
                pipeline.video_height,
                platform.VIDEO_FPS,
            )
        )
    else:
        print("RTSP: OFFLINE (LCD/AI/UART remain active)")
    print("UART1: IO3 TX / IO4 RX / 115200 8N1")
    print("目标默认X=%d, 媒体重建次数=%d" % (
        TARGET_DEFAULT_X,
        restart_count,
    ))
    print("=" * 60)


def run_media_loop(
    pipeline,
    yolo,
    ap,
    touch_state,
    target_line,
    uart_sender,
    udp_sender,
):
    estimator = MotionEstimator(DISPLAY_SIZE[0], DISPLAY_SIZE[1])
    renderer = OsdRenderer()
    latency = LatencyStats()
    frame_count = 0

    while True:
        loop_started_us = time.ticks_us()
        try:
            os.exitpoint()
        except KeyboardInterrupt:
            raise
        except Exception:
            pass

        stage_started_us = time.ticks_us()
        input_np = pipeline.get_frame()
        latency.add(
            "capture",
            time.ticks_diff(time.ticks_us(), stage_started_us),
        )

        stage_started_us = time.ticks_us()
        result = yolo.run(input_np)
        best_ball = get_best_ball(result)
        motion = estimator.update(best_ball, time.ticks_us())
        latency.add(
            "infer",
            time.ticks_diff(time.ticks_us(), stage_started_us),
        )

        # Touch target -> status RX -> target TX -> vision TX -> UDP.
        stage_started_us = time.ticks_us()
        now_ms = time.ticks_ms()
        uart_sender.poll_status()
        target_locked = uart_sender.target_locked()
        target_selection_allowed = (
            uart_sender.target_selection_allowed()
        )
        if (
            target_locked
            and uart_sender.status_parser.target_x_q4 is not None
        ):
            target_line.lock_to(
                uart_sender.status_parser.target_x_q4
            )
        if (
            target_selection_allowed
            and not target_locked
            and
            touch_state[0] is None
            and time.ticks_diff(now_ms, touch_state[1]) >= TOUCH_RETRY_MS
        ):
            touch_state[0] = create_touch()
            touch_state[1] = now_ms

        points = ()
        if (
            target_selection_allowed
            and not target_locked
            and touch_state[0] is not None
        ):
            try:
                points = touch_state[0].read(1)
            except Exception as error:
                print("触摸读取失败，1秒后重试:", error)
                touch_state[0] = None
                touch_state[1] = now_ms

        target_changed, target_committed = (
            target_line.update(points)
            if target_selection_allowed and not target_locked
            else (False, False)
        )
        if target_changed:
            uart_sender.set_target(target_line.target_x_q4)
        if target_committed:
            uart_sender.set_target(
                target_line.target_x_q4,
                force=True,
            )

        uart_sender.service_target(force=target_committed)
        uart_sender.send(motion)
        udp_sender.send(motion)
        latency.add(
            "service",
            time.ticks_diff(time.ticks_us(), stage_started_us),
        )

        # OSD is intentionally updated once for every completed inference.
        stage_started_us = time.ticks_us()
        renderer.draw(
            pipeline,
            yolo,
            result,
            motion,
            target_line,
            uart_sender,
        )
        latency.add(
            "osd",
            time.ticks_diff(time.ticks_us(), stage_started_us),
        )

        # RTSP uses its independent raw 480x360 H.264 channel.
        pipeline.poll_rtsp_client(ap)

        frame_count += 1
        if frame_count % GC_INTERVAL_FRAMES == 0:
            gc.collect()

        health_problem = pipeline.health_error()
        if health_problem:
            raise RuntimeError("媒体链路异常: %s" % health_problem)

        latency.add(
            "loop",
            time.ticks_diff(time.ticks_us(), loop_started_us),
        )
        latency.finish_loop(time.ticks_ms())


def main():
    pipeline = None
    yolo = None
    uart_sender = None
    udp_sender = None
    ap = None

    # This is the only platform global changed at runtime: channel 2 must
    # match the V2 detector. The fallback file itself remains untouched.
    platform.AI_FRAME_SIZE = DETECTION_SIZE
    platform.VIDEO_WIDTH = 1280
    platform.VIDEO_HEIGHT = 720
    platform.VIDEO_BIT_RATE = 2500


    target_line = DraggableTargetLine(TARGET_DEFAULT_X * 16)
    touch_state = [None, time.ticks_ms() - TOUCH_RETRY_MS]
    restart_count = 0
    network_allowed = False

    try:
        print("=" * 60)
        print("K230 steel-ball V2 full control starting")
        print("=" * 60)
        os.exitpoint(os.EXITPOINT_ENABLE)

        model_info = os.stat(MODEL_PATH)
        print("V2模型:", MODEL_PATH, "bytes=", model_info[6])

        time.sleep_ms(platform.BOOT_SETTLE_MS)
        try:
            ap, ap_ip = platform.start_ap()
        except BaseException as error:
            ap = None
            ap_ip = "0.0.0.0"
            platform.show_exception(error, "Wi-Fi旁路")
            platform.log_event(
                "Wi-Fi/RTSP启动失败，本次上电仅运行LCD/AI/UART"
            )
        network_allowed = ap is not None
        udp_sender = platform.UdpBallSender(ap_ip)
        uart_sender = platform.BallUartSender()
        uart_sender.set_target(target_line.target_x_q4, force=True)
        touch_state[0] = create_touch()
        touch_state[1] = time.ticks_ms()

        while True:
            try:
                pipeline = platform.CombinedMediaPipeline(
                    enable_network=network_allowed
                )
                pipeline.start()
                if not pipeline.network_enabled:
                    network_allowed = False
                yolo = create_yolo()
                uart_sender.request_target_resend()
                pipeline.poll_rtsp_client(ap)
                print_startup_summary(pipeline, ap_ip, restart_count)
                platform.log_event(
                    "li.py V2视觉与媒体链路就绪 restart=%d"
                    % restart_count
                )
                run_media_loop(
                    pipeline,
                    yolo,
                    ap,
                    touch_state,
                    target_line,
                    uart_sender,
                    udp_sender,
                )
            except KeyboardInterrupt:
                raise
            except BaseException as error:
                platform.show_exception(error, "li.py媒体循环")
                restart_count += 1
            finally:
                if (
                    pipeline is not None
                    and not pipeline.network_enabled
                ):
                    # An optional-network failure is sticky for this boot;
                    # later camera/KPU recovery must not retry Wi-Fi/VENC.
                    network_allowed = False
                if pipeline is not None:
                    try:
                        pipeline.stop_stream_thread()
                    except Exception:
                        pass
                if yolo is not None:
                    try:
                        yolo.deinit()
                    except Exception:
                        pass
                yolo = None
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
                pipeline = None
                gc.collect()

            print("%dms后重建媒体链路，保留目标X=%.1f" % (
                platform.RESTART_DELAY_MS,
                target_line.target_x_q4 / 16.0,
            ))
            platform.delay_with_exitpoint(platform.RESTART_DELAY_MS)

    except KeyboardInterrupt:
        print("用户停止li.py")
    except BaseException as error:
        try:
            platform.show_exception(error, "li.py主程序")
        except Exception:
            try:
                sys.print_exception(error)
            except Exception:
                print(error)
    finally:
        if pipeline is not None:
            try:
                pipeline.stop_stream_thread()
            except Exception:
                pass
        if yolo is not None:
            try:
                yolo.deinit()
            except Exception:
                pass
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        if uart_sender is not None:
            uart_sender.close()
        if udp_sender is not None:
            udp_sender.close()
        gc.collect()
        time.sleep_ms(100)
        print("li.py已退出")


if __name__ == "__main__":
    main()
