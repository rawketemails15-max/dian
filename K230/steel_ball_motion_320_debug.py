from libs.PipeLine import PipeLine
from media.sensor import Sensor
from media.display import Display
import nncase_runtime as nn
import ulab.numpy as np
import time
import gc


# 2: K230 ST7701 LCD
# 3: CanMV IDE image window
DISPLAY_MODE = 2

KMODEL_PATH = "/sdcard/examples/kmodel/steel_ball_single_320_lowlatency.kmodel"
AI_FRAME_SIZE = [320, 320]
# Use a stricter score before acquiring a target. Once locked, permit a lower
# score so motion blur does not cause the marker to lag or disappear.
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
GC_INTERVAL = 60


def init_display(mode):
    if mode == 2:
        width = 800
        height = 480
        Display.init(
            Display.ST7701,
            width=width,
            height=height,
            to_ide=False
        )
        display_name = "st7701"
    elif mode == 3:
        width = 800
        height = 480
        Display.init(
            Display.VIRT,
            width=width,
            height=height,
            fps=30,
            to_ide=True
        )
        display_name = "virtual"
    else:
        raise ValueError("DISPLAY_MODE must be 2 or 3")

    return width, height, display_name


class SingleBallTracker:
    def __init__(self, kmodel_path):
        self.kpu = nn.kpu()
        self.kpu.load_kmodel(kmodel_path)
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_time_us = 0
        self.has_last_position = False
        self.lost_frames = 0

    @staticmethod
    def _is_pipe_inner_wall(pixels, x, y):
        x = int(x)
        y = int(y)
        if x < 0 or y < 0 or x >= AI_FRAME_SIZE[0] or y >= AI_FRAME_SIZE[1]:
            return False

        red = int(pixels[0, y, x])
        green = int(pixels[1, y, x])
        blue = int(pixels[2, y, x])

        # Current trough is yellow-brown: R is slightly above G, and both are
        # clearly above B. Using channel differences makes this tolerant of
        # exposure changes while rejecting the nearly neutral white board and
        # silver steel ball.
        return (
            red >= 35
            and green >= 30
            and red - blue >= 10
            and green - blue >= 5
            and red - green >= -5
        )

    def _has_pipe_context(self, pixels, center_x, center_y, width, height):
        # A real ball is silver/grey but the pixels immediately around it are
        # the green inner wall. Pipe scratches and paper text fail this test.
        radius_x = int(width * 0.65) + 3
        radius_y = int(height * 0.65) + 3
        sample_points = (
            (center_x - radius_x, center_y),
            (center_x + radius_x, center_y),
            (center_x, center_y - radius_y),
            (center_x, center_y + radius_y),
            (center_x - radius_x, center_y - radius_y),
            (center_x + radius_x, center_y - radius_y),
            (center_x - radius_x, center_y + radius_y),
            (center_x + radius_x, center_y + radius_y),
        )

        pipe_hits = 0
        for x, y in sample_points:
            if self._is_pipe_inner_wall(pixels, x, y):
                pipe_hits += 1

        # The trough is narrow, so two valid surrounding points are enough
        # even when the ball is close to one of its white edges.
        return pipe_hits >= 2

    def _select_candidate(self, output_data, input_np):
        scores = output_data[4, :]
        candidate_indices = np.argsort(scores, axis=0)[::-1]
        pixels = input_np[0] if len(input_np.shape) == 4 else input_np
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
                    delta_x * delta_x + delta_y * delta_y
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
                (result.shape[0] * result.shape[1], result.shape[2])
            )

        if result.shape[0] <= result.shape[1]:
            output_data = result
        else:
            output_data = result.transpose()

        candidate = self._select_candidate(output_data, input_np)
        if candidate is None:
            self.lost_frames += 1
            if self.lost_frames >= LOST_FRAMES_TO_UNLOCK:
                self.has_last_position = False
            return None

        center_x, center_y, best_score = candidate
        now_us = time.ticks_us()

        velocity_x = 0.0
        velocity_y = 0.0
        if self.has_last_position:
            delta_us = time.ticks_diff(now_us, self.last_time_us)
            if delta_us > 0:
                scale = 1000000.0 / float(delta_us)
                velocity_x = (center_x - self.last_x) * scale
                velocity_y = (center_y - self.last_y) * scale

        self.last_x = center_x
        self.last_y = center_y
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


def draw_marker(pipeline, motion, display_size):
    pipeline.osd_img.clear()

    if motion is not None:
        center_x = int(
            motion[0] * display_size[0] // AI_FRAME_SIZE[0]
        )
        center_y = int(
            motion[1] * display_size[1] // AI_FRAME_SIZE[1]
        )

        left = center_x - MARKER_SIZE
        top = center_y - MARKER_SIZE
        if left < 0:
            left = 0
        if top < 0:
            top = 0

        marker_width = MARKER_SIZE * 2
        marker_height = MARKER_SIZE * 2
        if left + marker_width >= display_size[0]:
            marker_width = display_size[0] - left - 1
        if top + marker_height >= display_size[1]:
            marker_height = display_size[1] - top - 1

        pipeline.osd_img.draw_rectangle(
            left,
            top,
            marker_width,
            marker_height,
            color=MARKER_COLOR,
            thickness=2
        )

    pipeline.show_image()


def main():
    pipeline = None
    tracker = None
    frame_count = 0

    try:
        display_width, display_height, display_name = init_display(
            DISPLAY_MODE
        )
        display_size = [display_width, display_height]

        pipeline = PipeLine(
            rgb888p_size=AI_FRAME_SIZE,
            display_size=display_size,
            display_mode=display_name
        )
        pipeline.create(Sensor(width=1280, height=960))

        tracker = SingleBallTracker(KMODEL_PATH)

        while True:
            input_np = pipeline.get_frame()
            motion = tracker.run(input_np)
            draw_marker(pipeline, motion, display_size)

            frame_count += 1
            if frame_count % GC_INTERVAL == 0:
                gc.collect()

    except KeyboardInterrupt:
        pass
    finally:
        if tracker is not None:
            tracker.deinit()
        if pipeline is not None:
            pipeline.destroy()
        Display.deinit()
        gc.collect()


if __name__ == "__main__":
    main()
