"""
K230 touch-screen UART feasibility test.

Target runtime:
  Hiwonder K230, CanMV v1.4-19-ga7de1c8
  800x480 ST7701 LCD and TOUCH(0)

Each new touch that starts inside button A, B, or C sends exactly one ASCII
byte on UART1.  Holding a finger down does not repeat the byte.
"""

import os
import time

import image
from machine import FPIOA, TOUCH, UART
from media.display import Display
from media.media import MediaManager


DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
UART_TX_IO = 3
UART_RX_IO = 4
BUTTONS = (
    ("A", 50, 140, 200, 220, (40, 110, 230)),
    ("B", 300, 140, 200, 220, (30, 180, 100)),
    ("C", 550, 140, 200, 220, (235, 145, 35)),
)


def button_at(x, y):
    for label, bx, by, bw, bh, _color in BUTTONS:
        if bx <= x < bx + bw and by <= y < by + bh:
            return label
    return None


def draw_ui(canvas, last_sent):
    canvas.draw_rectangle(
        0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT,
        color=(18, 24, 36), fill=True)
    canvas.draw_string_advanced(
        210, 28, 42, "UART Touch Test", color=(235, 240, 250))

    status = "Ready" if last_sent is None else "Sent: " + last_sent
    canvas.draw_string_advanced(
        330, 88, 30, status, color=(120, 215, 255))

    for label, x, y, width, height, base_color in BUTTONS:
        if label == last_sent:
            fill_color = (245, 245, 245)
            text_color = base_color
            border_color = (255, 220, 90)
        else:
            fill_color = base_color
            text_color = (255, 255, 255)
            border_color = (135, 155, 185)

        canvas.draw_rectangle(
            x, y, width, height, color=border_color, thickness=4, fill=False)
        canvas.draw_rectangle(
            x + 8, y + 8, width - 16, height - 16,
            color=fill_color, fill=True)
        canvas.draw_string_advanced(
            x + 61, y + 55, 96, label, color=text_color)

    canvas.draw_string_advanced(
        215, 410, 24, "One byte per new touch: A / B / C",
        color=(165, 180, 205))


def main():
    uart = None
    display_initialized = False
    media_initialized = False

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        fpioa = FPIOA()
        fpioa.set_function(
            UART_TX_IO, FPIOA.UART1_TXD,
            ie=0, oe=1, pu=0, pd=0)
        fpioa.set_function(
            UART_RX_IO, FPIOA.UART1_RXD,
            ie=1, oe=0, pu=1, pd=0, st=1)
        fpioa.help(UART_TX_IO)
        fpioa.help(UART_RX_IO)

        uart = UART(
            UART.UART1,
            baudrate=115200,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
            timeout=2)

        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            to_ide=False)
        display_initialized = True

        MediaManager.init()
        media_initialized = True

        touch = TOUCH(0)
        canvas = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB565)
        last_sent = None
        was_touching = False

        draw_ui(canvas, last_sent)
        Display.show_image(canvas)

        while True:
            os.exitpoint()
            points = touch.read(1)
            touching = points != ()

            if touching and not was_touching:
                label = button_at(points[0].x, points[0].y)
                if label is not None:
                    uart.write(label.encode())
                    print("UART TX:", label)
                    last_sent = label
                    draw_ui(canvas, last_sent)
                    Display.show_image(canvas)

            was_touching = touching
            time.sleep_ms(20)

    except KeyboardInterrupt:
        print("Touch UART test stopped")
    except BaseException as error:
        print("Touch UART test error:", error)
        raise
    finally:
        if uart is not None:
            uart.deinit()
        if display_initialized:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_initialized:
            MediaManager.deinit()


main()
