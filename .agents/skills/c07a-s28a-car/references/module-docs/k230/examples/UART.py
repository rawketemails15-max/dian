from machine import UART, Pin, FPIOA
import time

LED_PIN_NUM = 52
UART_ID = UART.UART1    # 可根据实际硬件调整
BAUDRATE = 115200
UART1_TX_PIN = 3        # 可根据实际硬件调整
UART1_RX_PIN = 4
fpioa = FPIOA()
fpioa.set_function(LED_PIN_NUM, getattr(FPIOA, f'GPIO{LED_PIN_NUM}'))
fpioa.set_function(UART1_TX_PIN, FPIOA.UART1_TXD, oe=1)
fpioa.set_function(UART1_RX_PIN, FPIOA.UART1_RXD, ie=1)

led_blue = Pin(LED_PIN_NUM, Pin.OUT)
uart = UART(UART_ID, BAUDRATE)
try:
    led_blue.value(0)  # 0为熄灭, ​1为点亮
    print("UART LED Control Ready!")
    print("---------------------------------")
    print(f"UART Port: {UART_ID}, Baudrate: {BAUDRATE}")
    print("Send '1' to turn the LED ON.")
    print("Send '0' to turn the LED OFF.")
    print("---------------------------------")

    while True:
        data = uart.read(1)
        if data:
            if data == b'1':
                led_blue.value(1) # 点亮LED
                print("Received '1', LED is ON.") # 打印反馈信息

            elif data == b'0':
                led_blue.value(0) # 熄灭LED
                print("Received '0', LED is OFF.") # 打印反馈信息

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:

    print("Deinitializing UART and turning off LED...")
    led_blue.value(0)
    uart.deinit()
    print("Cleanup complete.")

