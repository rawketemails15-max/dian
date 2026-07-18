#配置 tcp/udp socket调试工具
import socket
import network
import time

CONTENT = b"""
Hello #%d from k230 canmv MicroPython!
"""

def server():
    WIFI_SSID = "your_ssid_name"
    WIFI_PASSWORD = "your_ssid_password"

    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
        print('正在连接到WiFi网络...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            print('.')
            time.sleep_ms(500)

    ip = wlan.ifconfig()[0]
    print('网络已连接，IP配置:', wlan.ifconfig())

    counter = 1
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ai = socket.getaddrinfo(ip, 8080)
    addr = ai[0][-1]
    s.bind(addr)
    s.listen(5)
    print("TCP服务器已启动于 %s 端口:%d\n" % (ip, 8080))

    while counter <= 10:
        client_sock = None
        try:
            res = s.accept()
            client_sock = res[0]
            client_addr = res[1]
            print("客户端地址:", client_addr)

            client_sock.setblocking(False)
            client_stream = client_sock
            client_stream.write(CONTENT % counter)

            while True:
                h = None
                try:
                    h = client_stream.read()
                except OSError as e:
                    if e.args[0] != 11:
                        print("读取数据时出错:", e)
                        break

                if h and h != b"":
                    print("收到消息:", h)
                    client_stream.write("recv :%s" % h)
                    if b"end" in h:
                        print("客户端请求关闭连接。")
                        break

                time.sleep_ms(100)

            counter += 1

        except OSError as e:
            if e.args[0] == 11:
                time.sleep_ms(200)
                continue
            else:
                print("服务器出现严重错误:", e)
                break
        finally:
            if client_sock:
                client_sock.close()

    print("服务器已达到最大连接次数，即将关闭!")
    s.close()

server()



