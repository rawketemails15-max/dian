import socket
import time
import network

def udpclient():
    WIFI_SSID = "your_ssid_name"
    WIFI_PASSWORD = "your_ssid_password"
    SERVER_IP = "your_IP"
    SERVER_PORT = 8080


    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f"正在连接到WiFi网络: {WIFI_SSID}...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        start_time = time.time()
        while not wlan.isconnected():
            if time.time() - start_time > 15:
                print("WiFi连接超时！请检查SSID、密码或信号强度。")
                return
            time.sleep(1)

    if wlan.isconnected():
        print("WiFi连接成功！")
        print("网络信息:", wlan.ifconfig())
    else:
        print("WiFi连接失败。")
        return

    try:
        ai = socket.getaddrinfo(SERVER_IP, SERVER_PORT)
        addr = ai[0][-1]
        print("目标服务器地址:", addr)
    except OSError as e:
        print(f"获取地址信息失败: {e}")
        return

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for i in range(10):
        message = "hiwonder K230 UDP {0} \r\n".format(i)
        print(f"正在发送 -> {message.strip()}")
        s.sendto(message.encode('utf-8'), addr)
        time.sleep(0.2)

    time.sleep(1)
    s.close()
    print("客户端运行结束")

udpclient()
