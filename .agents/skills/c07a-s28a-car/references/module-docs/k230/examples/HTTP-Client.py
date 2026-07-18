import network
import socket
import time

def main(use_stream=True):
    WIFI_SSID = "your_ssid_name"
    WIFI_PASSWORD = "your_ssid_password"

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
        try:
             mac_addr = wlan.config("mac")
             print("MAC地址:", mac_addr)
        except Exception as e:
             print(f"获取MAC地址失败: {e}")
    else:
        print("WiFi连接失败，程序退出。")
        return

    s = socket.socket()
    ai = []

    for attempt in range(3):
        try:
            ai = socket.getaddrinfo("www.baidu.com", 80)
            break
        except OSError as e:
            print(f"第 {attempt + 1} 次获取地址信息失败: {e}, 正在重试...")
            time.sleep(1)

    if not ai:
        print("获取目标地址失败，无法继续。")
        s.close()
        return

    addr = ai[0][-1]
    print("目标地址信息:", ai)
    print("解析出的连接地址:", addr)

    try:
        s.connect(addr)
    except OSError as e:
        print(f"连接到目标地址失败: {e}")
        s.close()
        return

    print("\n--- 开始HTTP请求 (模式: {}) ---".format("流" if use_stream else "收发"))

    if use_stream:
        # 使用流模式（类似文件读写）
        s_file = s.makefile("rwb", 0)
        s_file.write(b"GET /index.html HTTP/1.0\r\n\r\n")
        print(s_file.read())
        s_file.close()
    else:
        # 使用标准的收发模式
        s.send(b"GET /index.html HTTP/1.0\r\n\r\n")
        print(s.recv(4096))

    s.close()
    print("--- 请求结束，Socket已关闭 ---\n")


print("正在以“流”模式运行...")
main(use_stream=True)

print("正在以“收发”模式运行...")
main(use_stream=False)
