#配置 tcp/udp socket调试工具
import network
import socket
import time

def client():

    # 填入WiFi信息
    WIFI_SSID = 'your_ssid_name'
    WIFI_PASSWORD = 'your_ssid_password'
    # 初始化WLAN接口为STA模式
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

    # 确保已连接并获取到IP地址
    if wlan.isconnected():
        print("WiFi连接成功！")
        # 打印网络信息 (IP, 子网掩码, 网关, DNS)
        print("网络信息:", wlan.ifconfig())
    else:
        print("WiFi连接失败。")
        return
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)

    # 获取服务器地址及端口号 (地址可参考教程进行填写）
    try:
        ai = socket.getaddrinfo("192.168.140.86", 8080)
        addr = ai[0][-1]
        print("目标服务器地址:", addr)
    except OSError as e:
        print(f"获取地址信息失败: {e}")
        return

    # 连接到服务器地址
    try:
        s.connect(addr)
        print("成功连接到服务器。")
    except OSError as e:
        s.close()
        print(f"服务器连接错误: {e}")
        return

    for i in range(10):
        # 准备要发送的字符串
        message = "hiwonder k230 tcp {0} \r\n".format(i)
        print("正在发送 -> ", message.strip())

        # 发送字符串
        s.write(message.encode('utf-8'))
        time.sleep(0.2)

    time.sleep(1)
    s.close()
    print("end")

# 主程序入口
if __name__ == "__main__":
    client()
