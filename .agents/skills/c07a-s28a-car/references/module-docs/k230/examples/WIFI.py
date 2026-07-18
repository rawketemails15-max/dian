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


# 主程序入口
if __name__ == "__main__":
    client()
