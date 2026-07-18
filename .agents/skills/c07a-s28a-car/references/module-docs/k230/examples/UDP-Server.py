import socket
import time
import network

def udpserver():
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
        ip = wlan.ifconfig()[0]
        print("WiFi连接成功！")
        print("网络信息:", wlan.ifconfig())
    else:
        print("WiFi连接失败。")
        return

    ai = socket.getaddrinfo(ip, 8080)
    addr = ai[0][-1]

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)

    print(f"UDP服务器已启动于 {ip} 端口:8080\n")

    s.settimeout(10.0)

    count = 0
    while count < 10:
        try:
            data, client_addr = s.recvfrom(1024)
            print(f"从 {client_addr} 收到消息: {data.decode('utf-8').strip()}")

            response = f"已收到的消息: {data.decode('utf-8').strip()}".encode('utf-8')
            s.sendto(response, client_addr)
            count += 1
        except OSError:
            print("等待客户端消息超时...继续等待...")
            continue
        except Exception as e:
            print(f"发生错误: {e}")
            break

    s.close()
    print("UDP 服务器已关闭。")

udpserver()
