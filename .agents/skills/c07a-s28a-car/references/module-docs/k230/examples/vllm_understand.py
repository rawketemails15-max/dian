import network
import usocket
import json
import time

# ========= WiFi 连接 =========
def connect_wifi(ssid="hiwonder", password="hiwonder", timeout=20):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("📡 正在连接 WiFi:", ssid)
        wlan.connect(ssid, password)
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                print("❌ WiFi 连接超时")
                return False
            time.sleep(1)
    print("✅ WiFi 已连接:", wlan.ifconfig())
    return True


# ========= 向 PC 端 Flask 代理发送请求 =========
def send_to_pc(pc_ip, port, image_url, question):
    data = {
        "image_url": image_url,
        "question": question
    }

    # 转为 JSON 并准备 HTTP 请求
    payload = json.dumps(data)
    payload_bytes = payload.encode("utf-8")
    content_length = len(payload_bytes)

    # 建立 TCP 连接
    print("🌐 连接到 Flask 代理服务器 {}:{} ...".format(pc_ip, port))
    addr = usocket.getaddrinfo(pc_ip, port)[0][-1]
    s = usocket.socket()
    s.settimeout(10)
    try:
        s.connect(addr)
    except Exception as e:
        print("❌ 无法连接到 Flask 代理:", e)
        s.close()
        return

    # 构建 HTTP POST 请求
    request = (
        "POST /process HTTP/1.1\r\n"
        "Host: {}\r\n"
        "User-Agent: CanMV-K230\r\n"
        "Content-Type: application/json\r\n"
        "Accept: application/json\r\n"
        "Connection: close\r\n"
        "Content-Length: {}\r\n"
        "\r\n"
    ).format(pc_ip, content_length).encode("utf-8")

    try:
        s.send(request)
        s.send(payload_bytes)
        print("✅ 请求已发送，等待响应...")
    except Exception as e:
        print("❌ 请求发送失败:", e)
        s.close()
        return

    # 读取响应
    resp = b""
    last_data_time = time.time()
    while True:
        try:
            chunk = s.recv(512)
            if chunk:
                resp += chunk
                last_data_time = time.time()
            else:
                time.sleep(0.1)
        except Exception:
            time.sleep(0.1)

        # 如果 3 秒没新数据，则认为结束
        if time.time() - last_data_time > 3:
            break

    s.close()

    if not resp:
        print("❌ 没有收到任何响应")
        return

    # 分离 HTTP 头与正文
    header_end = resp.find(b"\r\n\r\n")
    if header_end == -1:
        print("⚠️ 响应格式不正确:")
        print(resp.decode("utf-8", "ignore")[:200])
        return

    body = resp[header_end + 4:]
    print("📦 收到响应内容长度:", len(body))

    # 解析 JSON
    try:
        result_json = json.loads(body)
        if "choices" in result_json:
            content = result_json["choices"][0]["message"]["content"]
            print("\n🤖 模型回答：", content)
        elif "answer" in result_json:
            # 若 Flask 返回精简格式
            print("\n🤖 模型回答：", result_json["answer"])
        else:
            print("⚠️ 未找到回答字段，完整返回：", result_json)
    except Exception as e:
        print("⚠️ 解析响应失败:", e)
        print("原始响应：", body.decode("utf-8", "ignore"))


# ========= 主函数 =========
def main():
    WIFI_SSID = "your_ssid_name"
    WIFI_PASSWORD = "your_ssid_password"
    PC_IP = "you_PC_IP"  # ⚠️ 改成你电脑在同一局域网的 IP
    PORT = 5000

    if not connect_wifi(WIFI_SSID, WIFI_PASSWORD):
        return

    print("📡 等待网络稳定...")
    time.sleep(2)

    print("📡 K230 的局域网 IP 地址：", network.WLAN(network.STA_IF).ifconfig()[0])

    image_url = "https://help-static-aliyun-doc.aliyuncs.com/file-manage-files/zh-CN/20241022/emyrja/dog_and_girl.jpeg"
    question = "请用中文描述这张图片,在120个字以内"

    print("📤 发送请求到 Flask 代理...")
    send_to_pc(PC_IP, PORT, image_url, question)


if __name__ == "__main__":
    main()
