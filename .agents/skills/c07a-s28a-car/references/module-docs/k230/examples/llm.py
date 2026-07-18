import network
import time
import usocket
import ussl
import json

def connect_wifi(ssid, password, timeout=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("连接WiFi:", ssid)
        wlan.connect(ssid, password)
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                print("WiFi连接超时")
                return False
            time.sleep(1)
    print("WiFi已连接:", wlan.ifconfig())
    return True

def https_post(host, url, api_key, data):
    addr = usocket.getaddrinfo(host, 443)[0][-1]
    sock = usocket.socket()
    sock.settimeout(10)
    last_result = None  # 用于保存最后一次完整的结果

    try:
        sock.connect(addr)
        ssl_sock = ussl.wrap_socket(sock, server_hostname=host)

        payload = json.dumps(data)
        payload_bytes = payload.encode('utf-8')
        content_length = len(payload_bytes)

        headers = (
            "POST {} HTTP/1.1\r\n"
            "Host: {}\r\n"
            "Authorization: Bearer {}\r\n"
            "Content-Type: application/json\r\n"
            "Connection: close\r\n"
            "Content-Length: {}\r\n"
            "X-DashScope-SSE: enable\r\n"
            "\r\n"
        ).format(url, host, api_key, content_length)

        request = headers.encode('utf-8') + payload_bytes
        ssl_sock.write(request)

        resp = b""
        while True:
            chunk = ssl_sock.read(1024)
            if not chunk:
                break
            resp += chunk

        ssl_sock.close()

    except Exception as e:
        print("请求异常:", e)
        sock.close()
        return None

    print("响应字节长度:", len(resp))
    header_end = resp.find(b"\r\n\r\n")
    if header_end == -1:
        print("没找到响应头体分隔")
        return None

    body = resp[header_end + 4:]

    # 解析响应体
    try:
        lines = body.decode('utf-8').splitlines()
        for line in lines:
            if line.startswith("data:"):
                json_data = line[5:].strip()  # 获取 JSON 数据
                last_result = json_data  # 解析 JSON 数据
    except Exception as e:
        print("解析响应失败:", e)

    return last_result  # 返回最后一次完整的结果

def main():
    WIFI_SSID = "your_ssid_name"
    WIFI_PASSWORD = "your_ssid_password"
    API_KEY = "your_API_Key"   # 填写你的API Key

    if not connect_wifi(WIFI_SSID, WIFI_PASSWORD):
        return

    host = "dashscope.aliyuncs.com"
    url = "/api/v1/services/aigc/text-generation/generation"
    data = {
        "model": "qwen-plus",
        "input": {
            "messages": [
                {"role": "system", "content": "你是一个AI助手"},
                {"role": "user", "content": "联网查询深圳今天天气怎么样?"}
            ]
        },
        "parameters": {
            "result_format": "message",
            "top_p": 0.8,
            "temperature": 0.7,
            "enable_search": False,
            "enable_thinking": False,
            "thinking_budget": 4000
        }
    }

    print("发送请求中...")
    result = https_post(host, url, API_KEY, data)

    if result:
        print("收到响应:\n",result)


    else:
        print("请求失败或无响应")

if __name__ == "__main__":
    main()
