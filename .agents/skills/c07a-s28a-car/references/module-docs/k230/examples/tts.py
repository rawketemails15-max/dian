import usocket
import ussl
import network
import time
import ubinascii
import urandom
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

def create_ws_key():
    b = bytearray()
    for _ in range(4):
        val = urandom.getrandbits(32)
        b.extend(val.to_bytes(4, 'big'))
    return ubinascii.b2a_base64(b).strip().decode()

def websocket_handshake(ssl_sock, host, path, api_key):
    ws_key = create_ws_key()
    handshake = (
        "GET {} HTTP/1.1\r\n"
        "Host: {}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: {}\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "Authorization: bearer {}\r\n"
        "X-DashScope-DataInspection: enable\r\n"
        "\r\n"
    ).format(path, host, ws_key, api_key)

    ssl_sock.write(handshake.encode())

    # 读取 HTTP 响应头直到空行
    while True:
        line = ssl_sock.readline()
        if not line or line == b'\r\n':
            break
        print(line.decode().strip())
    return ws_key

def mask_payload(payload, mask_key):
    return bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))

def send_ws_text(ssl_sock, text):
    payload = text.encode()
    length = len(payload)
    frame = bytearray()

    # FIN=1，opcode=1 (文本帧)
    frame.append(0x81)

    # 设置掩码位，长度字段处理
    if length <= 125:
        frame.append(0x80 | length)  # MASK=1 + 长度
    elif length <= 65535:
        frame.append(0x80 | 126)
        frame.extend(length.to_bytes(2, 'big'))
    else:
        frame.append(0x80 | 127)
        frame.extend(length.to_bytes(8, 'big'))

    # 生成4字节掩码key
    mask_key = urandom.getrandbits(32).to_bytes(4, 'big')
    frame.extend(mask_key)

    # 掩码处理负载
    masked_payload = mask_payload(payload, mask_key)
    frame.extend(masked_payload)

    ssl_sock.write(frame)
    print("已发送文本消息:", text)

def safe_read(sock, size):
    data = b''
    while len(data) < size:
        chunk = sock.read(min(size - len(data), 512))
        if not chunk:
            # 连接关闭或无数据
            if data:
                return data
            return None
        data += chunk
    return data

def recv_ws_message(ssl_sock):
    header = safe_read(ssl_sock, 2)
    if not header or len(header) < 2:
        return None

    fin = (header[0] >> 7) & 1
    opcode = header[0] & 0x0F
    masked = (header[1] >> 7) & 1
    payload_len = header[1] & 0x7F

    if payload_len == 126:
        ext_len = safe_read(ssl_sock, 2)
        if not ext_len or len(ext_len) < 2:
            return None
        payload_len = int.from_bytes(ext_len, 'big')
    elif payload_len == 127:
        ext_len = safe_read(ssl_sock, 8)
        if not ext_len or len(ext_len) < 8:
            return None
        payload_len = int.from_bytes(ext_len, 'big')

    if payload_len < 0:
        # 非法长度
        return None

    mask_key = safe_read(ssl_sock, 4) if masked else None

    payload = safe_read(ssl_sock, payload_len)
    if not payload:
        return None

    if masked and mask_key:
        payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))

    if opcode == 1:  # 文本帧
        try:
            return payload.decode('utf-8')
        except:
            return None
    elif opcode == 2:  # 二进制帧
        return payload
    elif opcode == 8:  # 关闭帧
        print("收到关闭帧，连接关闭")
        return None
    else:
        return None

def main():
    WIFI_SSID = "your_ssid_name" #
    WIFI_PASSWORD = "your_ssid_password" #
    API_KEY = "your_api_key"  # API KEY
    HOST = "dashscope.aliyuncs.com"
    PORT = 443
    PATH = "/api-ws/v1/inference/"
    OUTPUT_FILE = "/sdcard/output_0001.mp3"
    # 发送多条合成文本（continue-task）
    texts = [
        "床前明月光，疑是地上霜",
        "举头望明月，低头思故乡"
    ]

    if not connect_wifi(WIFI_SSID, WIFI_PASSWORD):
        return

    addr_info = usocket.getaddrinfo(HOST, PORT)[0][-1]
    sock = usocket.socket()
    sock.connect(addr_info)
    ssl_sock = ussl.wrap_socket(sock, server_hostname=HOST)

    ws_key = websocket_handshake(ssl_sock, HOST, PATH, API_KEY)
    task_id = ws_key  # 任务ID用握手key保证唯一

    # 发送启动任务命令 (run-task)
    run_task_cmd = {
        "header": {
            "action": "run-task",
            "task_id": task_id,
            "streaming": "duplex"
        },
        "payload": {
            "task_group": "audio",
            "task": "tts",
            "function": "SpeechSynthesizer",
            "model": "cosyvoice-v2",
            "parameters": {
                "text_type": "PlainText",
                "voice": "longxiaochun_v2",
                "format": "mp3",
                "sample_rate": 22050,
                "volume": 50,
                "rate": 1,
                "pitch": 1
            },
            "input": {}
        }
    }
    send_ws_text(ssl_sock, json.dumps(run_task_cmd))


    for t in texts:
        continue_task_cmd = {
            "header": {
                "action": "continue-task",
                "task_id": task_id,
                "streaming": "duplex"
            },
            "payload": {
                "input": {
                    "text": t
                }
            }
        }
        send_ws_text(ssl_sock, json.dumps(continue_task_cmd))
        time.sleep(0.1)

    # 发送结束任务命令 (finish-task)
    finish_task_cmd = {
        "header": {
            "action": "finish-task",
            "task_id": task_id,
            "streaming": "duplex"
        },
        "payload": {
            "input": {}
        }
    }
    send_ws_text(ssl_sock, json.dumps(finish_task_cmd))

    # 接收服务器响应消息，保存音频数据
    try:
        with open(OUTPUT_FILE, "wb") as f:
            while True:
                msg = recv_ws_message(ssl_sock)
                print("msg:",msg)
                if msg is None:
                    print("连接关闭或无数据，退出循环")
                    break
                if isinstance(msg, str):
                    print("收到文本消息:", msg)
                elif isinstance(msg, bytes):
                    print(f"收到二进制消息，大小:{len(msg)}字节")
                    f.write(msg)
                else:
                    print("收到未知类型消息:", type(msg))
    except Exception as e:
        print("接收异常:", e)
    ssl_sock.close()
    sock.close()
    print("音频已保存到:", OUTPUT_FILE)

if __name__ == "__main__":
    main()
