import usocket
import ussl
import json
import time
import network

def connect_wifi(ssid, password, timeout=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi:", ssid)
        wlan.connect(ssid, password)
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                print("WiFi connection timeout")
                return False
            time.sleep(1)
    print("WiFi connected:", wlan.ifconfig())
    return True

def https_post(host, url, api_key, data):
    addr = usocket.getaddrinfo(host, 443)[0][-1]
    sock = usocket.socket()
    sock.settimeout(10)
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
            "\r\n"
        ).format(url, host, api_key, content_length)

        request = headers.encode() + payload_bytes
        ssl_sock.write(request)

        resp = b""
        while True:
            chunk = ssl_sock.read(1024)
            if not chunk:
                break
            resp += chunk

        ssl_sock.close()
        sock.close()
    except Exception as e:
        print("Request error:", e)
        sock.close()
        return None

    header_end = resp.find(b"\r\n\r\n")
    if header_end == -1:
        print("Response header and body separator not found")
        return None

    body = resp[header_end+4:]
    try:
        return body.decode('utf-8')
    except:
        return body.decode('utf-8', 'ignore')

def main():
    WIFI_SSID = "Your_ssid_name"
    WIFI_PASSWORD = "Your_ssid_password"
    API_KEY = "Your_api_key"  # API KEY

    if not connect_wifi(WIFI_SSID, WIFI_PASSWORD):
        return

    host = "dashscope.aliyuncs.com"
    url = "/api/v1/services/aigc/multimodal-generation/generation"

    data = {
        "model": "qwen-audio-asr",
        "input": {
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "audio": "https://dashscope.oss-cn-beijing.aliyuncs.com/audios/welcome.mp3"
                        }
                    ]
                }
            ]
        }
    }

    print("Sending request...")
    result = https_post(host, url, API_KEY, data)
    if result:
        print("Response received:")
        print(result)
        data = json.loads(result)
        text = data["output"]["choices"][0]["message"]["content"][0]["text"]
        print(text)

    else:
        print("Request failed or no response")

if __name__ == "__main__":
    main()
