import socket
import time
import network

def tcp_http_server():
    WIFI_SSID = "hiwonder"
    WIFI_PASSWORD = "hiwonder"

    # 启动 WiFi
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f"正在连接到 WiFi 网络: {WIFI_SSID}...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        start_time = time.time()
        while not wlan.isconnected():
            if time.time() - start_time > 15:
                print("WiFi 连接超时！请检查 SSID、密码或信号强度。")
                return
            time.sleep(1)

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print("WiFi 连接成功！")
        print("网络信息:", wlan.ifconfig())
    else:
        print("WiFi 连接失败。")
        return

    CONTENT_TEMPLATE = b"""HTTP/1.0 200 OK

Hello Hiwonder k230 canmv! Request number: %d
"""
    counter = 0
    s = None
    client_sock = None

    try:
        addr = ('0.0.0.0', 8081)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass
        s.bind(addr)
        s.listen(5)
        s.setblocking(True)
        print(f"TCP HTTP 服务器已启动，访问地址: http://{ip}:8081")

        while True:
            try:
                client_sock, client_addr = s.accept()
                print(f"客户端连接来自: {client_addr}")

                try:
                    client_sock.settimeout(5)
                except Exception:
                    pass

                request = b""
                while True:
                    try:
                        chunk = client_sock.recv(1024)
                        if not chunk:
                            break
                        request += chunk
                        if b"\r\n\r\n" in request:
                            break
                    except OSError as e:
                        err_no = e.args[0] if len(e.args) > 0 else None
                        if err_no == 11:
                            time.sleep(0.1)
                            continue
                        else:
                            print("recv 出错:", e)
                            break
                    except Exception as e:
                        print("recv 异常:", e)
                        break

                print("收到请求:")
                try:
                    print(request.decode('utf-8', 'replace'))
                except Exception:
                    print(request)

                response = CONTENT_TEMPLATE % counter
                try:
                    client_sock.send(response)
                except Exception as e:
                    print("发送响应出错:", e)

                counter += 1

            except Exception as e:
                print("处理客户端请求错误:", e)

            finally:
                if client_sock:
                    try:
                        client_sock.close()
                    except Exception:
                        pass
                    client_sock = None

            time.sleep(1)

            # 如果只想处理一次请求则退出
            if counter > 0:
                print("服务器响应完毕，退出。")
                break

    finally:
        if s:
            try:
                s.close()
            except Exception:
                pass
        print("TCP HTTP 服务器已关闭。")


if __name__ == '__main__':
    tcp_http_server()
