from flask import Flask, request, jsonify
import requests

app = Flask(__name__)

# ==== 配置信息 ====
DASHSCOPE_API_KEY = "your_API_key"
DASHSCOPE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"


@app.route('/process', methods=['POST'])
def process():
    """K230 发送请求 -> Flask 转发到 DashScope -> 返回结果"""
    try:
        data = request.get_json(force=True)
        print("🛰️ 收到来自 K230 的请求:", data)
    except Exception as e:
        return jsonify({"error": f"JSON 解析失败: {str(e)}"}), 400

    payload = {
        "model": "qwen3-vl-plus",
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": data.get("image_url")}},
                    {"type": "text", "text": data.get("question", "请用中文描述这张图片")}
                ]
            }
        ]
    }

    headers = {
        "Authorization": f"Bearer {DASHSCOPE_API_KEY}",
        "Content-Type": "application/json"
    }

    try:
        # 转发到 DashScope
        r = requests.post(DASHSCOPE_URL, headers=headers, json=payload, timeout=30)
        print(f"✅ DashScope 响应状态: {r.status_code}")
        # 原样转发响应体
        return (r.text, r.status_code, {"Content-Type": "application/json"})
    except Exception as e:
        print("❌ DashScope 请求异常:", e)
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    print("✅ Flask 代理已启动：http://0.0.0.0:5000")
    print("📡 确保 K230 可访问此电脑的局域网 IP")
    app.run(host='0.0.0.0', port=5000)
