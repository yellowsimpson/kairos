import websocket
import json

# ROSBridge 서버 URL
server_url = "ws://192.168.0.200:9090"

# WebSocket 이벤트 핸들러: 연결 성공 시 실행
def on_open(ws):
    print("[WebSocket] Connected to ROSBridge")

    # 1. 토픽 Advertise: ROSBridge에 신규 토픽과 메시지 유형 알림
    advertise_message = {
        "op": "advertise",
        "topic": "/robot_arm",
        "type": "std_msgs/String"
    }
    ws.send(json.dumps(advertise_message))
    print("[WebSocket] Advertised /robot_arm with type std_msgs/String")

    # 2. 토픽 Publish: 데이터를 퍼블리시
    publish_message = {
        "op": "publish",
        "topic": "/robot_arm",
        "msg": {
            "data": "Robot arm is operational"
        }
    }
    ws.send(json.dumps(publish_message))
    print("[WebSocket] Published to /robot_arm: 'Robot arm is operational'")

# WebSocket 이벤트 핸들러: 오류 발생 시 실행
def on_error(ws, error):
    print(f"[WebSocket] Error: {error}")

# WebSocket 이벤트 핸들러: 연결 종료 시 실행
def on_close(ws, close_status_code, close_msg):
    print("[WebSocket] Disconnected from ROSBridge")

# WebSocket 클라이언트 생성 및 이벤트 핸들러 설정
ws = websocket.WebSocketApp(
    server_url,
    on_open=on_open,
    on_error=on_error,
    on_close=on_close
)

# WebSocket 연결 유지
ws.run_forever()
