import asyncio
 
import json

async def send_message():
    server_ip = "192.168.0.200"
    port = 9090
    publish_message = {
        "op": "publish",
        "topic": "/robot_arm",
        "msg": {
            "data": "Hello from robot arm"
        }
    }

    ws_url = f"ws://{server_ip}:{port}"

    try:
        async with websockets.connect(ws_url) as websocket:
            # 메시지를 JSON 형식으로 변환 후 전송
            await websocket.send(json.dumps(publish_message))
            print(f"Sent: {publish_message}")
            
            # 서버로부터 응답 받기
            response = await websocket.recv()
            print(f"Received: {response}")

    except Exception as e:
        print(f"Error: {e}")

# asyncio 이벤트 루프 실행
if __name__ == "__main__":
    asyncio.run(send_message())
