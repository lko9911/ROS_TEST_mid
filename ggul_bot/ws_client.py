import asyncio
import websockets
import json

# JSON 파일에서 검출된 객체 불러오기
def load_detected_objects(file_path="detected_objects.json"):
    with open(file_path, "r") as f:
        detected_objects = json.load(f)
    return detected_objects

# WebSocket을 통해 각 객체 전송
async def send_detected_objects():
    uri = "ws://192.168.150.77:8765"
    detected = load_detected_objects()

    if not detected.get("detected_objects"):
        print("🔍 검출된 대상이 없습니다.")
        return

    async with websockets.connect(uri) as websocket:
        for obj in detected["detected_objects"]:
            data = {
                "index": obj["index"],
                "X": obj["X"],
                "Y": obj["Y"],
                "Z": obj["Z"]
            }
            await websocket.send(json.dumps(data))
            response = await websocket.recv()
            print(f"[index {obj['index']}] 서버 응답: {response}")

# 실행
asyncio.run(send_detected_objects())
