import asyncio
import websockets
import json

# ───── JSON 로드 함수 ─────
def load_detected_objects(file_path="detected_objects.json"):
    with open(file_path, "r") as f:
        return json.load(f)

# ───── 클라이언트: 좌표 전송 ─────
async def send_detected_objects():
    uri = "ws://192.168.150.77:8765"  # 좌표 받는 쪽
    detected = load_detected_objects()

    if not detected.get("detected_objects"):
        print("🔍 검출된 대상이 없습니다.")
        return

    await asyncio.sleep(1)  # 서버 연결 준비 시간 확보

    async with websockets.connect(uri) as websocket:
        for obj in detected["detected_objects"]:
            data = {
                "X": obj["X"],
                "Y": obj["Y"],
                "Z": obj["Z"]
            }
            await websocket.send(json.dumps(data))
            response = await websocket.recv()
            print(f"[index {obj['index']}] 서버 응답: {response}")

# ───── 서버: 조인트 상태 수신 ─────
async def receive_joint_states(websocket):
    async for message in websocket:
        try:
            data = json.loads(message)
            print("🤖 Joint States Received:", data)
            # 필요 시 파일로 저장하거나 제어 로직 삽입 가능
        except json.JSONDecodeError:
            print("⚠️ 잘못된 JSON 수신:", message)

# ───── 조인트 상태 서버 실행 ─────
async def start_joint_state_server():
    server = await websockets.serve(receive_joint_states, "0.0.0.0", 8766)
    print("🟢 조인트 상태 수신 서버 실행 중 (port 8766)...")
    await server.wait_closed()

# ───── 통합 실행 ─────
async def main():
    await asyncio.gather(
        start_joint_state_server(),  # 8766 포트에서 조인트 상태 수신
        send_detected_objects()      # 8765로 좌표 전송
    )

if __name__ == "__main__":
    asyncio.run(main())
