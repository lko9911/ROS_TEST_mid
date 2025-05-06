from ggul_bot.Strawberry_Vision import detect_and_save
from ggul_bot.coord_trans import load_detected_objects_test, print_detected_objects_test, transform_coordinates60
from ggul_bot.classify_disease import detect_and_show
from ggul_bot.Raspberry_Websocket import send_detected_objects, start_joint_state_server
import asyncio

async def main_loop():
    yolo_path = "model/yolov10x.pt"
    npz_path = "stereo_calibration_result_test.npz"
    keras_path = "model/best_model.keras"
    json_path = "detected_objects.json"

    i = 1

    # WebSocket 서버는 처음에 한 번 실행
    asyncio.create_task(start_joint_state_server())

    try:
        while True:
            print(f"\n========== [{i}번째 주기 시작] ==========")

            print(f"[{i}] YOLO 탐지 및 3D 위치 추정 중...")
            detect_and_save(model_path=yolo_path, npz_path=npz_path, save_path=json_path, time_interval=10000)

            transform_coordinates60(json_path)
            print(f"[{i}] 탐지된 객체 정보:")
            detected_objects = load_detected_objects_test(json_path)
            print_detected_objects_test(detected_objects)

            if not detected_objects["detected_objects"]:
                print(f"[{i}] 탐지된 객체가 없습니다. 다음 주기로 넘어갑니다.")
                i += 1
                continue

            print(f"[{i}] 질병 분류 모델 실행 중...")
            detect_and_show(model_path=yolo_path, npz_path=npz_path, keras_path=keras_path, time_end=10)

            print(f"[{i}] WebSocket을 통해 ROS2에 전송 중...")
            try:
                await send_detected_objects()
            except Exception as e:
                print(f"[{i}] WebSocket 통신 중 오류 발생: {e}")

            i += 1

    except KeyboardInterrupt:
        print("🔚 프로그램 종료됨.")

if __name__ == "__main__":
    asyncio.run(main_loop())
