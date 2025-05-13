from ggul_bot.Strawberry_Vision import detect_and_save
from ggul_bot.Coordinate_Transformations import load_detected_objects_test, print_detected_objects_test, transform_coordinates60
from ggul_bot.Classify_Disease import detect_and_show
from ggul_bot.Raspberry_Websocket import send_detected_objects, start_joint_state_server
#from ggul_bot.Robot_Operation import read_and_send_joint_values, process_joint_angles
import asyncio
import json

async def main_loop():
    yolo_path = "model/yolov10x.pt"
    npz_path = "stereo_calibration_result_test.npz"
    keras_path = "model/best_model.keras"
    json_path = "detected_objects.json"

    queue = asyncio.Queue()  # 큐 생성

    # WebSocket 서버 실행 (큐 공유)
    asyncio.create_task(start_joint_state_server(queue))

    i = 1
    try:
        while True:
            print(f"\n========== [{i}번째 주기 시작] ==========")

            #-----------------Strawberry-Vision 실행부------------------#
            print(f"[{i}] YOLO 탐지 및 3D 위치 추정 중...")
            detect_and_save(model_path=yolo_path, npz_path=npz_path, save_path=json_path, time_interval=10000)

            transform_coordinates60(json_path)
            print(f"[{i}] 탐지된 객체 정보:")
            detected_objects = load_detected_objects_test(json_path)
            print_detected_objects_test(detected_objects)

            if not detected_objects["detected_objects"]:
                print(f"[{i}] 탐지된 객체가 없습니다. 다음 주기로 넘어갑니다.")
                i += 1
                await asyncio.sleep(10)
                continue

            # print(f"[{i}] 질병 분류 모델 실행 중...")
            # detect_and_show(model_path=yolo_path, npz_path=npz_path, keras_path=keras_path, time_end=10)

            #-----------------ROS2에 내용 전송------------------#
            print(f"[{i}] WebSocket을 통해 ROS2에 전송 중...")
            try:
                await send_detected_objects()
            except Exception as e:
                print(f"[{i}] WebSocket 통신 중 오류 발생: {e}")


            #-----------------ROS2에 내용 수신신부------------------#

            # 큐에서 수신된 모든 조인트 데이터 수집
            joint_data_list = []
            try:
                while True:
                    joint_data = await asyncio.wait_for(queue.get(), timeout=1)
                    joint_data_list.append(joint_data)
            except asyncio.TimeoutError:
                pass  # 큐가 비었을 경우 멈춤

            if joint_data_list:
                print(f"[{i}] 수신된 조인트 상태 {len(joint_data_list)}개:")
                for idx, jd in enumerate(joint_data_list, 1):
                    print(f"  {idx}: {jd}")
                # 저장
                with open("joint_states_log.json", "a") as f:
                    for jd in joint_data_list:
                        f.write(json.dumps(jd) + "\n")
            else:
                print(f"[{i}] 이번 주기에는 조인트 상태가 도착하지 않았습니다.")

            #-----------------로봇팔 동작부------------------#
            log_file_path = "joint_states_log.json"
            #read_and_send_joint_values(log_file_path)

            i += 1
            await asyncio.sleep(10)  # 주기적 실행

    except KeyboardInterrupt:
        print("🔚 프로그램 종료됨.")

# 메인 실행
if __name__ == "__main__":
    asyncio.run(main_loop())
