import rospy  # ROS 퍼블리시를 위한 라이브러리
from ggul_bot.Strawberry_Vision import detect_and_save
from ggul_bot.utils import load_detected_objects, print_detected_objects
from ggul_bot.classify_disease import detect_and_show
from ggul_bot.ROS_node import publish_detected_object

def main():
    yolo_path = "model/yolov10x.pt"
    npz_path = "stereo_calibration_result_test.npz"
    keras_path = "model/best_model.keras"
    json_path = "detected_objects.json"

    i = 1

    try:
        while True:
            print(f"\n========== [{i}번째 주기 시작] ==========")
            
            # 1. YOLO 탐지 및 3D 위치 추정
            print(f"[{i}] YOLO 탐지 및 3D 위치 추정 중...")
            detect_and_save(model_path=yolo_path, npz_path=npz_path, save_path=json_path, time_interval=20)
            
            # 2. 탐지 결과 로드 및 출력
            detected_objects = load_detected_objects(json_path)
            print(f"[{i}] 탐지된 객체 정보:")
            print_detected_objects(detected_objects)

            # 탐지된 객체가 없으면 스킵
            if not detected_objects["detected_objects"]:
                print(f"[{i}] 탐지된 객체가 없습니다. 다음 주기로 넘어갑니다.")
                i += 1
                continue

            # 3. 질병 분류 수행
            print(f"[{i}] 질병 분류 모델 실행 중...")
            detect_and_show(model_path=yolo_path, npz_path=npz_path, keras_path=keras_path, time_end=10000)

            # 4. ROS2로 퍼블리시
            try:
                print(f"[{i}] ROS2 퍼블리시 시작...")
                publish_detected_object(json_path)
                print(f"[{i}] 퍼블리시 완료.")
            except rospy.ROSInterruptException:
                print(f"[{i}] ROS 퍼블리시 중단됨 (ROSInterruptException).")
            except Exception as e:
                print(f"[{i}] ROS 퍼블리시 실패: {e}")

            i += 1

    except KeyboardInterrupt:
        print("\n[종료] 사용자에 의해 중단되었습니다.")

if __name__ == "__main__":
    main()
