from yolo_detection.Strawberry_Vision import detect_and_save
from yolo_detection.utils import load_detected_objects, print_detected_objects
from yolo_detection.classify_disease import detect_and_show

i=1

yolo_path = "model/yolov10x.pt"
npz_path = "stereo_calibration_result_test.npz"

while True:
    # 1. webcam
    print(f"{i}번째 YOLO 탐지를 시작")
    detect_and_save(model_path=yolo_path, npz_path=npz_path ,save_path="detected_objects.json", time_interval=20)

    # 2. utils.py
    detected_objects = load_detected_objects("detected_objects.json")
    print("\n저장된 탐지된 객체 정보:")
    print_detected_objects(detected_objects)

    # 3. classify_disease
    detect_and_show(model_path=yolo_path, npz_path=npz_path, keras_path="model/best_model.keras", time_end=10000)

    i = i+1