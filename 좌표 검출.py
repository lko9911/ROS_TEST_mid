from yolo_detection.Strawberry_Vision import detect_and_save
from yolo_detection.utils import load_detected_objects, print_detected_objects
from yolo_detection.classify_disease import detect_and_show

yolo_path = "model/yolov10x.pt"
npz_path = "stereo_calibration_result_test.npz"

# 1. Strawberry_Vision
detect_and_save(model_path=yolo_path, npz_path=npz_path ,save_path="detected_objects.json", time_interval=2000)

# 2. utils.py
detected_objects = load_detected_objects("detected_objects.json")
print("\n저장된 탐지된 객체 정보:")
print_detected_objects(detected_objects)
