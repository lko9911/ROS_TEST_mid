# yolo_detection/utils.py
## json 파일 읽기 및 변환 행렬 적용 예정정

import json

def load_detected_objects(file_path="detected_objects.json"):
    with open(file_path, "r") as f:
        detected_objects = json.load(f)
    return detected_objects

def print_detected_objects(detected_objects):
    if not detected_objects['detected_objects']:  # 리스트가 비어있는 경우
        print("🔍 검출된 대상이 없습니다.")
        return
    
    if 'detected_objects' in detected_objects:
        for obj in detected_objects['detected_objects']:
            print(f"index : {obj['index']}, X : {obj['X']}, Y : {obj['Y']}, Z : {obj['Z']}")
    else:
        print("⚠️ 오류: detected_objects 키가 없습니다.")
