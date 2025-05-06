import json

def load_detected_objects_test(file_path="detected_objects.json"):
    with open(file_path, "r") as f:
        detected_objects = json.load(f)
    return detected_objects

def print_detected_objects_test(detected_objects):
    if not detected_objects['detected_objects']:  # 리스트가 비어있는 경우
        print("🔍 검출된 대상이 없습니다.")
        return
    
    if 'detected_objects' in detected_objects:
        for obj in detected_objects['detected_objects']:
            print(f"index : {obj['index']}, X : {obj['X']}, Y : {obj['Y']}, Z : {obj['Z']}")
    else:
        print("⚠️ 오류: detected_objects 키가 없습니다.")

def transform_coordinates60(file_path="detected_objects.json"):
    with open(file_path, "r") as f:
        data = json.load(f)

    transformed = []
    for obj in data["detected_objects"]:
        new_x = (obj["X"] - 472) / 100
        new_y = -(obj["Y"] - 406) / 100
        new_z = obj["Z"] / 100
        transformed.append({
            "index": obj["index"],
            "X": new_x,
            "Y": new_y,
            "Z": new_z
        })

    # 변환된 데이터를 원래 데이터에 저장
    data["detected_objects"] = transformed

    # 파일 덮어쓰기
    with open(file_path, "w") as f:
        json.dump(data, f, indent=4)

    return transformed
