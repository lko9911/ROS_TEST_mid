import cv2
import json
import numpy as np
from ultralytics import YOLO
import os
import time

# 카메라 설정값 - 수정하지 말것
focal_length = 60    
baseline = 60  

cx = 448.0
cy = 336.0

def depth_calculate(y, x, disparity, Q):
    y = int(y)
    x = int(x)
    
    disparity_value = disparity[y, x]
    
    if disparity_value > 0:
            disparity_value = float(disparity_value)

            # 깊이 Z 계산
            Z = (focal_length * baseline) / disparity_value

            # 중심 좌표 기준으로 변환
            X = ((x - cx) * Z) / focal_length
            Y = ((y - cy) * Z) / focal_length
            Y = -Y
            print(f"3D 좌표 (카메라 기준): ({X:.2f}, {Y:.2f}, {Z:.2f}) cm")
            return X,Y,Z 
    return None  

def detect_and_save(model_path="model/best3.pt", npz_path="stereo_calibration_result.npz", save_path="detected_objects.json",time_interval=10):
    
    image_save_dir = "saved_frames" 
    os.makedirs(image_save_dir, exist_ok=True)  

    # 스테레오 로드
    # 캘리브레이션 데이터 로드 및 정의
    data = np.load(npz_path)
    K1 = data['K1']
    dist1 = data['dist1']
    K2 = data['K2']
    dist2 = data['dist2']
    R = data['R']
    T = data['T']

    # resize = 0.7 기준
    new_dim = (896, 672)

    # 스테레오 정합 및 리매핑
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K1, dist1, K2, dist2, new_dim, R, T)

    # 디스패리티 맵 계산
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 5,
        blockSize=5,
        P1=8 * 3 * 5**2,
        P2=32 * 3 * 5**2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=45,
        speckleRange=4,
    )

    # YOLO 모델 로드
    yolo_model = YOLO(model_path)

    # 웹캠 열기
    cap = cv2.VideoCapture(0)

    detected_objects = []
    last_save_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        height, width, _ = frame.shape
        img_left, img_right = frame[:, :width // 2], frame[:, width // 2:]

        img_left_resized = cv2.resize(img_left, new_dim)
        img_right_resized = cv2.resize(img_right, new_dim)

        # 왜곡 보정
        undistorted_left = cv2.undistort(img_left_resized, K1, dist1)
        undistorted_right = cv2.undistort(img_right_resized, K2, dist2)

        # 보정된 이미지를 사용하여 그레이스케일 변환
        gray_left_undistorted = cv2.cvtColor(undistorted_left, cv2.COLOR_BGR2GRAY)
        gray_right_undistorted = cv2.cvtColor(undistorted_right, cv2.COLOR_BGR2GRAY)

        # 리매핑 생성
        map1x, map1y = cv2.initUndistortRectifyMap(K1, dist1, R1, P1, new_dim, cv2.CV_32FC1)
        map2x, map2y = cv2.initUndistortRectifyMap(K2, dist2, R2, P2, new_dim, cv2.CV_32FC1)
        rectified_left = cv2.remap(gray_left_undistorted, map1x, map1y, cv2.INTER_LINEAR)
        rectified_right = cv2.remap(gray_right_undistorted, map2x, map2y, cv2.INTER_LINEAR)

        # 디스패리티 계산
        disparity = stereo.compute(rectified_left, rectified_right).astype(np.float32) / 16.0

        # 디스패리티 맵 정규화
        disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        disparity_normalized = np.uint8(disparity_normalized)
        colormap = cv2.applyColorMap(disparity_normalized, cv2.COLORMAP_JET)

        # YOLO 예측
        yolo_results_left = yolo_model.predict(img_left_resized, conf=0.5)
        yolo_img_left = yolo_results_left[0].plot()  # 좌측 YOLO 검출 결과 이미지

        # YOLO 결과에서 신뢰도 및 경계 상자 추출
        boxes_left = yolo_results_left[0].boxes  # 좌측 이미지에서 상자 추출
        scores_left = boxes_left.conf  # 좌측 신뢰도
        class_indices_left = boxes_left.cls  # 클래스 인덱스
        class_names = yolo_results_left[0].names  # 클래스 이름이 포함된 리스트

        detected_objects.clear()  # 새 프레임마다 객체 정보 초기화

        for i, (box_left, score_left) in enumerate(zip(boxes_left.xyxy, scores_left)):
            if score_left >= 0.5:  
                x1_left, y1_left, x2_left, y2_left = box_left.cpu().numpy()
                center_x = (x1_left + x2_left) / 2
                center_y = (y1_left + y2_left) / 2

                # 좌측 깊이 계산
                depth_left = depth_calculate(center_y, center_x, disparity, Q)

                # Z 값 초기화 (기본 값 설정)
                X = center_x
                Y = center_y
                Z = 60.00 
                
                '''
                # 스테레오 3D
                if depth_left is not None:
                    X,Y,Z = depth_left 
                    label_left = f"Z: {Z:.2f} cm"
                else:
                    label_left = f"Z: {Z:.2f} cm" 

                class_name = class_names[int(class_indices_left[i])]  
                '''

                
                # 60cm 전방 대상
                if depth_left is not None:
                    X,Y,Z = depth_left
                    X = center_x
                    Y = center_y
                    label_left = f"Z: {Z:.2f} cm"
                else:
                    label_left = f"Z: {Z:.2f} cm" 

                class_name = class_names[int(class_indices_left[i])]  
                

                # 객체 정보 추가
                detected_objects.append({
                    "index": i,
                    # "class_name": class_name,  
                    "X": float(X),
                    "Y": float(Y),
                    "Z": float(Z)  
                })

                # 좌측 YOLO 이미지에 깊이 및 3D 좌표 추가
                text_x_left, text_y_left = x1_left, y1_left - 25

                cv2.putText(
                    yolo_img_left,  # 이미지
                    label_left,  # 출력할 텍스트
                    (int(text_x_left), int(text_y_left)),  # 윤곽선 위치 (약간 이동시켜서 그림)
                    cv2.FONT_HERSHEY_SIMPLEX,  # 글꼴
                    0.75,  # 글꼴 크기
                    (0, 0, 255),  # 색상 (빨간색)
                    2,  # 두께
                )


        cv2.imshow('Disparity + YOLO Detection', yolo_img_left)
        cv2.imshow('colormap',colormap)

        key = cv2.waitKey(1) & 0xFF  # 키 입력값 미리 받아오기

        if key == ord('s') or time.time() - last_save_time >= time_interval:
            # 이미지 파일 이름 설정
            left_image_filename = f"left_image.jpg"
            right_image_filename = f"right_image.jpg"
            
            # 파일 경로 설정
            left_image_path = os.path.join(image_save_dir, left_image_filename)
            right_image_path = os.path.join(image_save_dir, right_image_filename)

            cv2.imwrite(left_image_path, img_left_resized)
            cv2.imwrite(right_image_path, img_right_resized)

            print(f"양쪽 이미지 저장 완료 ✅")

            # JSON 파일 저장 (이미지 경로 추가)
            save_data = {
                "detected_objects": detected_objects,
                "image_path": [left_image_path,right_image_path]  # ✅ 올바른 이미지 경로 저장
            }
            with open(save_path, "w") as f:
                json.dump(save_data, f, indent=4)

            print(f"현재 프레임의 탐지된 객체 정보가 '{save_path}' 파일에 저장되었습니다.")
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# 마우스 클릭 콜백 (지역 변수 리스트를 param으로 전달)
def mouse_callback(event, x, y, flags, param):
    image_save_dir = "saved_frames" 
    os.makedirs(image_save_dir, exist_ok=True)
    detected_objects = param["detected_objects"]
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"📌 마우스 클릭 위치: X={x}, Y={y}")
        Z = 60.0
        index = len(detected_objects)

        detected_objects.append({
            "index": index,
            "X": float(x),
            "Y": float(y),
            "Z": float(Z)
        })
        print(f"✅ 객체 저장: {detected_objects[-1]}")

        left_image_filename = f"left_image.jpg"
        right_image_filename = f"right_image.jpg"

        # 파일 경로 설정
        left_image_path = os.path.join(image_save_dir, left_image_filename)
        right_image_path = os.path.join(image_save_dir, right_image_filename)

        save_path="detected_objects.json"
        # 저장
        save_data = {
            "detected_objects": detected_objects,
            "image_path": [left_image_path, right_image_path]
        }
        with open(save_path, "w") as f:
            json.dump(save_data, f, indent=4)
        print(f"💾 JSON 저장 완료 → {save_path}")

def test_mode():
    cap = cv2.VideoCapture(0)
    new_dim = (896, 672)
    detected_objects = []  # 지역 변수로 선언

    # param으로 넘길 딕셔너리 생성
    callback_param = {"detected_objects": detected_objects}

    cv2.namedWindow('Test_image')
    cv2.setMouseCallback('Test_image', mouse_callback, callback_param)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 읽을 수 없습니다.")
            break

        height, width, _ = frame.shape
        img_left = frame[:, :width // 2]
        img_right = frame[:, width // 2:]

        img_left_resized = cv2.resize(img_left, new_dim)
        img_right_resized = cv2.resize(img_right, new_dim)

        cv2.imshow('Test_image', img_left_resized)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()