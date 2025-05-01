import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics import YOLO
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array

# 병충해 감염 여부 분류 함수
def classify_disease(cropped_img, model, threshold=0.5):
    cropped_img = cv2.resize(cropped_img, (128, 128))  
    cropped_img = cropped_img.astype('float32') / 255.0 

    image_input = np.expand_dims(cropped_img, axis=0)  
    
    probabilities = model.predict(image_input)[0]
    predicted_label_idx = np.argmax(probabilities) 
    predicted_label = 'healthy_strawberry' if probabilities[predicted_label_idx] > threshold else 'infected_strawberry'
    
    return predicted_label

def depth_calculate(y, x, disparity, Q):
    y = int(y)
    x = int(x)
    
    disparity_value = disparity[y, x]
    
    if disparity_value > 0:
        point_3D = np.array([x, y, disparity_value, 1.0], dtype=np.float32)
        
        point_3D_homogeneous = np.dot(np.linalg.inv(Q), point_3D)

        #X = point_3D_homogeneous[0] / point_3D_homogeneous[3]
        #Y = point_3D_homogeneous[1] / point_3D_homogeneous[3]
        Z = point_3D_homogeneous[2] / point_3D_homogeneous[3]

        return Z  
    return None  

def detect_and_show(model_path="model/best3.pt", npz_path="stereo_calibration_result.npz", keras_path="model/best_model.keras", time_end=1000):    
    # 모델 로드
    best_model = load_model(keras_path)
    yolo_model = YOLO(model_path)

    # 스테레오 로드
    # 캘리브레이션 데이터 로드 및 정의
    data = np.load(npz_path)
    K1 = data['K1']
    dist1 = data['dist1']
    K2 = data['K2']
    dist2 = data['dist2']
    R = data['R']
    T = data['T']

    new_dim = (896, 672)

    # 스테레오 정합 및 리매핑
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K1, dist1, K2, dist2, new_dim, R, T)

    # 파라미터 업데이트 
    focal_length_mm = 2.6  # 초점 거리 (단위: mm)
    baseline_mm = 60.0  # 베이스라인 길이 (단위: mm)
    focal_length_pixel = K1[0, 0]  # 픽셀 단위 초점 거리

    # Q 행렬 업데이트
    Q[2, 3] = focal_length_pixel * baseline_mm  # (fx * B)
    Q[3, 2] = 1.0 / baseline_mm  # (1 / B)

    # P1, P2 행렬 업데이트
    baseline_pixel = baseline_mm * focal_length_pixel / focal_length_mm  # mm에서 픽셀로 변환된 베이스라인

    # P1 행렬: 첫 번째 카메라의 투영 행렬
    P1 = np.array([
        [focal_length_pixel, 0, Q[0, 3], 0],
        [0, focal_length_pixel, Q[1, 3], 0],
        [0, 0, 1, 0]
    ])

    # P2 행렬: 두 번째 카메라의 투영 행렬 (베이스라인 적용)
    P2 = np.array([
        [focal_length_pixel, 0, Q[0, 3], -focal_length_pixel * baseline_pixel],
        [0, focal_length_pixel, Q[1, 3], 0],
        [0, 0, 1, 0]
    ])

    # K1: 첫 번째 카메라의 내부 행렬
    K1 = np.array([
        [focal_length_pixel, 0, Q[0, 3]],
        [0, focal_length_pixel, Q[1, 3]],
        [0, 0, 1]
    ], dtype=np.float64) 

    K2 = np.array([
        [focal_length_pixel, 0, Q[0, 3]],
        [0, focal_length_pixel, Q[1, 3]],
        [0, 0, 1]
    ], dtype=np.float64) 

    # 디스패리티 맵 계산
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 5,  # 16의 배수여야 함
        blockSize=5,
        P1=8 * 3 * 5**2,
        P2=32 * 3 * 5**2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=45,
        speckleRange=4,
    )

    img_left = cv2.imread('saved_frames/left_image.jpg')
    img_right = cv2.imread('saved_frames/right_image.jpg')

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

    # 좌측 및 우측 YOLO 검출 결과
    yolo_results_left = yolo_model.predict(img_left_resized, imgsz=640, conf=0.5)
    yolo_results_right = yolo_model.predict(img_right_resized, imgsz=640, conf=0.5)

    # YOLO 결과에서 신뢰도 및 경계 상자 추출
    boxes_left = yolo_results_left[0].boxes  # 좌측 이미지에서 상자 추출
    boxes_right = yolo_results_right[0].boxes  # 우측 이미지에서 상자 추출
    scores_left = boxes_left.conf  # 좌측 신뢰도
    scores_right = boxes_right.conf  # 우측 신뢰도

    # 딸기 클래스 ID 
    strawberry_class_id = 0

    yolo_img_left = yolo_results_left[0].plot()  # 좌측 YOLO 검출 결과 이미지
    yolo_img_right = yolo_results_right[0].plot()  # 우측 YOLO 검출 결과 이미지
    class_indices_left = boxes_left.cls  # 클래스 인덱스
    class_names = yolo_results_left[0].names  # 클래스 이름이 포함된 리스트
    detected_count = 0  # 검출된 객체 개수 카운트

    # 기본 분석 : 딸기에 한해서만 적용해야 함
    for i, (box_left, score_left) in enumerate(zip(boxes_left.xyxy, scores_left)):
        if score_left >= 0.5:  # 신뢰도 0.5 이상인 경우만
            x1_left, y1_left, x2_left, y2_left = box_left.cpu().numpy()

            # 검출된 객체의 클래스 이름 확인
            class_name = class_names[int(class_indices_left[i])]
            
            # "Unripe Strawberry" 또는 "Ripe Strawberry"인 경우에만 병충해 감염 여부 분류
            if class_name in ["Unripe Strawberry", "Ripe Strawberry"]:
                # 딸기 객체 잘라내기
                cropped_img_left = img_left_resized[int(y1_left):int(y2_left), int(x1_left):int(x2_left)]
                
                # 병충해 감염 여부 분류
                prediction = classify_disease(cropped_img_left, best_model)
                
                # 예측 결과에 따라 레이블 설정
                if prediction == 'infected_strawberry':
                    label_disease = "Infected"
                    label_color = (0, 0, 255)  # 빨간색
                    depth_label_color = (0, 0, 255)  
                else:
                    label_disease = "Healthy"
                    label_color = (0, 255, 0)  # 초록색
                    depth_label_color = (0, 255, 0)

                # 좌측 YOLO 이미지에 병충해 감염 여부 추가
                text_x_left, text_y_left = x1_left + 37, y1_left - 45  # 텍스트 위치

                cv2.putText(yolo_img_left, label_disease, (int(text_x_left), int(text_y_left)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 3)
                cv2.putText(yolo_img_left, label_disease, (int(text_x_left), int(text_y_left)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, label_color, 2)
    
                center_x = (x1_left + x2_left) / 2
                center_y = (y1_left + y2_left) / 2
                
                # 깊이 계산
                depth_left = depth_calculate(center_y, center_x, disparity, Q)
                
                if depth_left is not None:
                    Z = depth_left
                    label_depth = f"Z: {Z:.2f} cm"
                else:
                    label_depth = "Z: 50.00 cm"

                # 좌측 YOLO 이미지에 깊이 정보 추가 (병충해 아래에 출력)
                text_x_left, text_y_left = x1_left, y1_left - 20  

                cv2.putText(yolo_img_left, label_depth, (int(text_x_left), int(text_y_left)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 3)
                cv2.putText(yolo_img_left, label_depth, (int(text_x_left), int(text_y_left)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, depth_label_color, 2)

                # 깊이 맵에 객체의 위치 표시
                colormap = cv2.applyColorMap(disparity_normalized, cv2.COLORMAP_JET)
                cv2.circle(disparity_normalized, (int(center_x), int(center_y)), 10, (0, 0, 0), -1)
                cv2.putText(disparity_normalized, f"({int(center_x)}, {int(center_y)})", (int(center_x) + 10, int(center_y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                detected_count += 1  # 검출된 객체 개수 증가


    print(f"총 {detected_count}개의 객체 검출됨")

    # 최종 이미지를 보여줌
    cv2.imshow("YOLO Depth Left", yolo_img_left)
    
    # 10초 대기 후 종료
    cv2.waitKey(time_end)  
    cv2.destroyAllWindows()


