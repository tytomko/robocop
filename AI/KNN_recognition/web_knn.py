import face_recognition
import cv2
import pickle
import numpy as np
import time
from ultralytics import YOLO

# YOLOv8 모델 로드
model = YOLO("yolov8n.pt").to("cuda")

# COCO 클래스 목록 확인 (딕셔너리 형태)
class_names = model.names  # {0: "person", 1: "bicycle", ...}

# 보행자(person)의 클래스 ID 찾기
person_class_id = None
for class_id, class_name in class_names.items():
    if class_name == "person":
        person_class_id = class_id
        break

# KNN 모델 로드 (numpy._core 에러시 해당 환경에서 모델 재학습해야함)
MODEL_PATH = "trained_knn_model.clf"
with open(MODEL_PATH, 'rb') as f:
    knn_clf = pickle.load(f)

# RealSense 파이프라인 초기화 (리얼센스를 사용할 경우)
video_capture = cv2.VideoCapture(0) # 환경에 따라 0~4번까지 변경해야함, 리얼센스 웹캠만 쓸거면 4번


def predict(frame, knn_clf, distance_threshold=0.38):
    """
    실시간 얼굴 인식을 위한 함수
    """
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    face_locations = face_recognition.face_locations(rgb_frame, model='cnn')
    face_encodings = face_recognition.face_encodings(rgb_frame, known_face_locations=face_locations)
    
    if len(face_encodings) == 0:
        return []
    
    closest_distances = knn_clf.kneighbors(face_encodings, n_neighbors=1)
    are_matches = [closest_distances[0][i][0] <= distance_threshold for i in range(len(face_locations))]
    
    return [(pred, loc, closest_distances[0][i][0]) if rec else ("Unknown", loc, closest_distances[0][i][0]) for i, (pred, loc, rec) in enumerate(zip(knn_clf.predict(face_encodings), face_locations, are_matches))]

# 수하 관련 변수 초기화
isCheckStart = False
isCheckNow = False
isCheckCount = 0
check_time = 0
isFindEnemy = False

# FPS 측정을 위한 변수 초기화
frame_count = 0
start_time = time.time()

while True:
    ret, frame = video_capture.read()
    if not ret:
        print("Error: 카메라에서 영상을 읽을 수 없습니다.")
        break

    key = cv2.waitKey(1) & 0xFF
    if key == ord('a'):
        print("수하를 시작합니다")
        isCheckStart = True
    
    

    # YOLO 실행
    results = model(frame, verbose=False)
    # 검출 결과 중 보행자(person)만 필터링
    for result in results:
        for box in result.boxes:
            if int(box.cls) == person_class_id:  # person 클래스 ID만 선택
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                confidence = box.conf[0].item()  # 신뢰도
                
                # 신뢰도가 50% 이상인 경우만 표시
                if confidence > 0.5:
                    label = f"Person: {confidence:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # face_recognition
    predictions = predict(frame, knn_clf)
    # 얼굴 검출, 신원 확인
    for name, (top, right, bottom, left), distance in predictions:

        # 얼굴을 감싸는 박스 그리기
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 1)
        cv2.rectangle(frame, (left, bottom - 25), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        # 이름과 유사도 표시
        cv2.putText(frame, f"{name} ({distance:.2f})", (left + 6, bottom - 6), font, 0.7, (255, 255, 255), 1)
        # 수하 프로세스
        # 데이터베이스의 인물 중 여러 명이 교차되어 인식되는 경우에 대해서도 예외처리할 필요가 있음음
        if isCheckStart:
            check_time = time.time()
            isCheckStart = False
            isCheckNow = True
            isCheckCount = 0
        
        if isCheckNow:
            if distance <= 0.38:
                power = 10 / distance
                isCheckCount += power
            #print(time.time() - check_time)
            if time.time() - check_time >= 10:
                isCheckNow = False
                isFindEnemy = True #나중에 함수로 대체
                print("신원확인에 실패하였습니다.")
            if isCheckCount >= 100:
                isCheckNow = False
                isCheckCount = 0
                print(f"신원이 확인되었습니다.{name}")

    # FPS 계산
    frame_count += 1
    elapsed_time = time.time() - start_time
    if True:
        fps = frame_count / elapsed_time
        # 프레임에 FPS 표시
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # 변수 초기화
        frame_count = 0
        start_time = time.time()
    
    cv2.imshow('Video', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #time.sleep(0.1)
    
# 종료 처리
video_capture.release()
cv2.destroyAllWindows()