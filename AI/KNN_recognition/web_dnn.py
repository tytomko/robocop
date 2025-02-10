import face_recognition
import cv2
import pickle
import numpy as np
import time
import keyboard

# KNN 모델 로드 (numpy._core 에러시 해당 환경에서 모델 재학습해야함)
MODEL_PATH = "trained_knn_model.clf"
with open(MODEL_PATH, 'rb') as f:
    knn_clf = pickle.load(f)

# KNN 대신 미리 학습된 DNN 모델을 사용 (config, model)
net = cv2.dnn.readNet("deploy.prototxt","res10_300x300_ssd_iter_140000.caffemodel")

# 비디오 입력 받아옴
video_capture = cv2.VideoCapture(0) # 환경에 따라 0~4번까지 변경해야함, 리얼센스 웹캠만 쓸거면 4번
    

def predict(frame, knn_clf, distance_threshold=0.38):
    """
    실시간 얼굴 인식을 위한 함수
    """
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # DNN 얼굴 검출
    # DNN 모델을 사용하기 위해 300,300 사이즈로 blob화
    (h, w) = rgb_frame.shape[:2]

    # DNN 얼굴 검출
    blob = cv2.dnn.blobFromImage(rgb_frame, 1.0, (300, 300), (104.0, 177.0, 123.0))
    net.setInput(blob)
    detections = net.forward()

    face_locations = []
    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            face_locations.append((startY, endX, endY, startX))
    #face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, known_face_locations=face_locations)
    
    if len(face_encodings) == 0:
        return []
    
    closest_distances = knn_clf.kneighbors(face_encodings, n_neighbors=1)
    are_matches = [closest_distances[0][i][0] <= distance_threshold for i in range(len(face_locations))]
    
    return [(pred, loc, closest_distances[0][i][0]) if rec else ("Unknown", loc, closest_distances[0][i][0]) for i, (pred, loc, rec) in enumerate(zip(knn_clf.predict(face_encodings), face_locations, are_matches))]


isCheckStart = False
isCheckNow = False
isCheckCount = 0
check_time = 0
isFindEnemy = False
while True:
    ret, frame = video_capture.read()
    if not ret:
        print("Error: 카메라에서 영상을 읽을 수 없습니다.")
        break
    
    predictions = predict(frame, knn_clf)

    if keyboard.is_pressed("a"):
        print("수하를 시작합니다")
        isCheckStart = True
       
    for name, (top, right, bottom, left), distance in predictions:
        top *= 1
        right *= 1
        bottom *= 1
        left *= 1

        # 얼굴을 감싸는 박스 그리기
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 1)
        cv2.rectangle(frame, (left, bottom - 25), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        # 이름과 유사도 표시
        cv2.putText(frame, f"{name} ({distance:.2f})", (left + 6, bottom - 6), font, 0.7, (255, 255, 255), 1)
        # 수하 프로세스
        if isCheckStart:
            check_time = time.time()
            isCheckStart = False
            isCheckNow = True
            isCheckCount = 0
        
        if isCheckNow:
            if distance <= 0.38:
                power = 10 / distance
                isCheckCount += power
            print(time.time() - check_time)
            if time.time() - check_time >= 10:
                isCheckNow = False
                isFindEnemy = True #나중에 함수로 대체
                print("신원확인에 실패하였습니다.")
            if isCheckCount >= 100:
                isCheckNow = False
                isCheckCount = 0
                print(f"신원이 확인되었습니다.{name}")
                    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.imshow('Video', frame)

    
    
# 종료 처리
video_capture.release()
cv2.destroyAllWindows()
