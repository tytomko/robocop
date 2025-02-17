# AI 통합 코드 통신 위주로
# AI
import face_recognition
import cv2
import pickle
import numpy as np
from ultralytics import YOLO

# 통신
import threading
import socket
import time
import json

# 유저 코드
import socket_network
import database_update
import knn_training


# 초기화
ROBOT_NUMBER = 1
CMD_STOP = '거수자인식-정지'
CMD_RESUME = '거수자 처리완료-재개'
CMD_DISAPPEAR = '거수자 사라짐'
OTHER_COMMAND = ''

# 네트워크 설정
ROS_IP = "192.168.100.34"
BACK_IP = "172.30.1.18"
PORT = 9000

# 전역 변수로 소켓 상태를 관리
client_socket_ros = None  
client_socket_back = None

# 콜백함수
def handle_message(message, IP, PORT):
    print(f"📩 [handle_message] 수신된 메시지: {message}")  # 디버깅 추가

    try:
        data = json.loads(message)  # JSON 파싱
        print(f"📩 [handle_message] JSON 데이터: {data}")  # JSON 파싱 성공 확인

        if "response_type" in data:
            if data["response_type"] == "ALL_UPDATE_PERSON":
                print("📌 [handle_message] database_update 시작")
                database_update.init(data)
                #handle_all_update_person(data["people"])
            else:
                print(f"⚠️ [handle_message] 알 수 없는 response_type: {data['response_type']}")
        else:
            print("⚠️ [handle_message] response_type이 없습니다.")

    except json.JSONDecodeError:
        print("❌ [handle_message] JSON 파싱 오류: 유효하지 않은 데이터 형식")
    except Exception as e:
        print(f"❌ [handle_message] 메시지 처리 중 오류 발생: {e}")


def handle_all_update_person(people):
    """
    ALL_UPDATE_PERSON을 처리하는 함수
    """
    for person in people:
        person_id = person.get("person_id")
        update_time = person.get("update_time")
        image_urls = person.get("image_urls", [])
        
        print(f"🔄 업데이트된 사용자 ID: {person_id}, 업데이트 시간: {update_time}")
        print("📷 이미지 목록:")
        for url in image_urls:
            print(f"  - {url}")
        
        # 필요한 추가 처리 (예: 이미지 다운로드, 데이터베이스 업데이트 등)
        process_person_data(person_id, update_time, image_urls)


def process_person_data(person_id, update_time, image_urls):
    """
    받은 인물 데이터를 처리하는 함수 (예: DB 업데이트, 파일 저장 등)
    """
    print(f"📝 {person_id}번 사용자의 데이터를 처리합니다.")

# 콜백 함수 등록
socket_network.set_callback(handle_message)

# 소켓 연결
client_socket_back = socket_network.persistent_connect_request(BACK_IP, PORT)

if client_socket_back:
    thread_back = threading.Thread(target=socket_network.receive_messages, args=(client_socket_back, BACK_IP, PORT), daemon=True)
    thread_back.start()


# AI 모델 로드
model = YOLO("yolov8n.pt").to("cuda")

# COCO 클래스 목록 확인
class_names = model.names
person_class_id = next((class_id for class_id, class_name in class_names.items() if class_name == "person"), None)

# KNN 모델 로드
MODEL_PATH = "trained_knn_model.clf"
with open(MODEL_PATH, 'rb') as f:
    knn_clf = pickle.load(f)

# 카메라 설정
video_capture = cv2.VideoCapture(4)  # 환경에 따라 변경 필요

# 얼굴 인식 함수
def predict(frame, knn_clf, distance_threshold=0.38):
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    face_locations = face_recognition.face_locations(rgb_frame, model='cnn')
    face_encodings = face_recognition.face_encodings(rgb_frame, known_face_locations=face_locations)
    
    if not face_encodings:
        return []
    
    closest_distances = knn_clf.kneighbors(face_encodings, n_neighbors=1)
    are_matches = [closest_distances[0][i][0] <= distance_threshold for i in range(len(face_locations))]
    
    return [(pred, loc, closest_distances[0][i][0]) if rec else ("Unknown", loc, closest_distances[0][i][0]) for i, (pred, loc, rec) in enumerate(zip(knn_clf.predict(face_encodings), face_locations, are_matches))]

# 수하 관련 변수 초기화
isCheckStart = False # 수하 시작 트리거
isCheckNow = False # 수하 중
isCheckCount = 0 # 인식치
check_time = 0
isFindEnemy = False

# FPS 측정을 위한 변수 초기화
frame_count = 0
start_time = time.time()

# 프레임 - 시간 동기화 변수 초기화
previous_time = check_time

# 수하 성공 이후 쿨다운
COOLDOWN = 10
cooltime = 0
isSafe = False

# 인원이 갑자기 감소하는 경우
prev_person_count = 0

# 수상함 수치가 임계치 이상이면 수하 시작
# 수하 성공 또는 인식 성공 후 수상함 수치 초기화
# 

try:
    while True:
        # 소켓 연결 확인
        if client_socket_back is None:
            client_socket_back = socket_network.persistent_connect_request(BACK_IP, PORT)

        ret, frame = video_capture.read()
        if not ret:
            print("Error: 카메라에서 영상을 읽을 수 없습니다.")
            break
        
        # YOLO 실행
        results = model(frame, verbose=False)
        # 검출 결과 중 보행자(person)만 필터링
        person_count = 0
        for result in results:
            for box in result.boxes:
                if int(box.cls) == person_class_id:  # person 클래스 ID만 선택
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                    confidence = box.conf[0].item()  # 신뢰도              
                    
                    # 신뢰도가 50% 이상인 경우만 표시
                    if confidence > 0.5:
                        person_count += 1
                        label = f"Person: {confidence:.2f}"
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 수하 시작
        if isCheckStart:
            print("수하 시작")
            socket_network.send_command(client_socket_back,BACK_IP,PORT,"수하 시작")
            check_time = time.time()
            isCheckStart = False
            isCheckNow = True
            isCheckCount = 0
        
        if isCheckNow:
            if time.time() - check_time >= 10:
                isCheckNow = False
                isFindEnemy = True #나중에 함수로 대체
                print("신원확인에 실패하였습니다.")
                socket_network.send_command(client_socket_back,BACK_IP,PORT,"신원확인에 실패하였습니다.")
            if isCheckCount >= 100:
                isCheckNow = False
                isCheckCount = 0
                print(f"신원이 확인되었습니다.{name}")
                socket_network.send_command(client_socket_back,BACK_IP,PORT,f"신원이 확인되었습니다. {name}")
                isSafe = True
        
        safe_person_count = 0
        predictions = predict(frame, knn_clf)

        # 프레임 상관없이 일정한 시간 맞추기
        current_time = time.time()
        delta_time = current_time - previous_time
        previous_time = current_time

        # 신원 확인 과정
        for name, (top, right, bottom, left), distance in predictions:
            if name != 'Unknown':
                safe_person_count += 1

            if isCheckNow:
                if distance <= 0.38:
                    power = (10 / distance) * delta_time
                    isCheckCount += power
            
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 1)
            cv2.putText(frame, f"{name} ({distance:.2f})", (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 1)
        
        # 인증된 사람이 한명이라도 있으면 안전모드
        if safe_person_count > 0:
            isSafe = True
            cooltime = 5

        # 인식되지 않은 사람이 있으면 수하 시작
        if person_count > safe_person_count and not isCheckNow and not isSafe:
            print("신원이 확인되지 않은 인원이 있습니다. 수하를 시작합니다.")
            socket_network.send_command(client_socket_back, BACK_IP, PORT, "신원이 확인되지 않은 인원이 있습니다. 수하를 시작합니다.")
            isCheckStart = True

        # 갑자기 화면에서 인식되지 않은 사람이 사라지면 경보
        if not isSafe and prev_person_count > person_count:
            print("인가되지 않은 인원이 침입했습니다.")
            socket_network.send_command(client_socket_back, BACK_IP, PORT,"인가되지 않은 인원이 침입했습니다.")

        prev_person_count = person_count
        # 안전 쿨다운 계산
        if isSafe:
            cooltime += 1 * delta_time

            if cooltime > COOLDOWN:
                isSafe = False
                cooltime = 0

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

finally:
    print("자원 정리 중...")
    if client_socket_back:
        client_socket_back.close()
    if client_socket_ros:
        client_socket_ros.close()
    video_capture.release()
    cv2.destroyAllWindows()
    print("프로그램 종료")
