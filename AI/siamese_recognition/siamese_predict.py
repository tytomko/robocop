import face_recognition
import cv2
import torch
import torch.nn as nn
import pickle
import numpy as np

# ✅ 1. Siamese Network 모델 정의
import torch.nn.functional as F

class SiameseNetwork(nn.Module):
    def __init__(self):
        super(SiameseNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1)
        )

    def forward(self, encoding1, encoding2):
        distance = F.pairwise_distance(encoding1, encoding2, p=2)  # L2 Distance
        return distance


# ✅ 2. 모델 및 등록된 얼굴 데이터 로드
MODEL_PATH = "siamese_face_model.pth"
DATA_PATH = "face_encodings.pkl"

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
siamese_model = SiameseNetwork().to(device)
siamese_model.load_state_dict(torch.load(MODEL_PATH, map_location=device))
siamese_model.eval()

# 등록된 얼굴 데이터 로드
with open(DATA_PATH, "rb") as f:
    known_faces = pickle.load(f)  # { "이름": [encoding1, encoding2, ...] }

# ✅ 3. 실시간 얼굴 인식 함수
def predict_with_siamese(frame, model, distance_threshold=0.5):
    """
    Siamese Network를 이용한 실시간 얼굴 인식
    """
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, known_face_locations=face_locations)
    
    predictions = []

    for face_encoding, (top, right, bottom, left) in zip(face_encodings, face_locations):
        face_tensor = torch.tensor(face_encoding, dtype=torch.float32).to(device)
        
        best_match = ("Unknown", float("inf"))  # 초기값 (유사도 거리 최대)
        
        for name, encodings in known_faces.items():
            for stored_encoding in encodings:
                stored_tensor = torch.tensor(stored_encoding, dtype=torch.float32).to(device)
                with torch.no_grad():
                    similarity = model(face_tensor, stored_tensor).item()
                
                if similarity < best_match[1]:  # 최소 거리 저장
                    best_match = (name, similarity)

        if best_match[1] < distance_threshold:  # 유사도가 임계값보다 작으면 동일 인물로 판단
            predictions.append((best_match[0], (top, right, bottom, left), best_match[1]))
        else:
            predictions.append(("Unknown", (top, right, bottom, left), best_match[1]))

    return predictions

# ✅ 4. 실시간 얼굴 인식 실행
video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()
    if not ret:
        print("Error: 카메라에서 영상을 읽을 수 없습니다.")
        break

    predictions = predict_with_siamese(frame, siamese_model)

    for name, (top, right, bottom, left), similarity in predictions:
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.rectangle(frame, (left, bottom - 25), (right, bottom), (0, 255, 0), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, f"{name} ({similarity:.2f})", (left + 6, bottom - 6), font, 0.6, (255, 255, 255), 1)

    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ✅ 5. 종료 처리
video_capture.release()
cv2.destroyAllWindows()
