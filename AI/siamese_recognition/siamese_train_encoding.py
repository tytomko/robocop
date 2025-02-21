import face_recognition
import os
import pickle
import numpy as np

# 데이터셋 경로 (Siamese Network 학습에 사용한 폴더)
DATASET_PATH = "encodings_database/"
ENCODINGS_PATH = "face_encodings.pkl"

# 등록할 얼굴 데이터 저장할 딕셔너리
face_encodings_dict = {}

# 데이터셋 디렉토리를 순회하며 각 인물의 얼굴 임베딩 추출
for person_name in os.listdir(DATASET_PATH):
    person_path = os.path.join(DATASET_PATH, person_name)
    if not os.path.isdir(person_path):
        continue  # 폴더가 아니면 무시

    encodings = []
    
    for img_name in os.listdir(person_path):
        img_path = os.path.join(person_path, img_name)
        image = face_recognition.load_image_file(img_path)
        encoding = face_recognition.face_encodings(image)
        
        if len(encoding) > 0:
            encodings.append(encoding[0])
            print(f"✅ {person_name} - {img_name} 얼굴 임베딩 저장 완료")
    
    # 인물당 모든 사진의 평균 벡터 저장
    if len(encodings) > 0:
        avg_encoding = np.mean(encodings, axis=0)  # 모든 사진의 평균 벡터
        face_encodings_dict[person_name] = [avg_encoding]  # 리스트 형태로 저장

# `face_encodings.pkl`로 저장
with open(ENCODINGS_PATH, "wb") as f:
    pickle.dump(face_encodings_dict, f)

print(f"✅ 총 {len(face_encodings_dict)}명의 얼굴 임베딩을 저장했습니다! (face_encodings.pkl)")
