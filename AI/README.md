## 얼굴인식 AI
### KNN_recognition 
---  
- KNN 알고리즘을 이용하여 신원을 확인하는 코드   
- KNN 알고리즘은 최근접 이웃 알고리즘으로 별도의 모델 학습이 필요 없고 직관적인 구조로 구현이 쉬우나 절대적인 성능이 떨어진다는 단점이 있음.
- 현재 다소 낮은 신뢰성을 보이나 다른 딥러닝 모델이 더 나은 성능을 보여주지 못해 1순위로 개발중.
---
- add_person.py: /new_person의 사진을 이름을 입력받아 knn_database/train/에 데이터 증강하여 인물별로 저장
- knn_training.py: knn_database/train/의 인물 사진을 바탕으로 knn 모델을 학습시켜 trained_knn_model.clf로 저장.
-web_knn.py: 웹캠과 연동하여 실시간으로 인물들의 신원과 distance 정보를 분석함, q로 종료.


### siamese_recognition
---
- few-shot-learning에 최적화된 알고리즘으로 현재 기획 의도상 일반인을 대상으로 인증해야 하므로 적은 사진을 이용하여 학습해야 할 필요가 있어 채택함.
- face_encodings.pkl/ siamese_face_model.pth 두 모델을 활용하여 얼굴 인식을 시행함.
- 현재 KNN보다 나은 성능을 보여주지 못하는 중...
---
- add_person_siamese.py: /new_person의 사진을 이름을 입력받아 encodings_database/에 데이터 증강하여 인물별로 저장, 원본 사진은 siamese_database/에 저장.
- siamese_training.py: siamese_database/의 인물 사진을 바탕으로 siamese 모델을 학습시켜 siamese_face_model.pth로 저장, 사진은 인물당 2장으로 pair를 이루어야 한다.
- siamese_train_encoding.py: encodings_database/의 데이터를 학습하여 encodings_databaseface_encodings.pth로 저장
- siamese_predict.py: 웹캠과 연동하여 실시간으로 인물들의 신원과 distance 정보를 분석함, q로 종료.
   
   
---
### 추후 개발 방향
- 위의 코드들을 통합하는 main 코드 필요
- 실제 WAS와 송수신 할 수 있는 통신 코드 필요
- AI 모델 성능 증강 필요 (이렇게 구릴 리가 없음)
- 수하 프로세스 필요