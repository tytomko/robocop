import torch
import torch.nn as nn
import torch.optim as optim
import random
import face_recognition
import os
import numpy as np
from torchvision import transforms
from PIL import Image

# 데이터셋 로드
def load_face_encodings(dataset_path):
    face_encodings = []
    labels = []
    
    for person_name in os.listdir(dataset_path):
        person_path = os.path.join(dataset_path, person_name)
        if not os.path.isdir(person_path):
            continue
        
        encodings = []
        for img_name in os.listdir(person_path):
            img_path = os.path.join(person_path, img_name)
            image = face_recognition.load_image_file(img_path)
            encoding = face_recognition.face_encodings(image)
            
            if len(encoding) > 0:
                encodings.append(encoding[0])
        
        if len(encodings) > 1:
            face_encodings.append(encodings)
            labels.append(person_name)

    return face_encodings, labels

# 데이터셋 클래스 정의
class SiameseDataset(torch.utils.data.Dataset):
    def __init__(self, face_encodings, labels):
        self.face_encodings = face_encodings
        self.labels = labels
        self.data_pairs = self.create_pairs()

    def create_pairs(self):
        pairs = []
        labels = []

        for i in range(len(self.face_encodings)):
            # Positive Pair 생성
            if len(self.face_encodings[i]) > 1:
                pairs.append((self.face_encodings[i][0], self.face_encodings[i][1]))
                labels.append(1)  # 같은 사람

            # Negative Pair 생성
            j = random.randint(0, len(self.face_encodings) - 1)
            while j == i:
                j = random.randint(0, len(self.face_encodings) - 1)

            pairs.append((self.face_encodings[i][0], self.face_encodings[j][0]))
            labels.append(0)  # 다른 사람

        return list(zip(pairs, labels))

    def __len__(self):
        return len(self.data_pairs)

    def __getitem__(self, index):
        (img1, img2), label = self.data_pairs[index]
        return torch.tensor(img1, dtype=torch.float32), torch.tensor(img2, dtype=torch.float32), torch.tensor([label], dtype=torch.float32)

# 데이터 로드
dataset_path = "siamese_database/"
face_encodings, labels = load_face_encodings(dataset_path)
dataset = SiameseDataset(face_encodings, labels)
dataloader = torch.utils.data.DataLoader(dataset, batch_size=16, shuffle=True)

class SiameseNetwork(nn.Module):
    def __init__(self):
        super(SiameseNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Sigmoid()  # 유사도 점수 (0~1)
        )

    def forward(self, encoding1, encoding2):
        distance = torch.abs(encoding1 - encoding2)
        return self.fc(distance)

# 모델 초기화
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
siamese_model = SiameseNetwork().to(device)

class ContrastiveLoss(nn.Module):
    def __init__(self, margin=1.0):
        super(ContrastiveLoss, self).__init__()
        self.margin = margin

    def forward(self, outputs, labels):
        loss = labels * torch.pow(outputs, 2) + (1 - labels) * torch.pow(torch.clamp(self.margin - outputs, min=0.0), 2)
        return torch.mean(loss)
    
# 손실 함수 및 옵티마이저
criterion = ContrastiveLoss()
optimizer = optim.Adam(siamese_model.parameters(), lr=0.001)

# 학습 루프
num_epochs = 30

for epoch in range(num_epochs):
    epoch_loss = 0
    for img1, img2, label in dataloader:
        img1, img2, label = img1.to(device), img2.to(device), label.to(device)
        
        # 모델 예측
        outputs = siamese_model(img1, img2)
        
        # 손실 계산
        loss = criterion(outputs, label)
        epoch_loss += loss.item()
        
        # 역전파 및 최적화
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    
    print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {epoch_loss:.4f}")

# 모델 저장
torch.save(siamese_model.state_dict(), "siamese_face_model.pth")