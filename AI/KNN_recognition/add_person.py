# 데이터베이스에 인물 추가
# 이미지는 다른 코드에서 받고 이 코드를 import해서 사용할것
# 이미지 받고 -> 이미지 증강 -> 올바른 경로에 저장장
import os
import random
from PIL import Image
from torchvision import transforms
import shutil

# 설정: 데이터 경로 및 저장 경로
"""
Structure:
        <train_dir>/
        ├── <person1>/
        |   ├── <origin>/
        |   |    ├── <somename1>.jpeg
        │   ├── <somename1>.jpeg
        │   ├── <somename2>.jpeg
        │   ├── ...
        ├── <person2>/
        |   ├── <origin>/
        |   |    ├── <somename1>.jpeg
        │   ├── <somename1>.jpeg
        │   └── <somename2>.jpeg
        └── ...
"""
person_name = input()
input_dir = "./new_person"  # 원본 이미지 폴더 경로
output_dir = f"./knn_database/train/{person_name}"  # 증강된 이미지 저장 폴더 경로
origin_dir = f"./knn_database/train/{person_name}/origin" # 원본 이미지 보관 폴더 경로
num_augmented_images_per_original = 10  # 원본 이미지당 생성할 증강 이미지 개수

# 증강 기법 설정
augmentation_transforms = transforms.Compose([
    transforms.RandomRotation(degrees=30),  # 30도 이내 회전
    transforms.RandomResizedCrop(size=(224, 224), scale=(0.8, 1.2)),  # 크기 조정
    transforms.RandomHorizontalFlip(p=0.5),  # 좌우 반전
    transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),  # 밝기 및 색조 조정
    transforms.RandomCrop(size=(170, 170)),  # 무작위 크롭
    transforms.ToTensor(),
    #transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # 정규화
])

# 저장 경로 생성
def ensure_dir_exists(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

ensure_dir_exists(output_dir)
ensure_dir_exists(origin_dir)

# 이미지 증강 및 저장
def augment_and_save_images(input_dir, output_dir, origin_dir):
    if len(os.listdir(input_dir)) == 0:
        print("파일이 없습니다")
        return False
    for filename in os.listdir(input_dir):
        input_path = os.path.join(input_dir, filename)

        if not filename.lower().endswith((".png", ".jpg", ".jpeg")):
            continue  # 이미지 파일이 아닌 경우 건너뜀

        with Image.open(input_path) as img:
            for i in range(num_augmented_images_per_original):
                augmented_img = augmentation_transforms(img)
                save_path = os.path.join(output_dir, f"{os.path.splitext(filename)[0]}_aug_{i}.jpg")
                # Tensor를 PIL 이미지로 변환하여 저장
                transforms.ToPILImage()(augmented_img).save(save_path)
        # 원본 이미지 이동
        shutil.move(os.path.join(input_dir,filename),os.path.join(origin_dir,filename))
    return True

# 실행
isComp = augment_and_save_images(input_dir, output_dir, origin_dir)
if isComp:
    print("Data augmentation 완료.")
else:
    print("Data augmentation 실패")

