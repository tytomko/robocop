import os
import shutil
from PIL import Image
from torchvision import transforms
import face_recognition

# 설정: 데이터 경로 및 저장 경로
person_name = input("추가할 인물의 이름을 입력하세요: ")
input_dir = "./new_person"  # 원본 이미지 폴더 경로
output_dir = f"./knn_database/train/{person_name}"  # 증강된 이미지 저장 폴더 경로
origin_dir = f"./knn_database/train/{person_name}/origin"  # 원본 이미지 보관 폴더 경로
num_augmented_images_per_original = 10  # 원본 이미지당 생성할 증강 이미지 개수

# 증강 기법 설정
augmentation_transforms = transforms.Compose([
    transforms.RandomRotation(degrees=30),  # 30도 이내 회전
    transforms.RandomResizedCrop(size=(224, 224), scale=(0.8, 1.2)),  # 크기 조정
    transforms.RandomHorizontalFlip(p=0.5),  # 좌우 반전
    transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),  # 밝기 및 색조 조정
    transforms.RandomCrop(size=(170, 170)),  # 무작위 크롭
    transforms.ToTensor(),
])

# 저장 경로 생성
def ensure_dir_exists(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

ensure_dir_exists(output_dir)
ensure_dir_exists(origin_dir)

# 얼굴 중심으로 크롭하는 함수
def crop_face_center(image, desired_size=224, margin=1.0):
    # face_recognition을 사용하여 얼굴 위치 검출
    face_locations = face_recognition.face_locations(image)

    if len(face_locations) == 0:
        return None  # 얼굴을 찾지 못한 경우

    # 가장 큰 얼굴 영역 선택
    top, right, bottom, left = max(face_locations, key=lambda loc: (loc[2] - loc[0]) * (loc[1] - loc[3]))

    # 얼굴 중심 좌표 계산
    center_x, center_y = (left + right) // 2, (top + bottom) // 2

    # 얼굴 영역의 너비와 높이 계산
    face_width = right - left
    face_height = bottom - top

    # 마진을 고려한 크롭 영역 크기 계산
    crop_size = int(max(face_width, face_height) * (1 + margin))
    half_crop = crop_size // 2

    # 원본 이미지 크기
    img_h, img_w = image.shape[:2]

    # 크롭 영역의 좌상단 좌표 계산
    start_x = max(center_x - half_crop, 0)
    start_y = max(center_y - half_crop, 0)

    # 크롭 영역이 이미지 경계를 넘지 않도록 조정
    end_x = min(start_x + crop_size, img_w)
    end_y = min(start_y + crop_size, img_h)

    # 최종 크롭 영역
    cropped_image = image[start_y:end_y, start_x:end_x]

    # 원하는 크기로 리사이즈
    pil_image = Image.fromarray(cropped_image)
    pil_image = pil_image.resize((desired_size, desired_size), Image.LANCZOS)

    return pil_image

# 이미지 증강 및 저장
def augment_and_save_images(input_dir, output_dir, origin_dir):
    if len(os.listdir(input_dir)) == 0:
        print("파일이 없습니다")
        return False

    for filename in os.listdir(input_dir):
        input_path = os.path.join(input_dir, filename)

        if not filename.lower().endswith((".png", ".jpg", ".jpeg")):
            continue  # 이미지 파일이 아닌 경우 건너뜀

        # face_recognition을 사용하여 이미지 로드
        image = face_recognition.load_image_file(input_path)

        # 얼굴 중심으로 크롭
        cropped_image = crop_face_center(image)

        if cropped_image is None:
            print(f"얼굴을 찾을 수 없습니다: {filename}")
            continue

        for i in range(num_augmented_images_per_original):
            augmented_img = augmentation_transforms(cropped_image)
            save_path = os.path.join(output_dir, f"{os.path.splitext(filename)[0]}_aug_{i}.jpg")
            # Tensor를 PIL 이미지로 변환하여 저장
            transforms.ToPILImage()(augmented_img).save(save_path)

        # 원본 이미지 이동
        shutil.move(input_path, os.path.join(origin_dir, filename))

    return True

# 실행
isComp = augment_and_save_images(input_dir, output_dir, origin_dir)
if isComp:
    print("Data augmentation 완료.")
else:
    print("Data augmentation 실패")
