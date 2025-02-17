import os
import shutil
from PIL import Image
import face_recognition
from torchvision import transforms

# 이미지 증강 기법 설정
AUGMENTATION_TRANSFORMS = transforms.Compose([
    transforms.RandomRotation(degrees=30),  # 30도 이내 회전
    transforms.RandomResizedCrop(size=(224, 224), scale=(0.8, 1.2)),  # 크기 조정
    transforms.RandomHorizontalFlip(p=0.5),  # 좌우 반전
    transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),  # 색조 조정
    transforms.RandomCrop(size=(170, 170)),  # 무작위 크롭
    transforms.ToTensor(),
])

NUM_AUGMENTED_IMAGES = 10  # 원본 이미지당 생성할 증강 이미지 개수


def ensure_dir_exists(directory):
    """지정된 디렉토리가 없으면 생성"""
    os.makedirs(directory, exist_ok=True)


def crop_face_center(image, desired_size=224, margin=1.0):
    """얼굴을 중심으로 크롭 후 지정된 크기로 변환"""
    face_locations = face_recognition.face_locations(image)

    if not face_locations:
        return None  # 얼굴을 찾지 못한 경우

    # 가장 큰 얼굴 영역 선택
    top, right, bottom, left = max(face_locations, key=lambda loc: (loc[2] - loc[0]) * (loc[1] - loc[3]))
    center_x, center_y = (left + right) // 2, (top + bottom) // 2

    # 얼굴 크기 및 마진 계산
    face_width = right - left
    face_height = bottom - top
    crop_size = int(max(face_width, face_height) * (1 + margin))
    half_crop = crop_size // 2

    # 이미지 크기 확인 후 크롭 영역 조정
    img_h, img_w = image.shape[:2]
    start_x = max(center_x - half_crop, 0)
    start_y = max(center_y - half_crop, 0)
    end_x = min(start_x + crop_size, img_w)
    end_y = min(start_y + crop_size, img_h)

    # 크롭 후 리사이즈
    cropped_image = image[start_y:end_y, start_x:end_x]
    pil_image = Image.fromarray(cropped_image).resize((desired_size, desired_size), Image.LANCZOS)

    return pil_image


def process_image(image_path, output_dir, origin_dir):
    """단일 이미지 처리: 얼굴 크롭 → 증강 → 저장"""
    try:
        image = face_recognition.load_image_file(image_path)
        cropped_image = crop_face_center(image)

        if cropped_image is None:
            print(f"❌ 얼굴을 찾을 수 없습니다: {image_path}")
            return False

        # 원본 이미지 저장 경로
        os.makedirs(origin_dir, exist_ok=True)

        for i in range(NUM_AUGMENTED_IMAGES):
            augmented_img = AUGMENTATION_TRANSFORMS(cropped_image)
            save_path = os.path.join(output_dir, f"{os.path.splitext(os.path.basename(image_path))[0]}_aug_{i}.jpg")
            transforms.ToPILImage()(augmented_img).save(save_path)

        # 원본 이미지 이동
        shutil.move(image_path, os.path.join(origin_dir, os.path.basename(image_path)))
        return True

    except Exception as e:
        print(f"⚠️ 이미지 처리 중 오류 발생 ({image_path}): {e}")
        return False


def process_person_images(person_name, input_dir="./new_person"):
    """새로운 사람의 이미지를 받아 증강 및 저장"""
    output_dir = f"./knn_database/train/{person_name}"
    origin_dir = os.path.join(output_dir, "origin")

    # 디렉토리 생성
    ensure_dir_exists(output_dir)
    ensure_dir_exists(origin_dir)

    if not os.path.exists(input_dir) or not os.listdir(input_dir):
        print("❌ 입력 폴더가 비어 있습니다.")
        return False

    success_count = 0
    for filename in os.listdir(input_dir):
        file_path = os.path.join(input_dir, filename)

        if not filename.lower().endswith((".png", ".jpg", ".jpeg")):
            print(f"⏭️ 이미지 파일이 아닙니다: {filename}")
            continue

        if process_image(file_path, output_dir, origin_dir):
            success_count += 1

    if success_count > 0:
        print(f"✅ {person_name}의 Data Augmentation 완료 ({success_count}개 파일).")
        return True
    else:
        print("⚠️ 얼굴을 찾지 못해 augmentation을 수행할 수 없습니다.")
        return False


# 실행
if __name__ == "__main__":
    person_name = input("추가할 인물의 이름을 입력하세요: ")
    process_person_images(person_name)
