import os
import shutil
import json
import requests
from urllib.parse import urlparse
from PIL import Image
import face_recognition
from torchvision import transforms

base_directory_name = "people_database_temp"

# 이미지 증강 기법 설정
AUGMENTATION_TRANSFORMS = transforms.Compose([
    transforms.RandomRotation(degrees=30),
    transforms.RandomResizedCrop(size=(224, 224), scale=(0.8, 1.2)),
    transforms.RandomHorizontalFlip(p=0.5),
    transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
    transforms.RandomCrop(size=(170, 170)),
    transforms.ToTensor(),
])

NUM_AUGMENTED_IMAGES = 10  # 원본 이미지당 생성할 증강 이미지 개수

def handle_all_update_person(people):
    """JSON 데이터를 기반으로 이미지 다운로드 및 처리"""
    for person in people:
        person_id = person.get("person_id")
        update_time = person.get("update_time")
        image_urls = person.get("image_urls", [])

        print(f"🔄 사용자 ID: {person_id}, 업데이트 시간: {update_time}")
        downloaded, save_path = download_images(person_id, update_time, image_urls)

        if downloaded:
            process_person_images(person_id, update_time, save_path)

def init(data):
    """메인에서 JSON을 받으면 데이터 처리를 실행"""
    if data.get("response_type") == "ALL_UPDATE_PERSON":
        handle_all_update_person(data["people"])

def ensure_dir_exists(directory):
    """지정된 디렉토리가 없으면 생성"""
    os.makedirs(directory, exist_ok=True)

def download_images(person_id, update_time, image_urls):
    """이미지 다운로드 및 `origin` 폴더에 저장"""
    global base_directory_name

    base_directory = os.path.join(os.getcwd(), base_directory_name)  # 기준 폴더 설정
    folder_name = f"{person_id}_{update_time.replace(':', '').replace(' ', '_')}"
    save_path = os.path.join(base_directory, folder_name)
    origin_path = os.path.join(save_path, "origin")  # 원본 이미지 저장 폴더

    # 폴더가 이미 존재하면 다운로드 건너뜀
    if os.path.exists(origin_path) and os.listdir(origin_path):
        print(f"🚀 {origin_path} 폴더가 이미 존재합니다. 다운로드를 건너뜁니다.")
        return False, save_path

    # 폴더 생성
    os.makedirs(origin_path, exist_ok=True)
    print(f"📁 원본 이미지 저장 폴더 생성 완료: {origin_path}")

    for url in image_urls:
        try:
            filename = os.path.basename(urlparse(url).path)
            file_path = os.path.join(origin_path, filename)

            # 이미지 다운로드
            response = requests.get(url, stream=True)
            response.raise_for_status()

            with open(file_path, "wb") as file:
                for chunk in response.iter_content(1024):
                    file.write(chunk)

            print(f"✅ 다운로드 완료: {file_path}")

        except requests.exceptions.RequestException as e:
            print(f"❌ 이미지 다운로드 실패: {url} - {e}")

    return True, save_path

def crop_face_center(image, desired_size=224, margin=1.0):
    """얼굴을 중심으로 크롭 후 지정된 크기로 변환"""
    face_locations = face_recognition.face_locations(image)
    if not face_locations:
        return None  # 얼굴을 찾지 못한 경우

    # 가장 큰 얼굴 영역 선택
    top, right, bottom, left = max(face_locations, key=lambda loc: (loc[2] - loc[0]) * (loc[1] - loc[3]))
    center_x, center_y = (left + right) // 2, (top + bottom) // 2

    crop_size = int(max(right - left, bottom - top) * (1 + margin))
    half_crop = crop_size // 2

    img_h, img_w = image.shape[:2]
    start_x = max(center_x - half_crop, 0)
    start_y = max(center_y - half_crop, 0)
    end_x = min(start_x + crop_size, img_w)
    end_y = min(start_y + crop_size, img_h)

    cropped_image = image[start_y:end_y, start_x:end_x]
    return Image.fromarray(cropped_image).resize((desired_size, desired_size), Image.LANCZOS)

def process_image(image_path, output_dir):
    """얼굴 크롭 → 증강 → 저장"""
    try:
        image = face_recognition.load_image_file(image_path)
        cropped_image = crop_face_center(image)

        if cropped_image is None:
            print(f"❌ 얼굴을 찾을 수 없습니다: {image_path}")
            return False

        for i in range(NUM_AUGMENTED_IMAGES):
            augmented_img = AUGMENTATION_TRANSFORMS(cropped_image)
            save_path = os.path.join(output_dir, f"{os.path.splitext(os.path.basename(image_path))[0]}_aug_{i}.jpg")
            transforms.ToPILImage()(augmented_img).save(save_path)

        return True

    except Exception as e:
        print(f"⚠️ 이미지 처리 오류 ({image_path}): {e}")
        return False

def process_person_images(person_id, update_time, save_path):
    """다운로드된 이미지 → 증강 및 저장"""
    origin_dir = os.path.join(save_path, "origin")  # 원본 이미지 폴더
    augments_dir = os.path.join(save_path, "augments")  # 증강된 이미지 폴더

    # 디렉토리 생성
    ensure_dir_exists(augments_dir)

    if not os.path.exists(origin_dir) or not os.listdir(origin_dir):
        print("❌ 원본 이미지 폴더가 비어 있습니다.")
        return False

    success_count = 0
    for filename in os.listdir(origin_dir):
        file_path = os.path.join(origin_dir, filename)

        if not filename.lower().endswith((".png", ".jpg", ".jpeg")):
            print(f"⏭️ 이미지 파일이 아닙니다: {filename}")
            continue

        if process_image(file_path, augments_dir):
            success_count += 1

    if success_count > 0:
        print(f"✅ {person_id}_{update_time}의 Data Augmentation 완료 ({success_count}개 파일).")
        return True
    else:
        print("⚠️ 얼굴을 찾지 못해 augmentation을 수행할 수 없습니다.")
        return False

# 예제 JSON 데이터 처리 (직접 실행할 때만 실행)
if __name__ == "__main__":
    json_data = '''{"response_type": "ALL_UPDATE_PERSON", "people": [{"person_id": 12, "update_time": "2025-02-14 11:59:26", "image_urls": ["https://i.pinimg.com/736x/ce/0d/02/ce0d02abe73cf1bb54bb7afd17d01caa.jpg", "https://cdn.slist.kr/news/photo/202410/584116_920682_1055.jpg"]}]}'''
    data = json.loads(json_data)

    if data["response_type"] == "ALL_UPDATE_PERSON":
        handle_all_update_person(data["people"])
