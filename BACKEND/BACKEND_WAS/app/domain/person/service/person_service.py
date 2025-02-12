import os
from typing import List, Optional
from fastapi import UploadFile, HTTPException
from datetime import datetime
from ..models.person_models import Person, PersonCreate, PersonUpdate, ImageInfo
from ..repository.person_repository import PersonRepository
from ....common.utils import validate_image_file, generate_unique_id
from ....common.config.manager import get_settings
import aiohttp

settings = get_settings()

MEDIA_SERVER_PATH = settings.storage.MEDIA_SERVER_URL  # Nginx 미디어 서버 경로


class PersonService:
    """사용자 서비스"""
    def __init__(self):
        """초기화"""
        self.repository = PersonRepository()
        self.media_server_url = settings.storage.MEDIA_SERVER_URL
        # UPLOAD_API_URL은 기본 URL만 포함
        self.upload_api_url = settings.storage.UPLOAD_API_URL

    async def initialize(self):
        """서비스를 초기화합니다."""
        await self.repository.connect()
        print("person service connected")

    async def create_person(self, person_data: PersonCreate, image: UploadFile) -> Person:
        """새로운 Person을 생성합니다."""
        # 이미지 유효성 검사
        if not validate_image_file(image.filename, 0):  # 파일 크기는 스트림이라 0으로 처리
            raise HTTPException(status_code=400, detail="지원하지 않는 이미지 형식입니다")

        # 이미지 저장
        ext = os.path.splitext(image.filename)[1].lower()
        image_id = generate_unique_id(f"person_{person_data.name}")  
        image_path = os.path.join(MEDIA_SERVER_PATH, f"persons_image/{image_id}{ext}")

        try:
            content = await image.read()
            
            # FormData 형식 수정
            async with aiohttp.ClientSession() as session:
                data = aiohttp.FormData()
                data.add_field('file',
                             content,
                             filename=f"{image_id}{ext}",
                             content_type=image.content_type)  # 원본 content-type 사용
                
                async with session.post(
                    f"{self.upload_api_url}/persons_image",
                    data=data  # headers 제거, FormData 사용
                ) as response:
                    if response.status != 200:
                        response_text = await response.text()
                        print(f"Upload failed: {response_text}")
                        raise HTTPException(status_code=500, detail=f"파일 업로드 실패: {response_text}")
            
            image_url = f"{self.media_server_url}/persons_image/{image_id}{ext}"
            
            # 디버깅 로그 추가
            print(f"Image uploaded successfully. URL: {image_url}")
            
            image_info = ImageInfo(
                imageId=image_id,
                url=image_url,
                uploadedAt=datetime.now(),
                imageNumber=1
            )
            
            # 디버깅 로그 추가
            print(f"Created ImageInfo: {image_info}")

            # Person 생성
            person = await self.repository.create_person(person_data)
            print(f"Created Person: {person}")  # 디버깅 로그

            # 이미지 정보 추가
            updated_person = await self.repository.add_person_image(
                identifier=person.id,  # 여기서 id가 제대로 전달되는지 확인
                image_info=image_info,
                is_name=False
            )
            print(f"Updated Person with image: {updated_person}")  # 디버깅 로그

            return updated_person

        except Exception as e:
            print(f"Error in create_person: {str(e)}")  # 디버깅 로그
            # 실패 시 이미지 파일 삭제
            if os.path.exists(image_path):
                os.remove(image_path)
            raise HTTPException(status_code=400, detail=str(e))


    async def add_person_image(self, identifier: str, person_position: str, image: UploadFile, is_name: bool = False) -> Person:
        """Person에 이미지를 추가합니다."""
        # Person 존재 확인
        if is_name:
            person = await self.repository.get_person_by_name(identifier)
        else:
            person = await self.repository.get_person_by_id(identifier)

        if not person:
            raise HTTPException(status_code=404, detail="해당 사용자를 찾을 수 없습니다")

        # 이미지 유효성 검사
        if not validate_image_file(image.filename, 0):
            raise HTTPException(status_code=400, detail="지원하지 않는 이미지 형식입니다")

        try:
            # 이미지 저장
            ext = os.path.splitext(image.filename)[1].lower()
            next_number = await self.repository.get_next_image_number(person.id)
            image_id = f"person_{person.seq}_{next_number}"
            image_path = os.path.join(MEDIA_SERVER_PATH, f"persons_image/{image_id}{ext}")

            with open(image_path, "wb") as buffer:
                content = await image.read()
                buffer.write(content)

            # 이미지 정보 생성
            image_info = ImageInfo(
                imageId=image_id,
                url=f"{MEDIA_SERVER_PATH}/persons_image/{image_id}{ext}",
                uploadedAt=datetime.now(),
                imageNumber=next_number
            )

            # 이미지 정보 추가
            updated_person = await self.repository.add_person_image(
                identifier=identifier,
                image_info=image_info,
                is_name=is_name
            )

            if not updated_person:
                raise HTTPException(status_code=500, detail="이미지 정보 업데이트에 실패했습니다")

            return updated_person

        except Exception as e:
            # 실패 시 이미지 파일 삭제
            if os.path.exists(image_path):
                os.remove(image_path)
            raise HTTPException(status_code=400, detail=str(e))

    async def update_person(self, person_id: str, person_data: PersonUpdate) -> Person:
        """Person 정보를 업데이트합니다."""
        try:
            # 이름으로 조회된 경우 ID로 변환
            if not person_id.isdigit():
                person = await self.repository.get_person_by_name(person_id)
                if not person:
                    raise HTTPException(status_code=404, detail="해당 사용자를 찾을 수 없습니다")
                person_id = person.id
            else:
                # ID로 조회
                person = await self.repository.get_person_by_id(person_id)
                if not person:
                    raise HTTPException(status_code=404, detail="해당 사용자를 찾을 수 없습니다")

            # 업데이트 수행
            updated_person = await self.repository.update_person(person_id, person_data.dict())
            if not updated_person:
                raise HTTPException(status_code=500, detail="사용자 정보 업데이트에 실패했습니다")

            return updated_person
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    async def delete_person(self, seq: int) -> bool:
        """Person을 삭제합니다."""
        # Person 존재 확인 
        person = await self.repository.get_person_by_seq(seq)
        if not person:
            raise HTTPException(status_code=404, detail="해당 사용자를 찾을 수 없습니다")

        # 이미지 파일 삭제
        for image in person.images:
            image_path = os.path.join("storage", "persons", os.path.basename(image.url))
            if os.path.exists(image_path):
                os.remove(image_path)

        # Person 삭제
        if not await self.repository.delete_person(seq=seq):
            raise HTTPException(status_code=500, detail="사용자 삭제에 실패했습니다")

        return True
    
    async def get_all_persons(self) -> List[Person]:
        """모든 Person을 조회합니다."""
        return await self.repository.get_all_persons()

    async def delete_person_by_name(self, name: str) -> bool:
        """이름으로 Person을 삭제합니다."""
        # Person 존재 확인
        person = await self.repository.get_person_by_name(name)
        if not person:
            raise HTTPException(status_code=404, detail="해당 이름의 사용자를 찾을 수 없습니다")

        # 이미지 파일 삭제
        for image in person.images:
            image_path = os.path.join("storage", "persons", os.path.basename(image.url))
            if os.path.exists(image_path):
                os.remove(image_path)

        # Person 삭제
        if not await self.repository.delete_person_by_name(name):
            raise HTTPException(status_code=500, detail="사용자 삭제에 실패했습니다")

        return True

    async def get_person_images(self, identifier: str, is_name: bool = False) -> List[ImageInfo]:
        """사용자의 모든 이미지를 조회합니다."""
        images = await self.repository.get_person_images(identifier, is_name)
        if images is None:
            raise HTTPException(status_code=404, detail="해당 사용자를 찾을 수 없습니다")
        return images

    async def delete_person_image(self, identifier: str, image_number: int, is_name: bool = False) -> bool:
        """사용자의 특정 이미지를 삭제합니다."""
        # 이미지 정보 조회
        images = await self.get_person_images(identifier, is_name)
        target_image = next((img for img in images if img.imageNumber == image_number), None)
        if not target_image:
            raise HTTPException(status_code=404, detail="해당 이미지를 찾을 수 없습니다")

        # EC2 미디어 서버에 삭제 요청
        try:
            async with aiohttp.ClientSession() as session:
                filename = os.path.basename(target_image.url)
                async with session.delete(
                    f"{self.upload_api_url}/persons_image/{filename}"
                ) as response:
                    if response.status != 200:
                        raise HTTPException(status_code=500, detail="이미지 파일 삭제 실패")

            # DB에서 이미지 정보 삭제
            if not await self.repository.delete_person_image(identifier, image_number, is_name):
                raise HTTPException(status_code=500, detail="이미지 삭제에 실패했습니다")
            return True
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))