import os
from typing import List, Optional
from fastapi import UploadFile, HTTPException
from datetime import datetime
from ..models.person_models import Person, PersonCreate, PersonUpdate, ImageInfo
from ..repository.person_repository import PersonRepository
from ....common.utils import validate_image_file, generate_unique_id

UPLOAD_DIR = "storage/persons"
os.makedirs(UPLOAD_DIR, exist_ok=True)

class PersonService:
    """사용자 서비스"""
    def __init__(self):
        """초기화"""
        self.repository = PersonRepository()

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
        image_path = os.path.join(UPLOAD_DIR, f"{image_id}{ext}")

        try:
            with open(image_path, "wb") as buffer:
                content = await image.read()
                buffer.write(content)

            # 이미지 정보 생성
            image_info = ImageInfo(
                imageId=image_id,
                url=f"/storage/persons/{image_id}{ext}",
                uploadedAt=datetime.now()
            )

            # Person 생성
            person = await self.repository.create_person(person_data)

            # 이미지 정보 추가
            updated_person = await self.repository.add_person_image(
                person_id=person.id,
                image_info=image_info
            )

            return updated_person

        except Exception as e:
            # 실패 시 이미지 파일 삭제
            if os.path.exists(image_path):
                os.remove(image_path)
            raise HTTPException(status_code=400, detail=str(e))


    async def add_person_image(self, person_id: int, image: UploadFile) -> Person:
        """Person에 이미지를 추가합니다."""
        # Person 존재 확인
        person = await self.repository.get_person_by_id(person_id)
        if not person:
            raise HTTPException(status_code=404, detail="해당 사용자를 찾을 수 없습니다")

        # 이미지 유효성 검사
        if not validate_image_file(image.filename, 0):
            raise HTTPException(status_code=400, detail="지원하지 않는 이미지 형식입니다")

        # 이미지 저장
        ext = os.path.splitext(image.filename)[1].lower()
        image_id = generate_unique_id(f"person_{person_id}")
        image_path = os.path.join(UPLOAD_DIR, f"{image_id}{ext}")

        try:
            with open(image_path, "wb") as buffer:
                content = await image.read()
                buffer.write(content)

            # 이미지 정보 생성
            image_info = ImageInfo(
                imageId=image_id,
                url=f"/storage/persons/{image_id}{ext}",
                uploadedAt=datetime.now()
            )

            # 이미지 정보 추가
            updated_person = await self.repository.add_image(person_id, image_info)
            if not updated_person:
                raise HTTPException(status_code=500, detail="이미지 정보 업데이트에 실패했습니다")

            return updated_person

        except Exception as e:
            # 실패 시 이미지 파일 삭제
            if os.path.exists(image_path):
                os.remove(image_path)
            raise HTTPException(status_code=400, detail=str(e))

    async def update_person(self, person_id: int, person_data: PersonUpdate) -> Person:
        """Person 정보를 업데이트합니다."""
        # Person 존재 확인
        person = await self.repository.get_person_by_id(person_id)
        if not person:
            raise HTTPException(status_code=404, detail="해당 사용자를 찾을 수 없습니다")

        # 업데이트 수행
        updated_person = await self.repository.update_person(person_id, person_data.dict())
        if not updated_person:
            raise HTTPException(status_code=500, detail="사용자 정보 업데이트에 실패했습니다")

        return updated_person

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