from typing import List, Optional
from motor.motor_asyncio import AsyncIOMotorCollection
from ....infrastructure.database.connection import DatabaseConnection
from ..models.person_models import Person, PersonCreate, PersonUpdate, ImageInfo
from datetime import datetime

class PersonRepository:
    """사용자 저장소"""
    def __init__(self):
        """초기화"""
        self.collection = None

    async def connect(self):
        """데이터베이스에 연결합니다."""
        if self.collection is None:
            db = await DatabaseConnection.get_db()
            self.collection = db.persons

    async def create_person(self, person_data: PersonCreate) -> Person:
        """사용자를 생성합니다."""
        await self.connect()
        
        person_dict = person_data.dict()
        person_dict["created_at"] = datetime.utcnow()
        person_dict["images"] = []
        
        result = await self.collection.insert_one(person_dict)
        person_dict["id"] = str(result.inserted_id)
        
        return Person(**person_dict)

    async def get_person_by_id(self, person_id: str) -> Optional[Person]:
        """ID로 사용자를 조회합니다."""
        await self.connect()
        person_dict = await self.collection.find_one({"_id": person_id})
        if person_dict:
            person_dict["id"] = str(person_dict.pop("_id"))
            return Person(**person_dict)
        return None

    async def get_person_by_email(self, email: str) -> Optional[Person]:
        """이메일로 사용자를 조회합니다."""
        await self.connect()
        person_dict = await self.collection.find_one({"email": email})
        if person_dict:
            person_dict["id"] = str(person_dict.pop("_id"))
            return Person(**person_dict)
        return None

    async def get_all_persons(self) -> List[Person]:
        """모든 사용자를 조회합니다."""
        await self.connect()
        persons = []
        async for person_dict in self.collection.find():
            person_dict["id"] = str(person_dict.pop("_id"))
            persons.append(Person(**person_dict))
        return persons

    async def update_person(self, person_id: str, person_data: PersonUpdate) -> Optional[Person]:
        """사용자 정보를 업데이트합니다."""
        await self.connect()
        update_data = {k: v for k, v in person_data.dict(exclude_unset=True).items()}
        if update_data:
            update_data["updated_at"] = datetime.utcnow()
            result = await self.collection.update_one(
                {"_id": person_id},
                {"$set": update_data}
            )
            if result.modified_count:
                return await self.get_person_by_id(person_id)
        return None

    async def delete_person(self, person_id: str) -> bool:
        """사용자를 삭제합니다."""
        await self.connect()
        result = await self.collection.delete_one({"_id": person_id})
        return result.deleted_count > 0

    async def add_person_image(self, person_id: str, image_info: ImageInfo) -> Optional[Person]:
        """사용자 이미지를 추가합니다."""
        await self.connect()
        result = await self.collection.update_one(
            {"_id": person_id},
            {"$push": {"images": image_info.dict()}}
        )
        if result.modified_count:
            return await self.get_person_by_id(person_id)
        return None