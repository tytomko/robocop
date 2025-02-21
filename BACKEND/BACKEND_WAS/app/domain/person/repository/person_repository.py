from typing import List, Optional
from motor.motor_asyncio import AsyncIOMotorCollection
from ....infrastructure.database.connection import DatabaseConnection
from ..models.person_models import Person, PersonCreate, PersonUpdate, ImageInfo
from datetime import datetime
from bson import ObjectId

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
        
        # 다음 seq 값을 가져옵니다
        db = await DatabaseConnection.get_db()
        counter = await db.counters.find_one_and_update(
            {"_id": "person_id"},
            {"$inc": {"seq": 1}},
            upsert=True,
            return_document=True
        )
        
        person_dict = person_data.dict()
        person_dict.update({
            "seq": counter["seq"],
            "createdAt": datetime.utcnow(),
            "images": [],
            "isDeleted": False
        })
        
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
            update_data["updatedAt"] = datetime.utcnow() # updated_at -> updatedAt
            result = await self.collection.update_one(
                {"_id": person_id},
                {"$set": update_data}
            )
            if result.modified_count:
                return await self.get_person_by_id(person_id)
        return None

    async def delete_person(self, seq: int) -> bool:
        """사용자를 소프트 삭제합니다."""
        await self.connect()
        
        # seq로 조회 조건 생성
        query = {"seq": seq, "isDeleted": False}
            
        # 소프트 딜리트를 위한 업데이트
        result = await self.collection.update_one(
            query,
            {
                "$set": {
                    "deletedAt": datetime.utcnow(),
                    "isDeleted": True,  # true -> False
                    "updatedAt": datetime.utcnow()
                }
            }
        )
        return result.modified_count > 0

    async def get_next_image_number(self, person_id: str) -> int:
        """해당 person의 다음 이미지 번호를 가져옵니다."""
        await self.connect()
        person = await self.collection.find_one({"_id": person_id})
        if not person or "images" not in person:
            return 1
        return len(person["images"]) + 1

    async def get_person_id_by_name(self, name: str) -> Optional[str]:
        """이름으로 person의 id를 조회합니다."""
        await self.connect()
        person = await self.collection.find_one({"name": name, "isDeleted": False})
        if person:
            return str(person["_id"])
        return None

    async def add_person_image(self, identifier: str, image_info: ImageInfo, is_name: bool = False) -> Optional[Person]:
        """사용자 이미지를 추가합니다.
        Args:
            identifier: person의 id 또는 name
            image_info: 이미지 정보
            is_name: identifier가 name인지 여부
        """
        await self.connect()
        
        try:
            # name으로 검색할 경우 id로 변환
            person_id = await self.get_person_id_by_name(identifier) if is_name else identifier
            if not person_id:
                return None
            
            # ObjectId로 변환 - 여기가 핵심
            person_id = ObjectId(person_id) if isinstance(person_id, str) else person_id
            
            # 다음 이미지 번호 가져오기
            next_image_number = await self.get_next_image_number(person_id)
            
            # 이미지 정보에 번호 추가
            image_dict = image_info.dict()
            image_dict["imageNumber"] = next_image_number
            
            # 이미지 추가
            result = await self.collection.update_one(
                {"_id": person_id},  # ObjectId로 변환된 id 사용
                {
                    "$push": {"images": image_dict},
                    "$set": {"updatedAt": datetime.utcnow()}
                }
            )
            
            if result.modified_count:
                person_data = await self.collection.find_one({"_id": person_id})
                if person_data:
                    person_data["id"] = str(person_data.pop("_id"))
                    return Person(**person_data)
                
            return None
        except Exception as e:
            print(f"Error in add_person_image: {str(e)}")  # 디버깅 추가
            return None

    async def delete_person_by_name(self, name: str) -> bool:
        """이름으로 사용자를 삭제합니다."""
        await self.connect()
        print(f"Attempting to delete person with name: {name}")  # 디버깅용 로그
        result = await self.collection.update_one(
            {"name": name, "isDeleted": False},
            {
                "$set": {
                    "deletedAt": datetime.now(),
                    "isDeleted": True,  # true -> True
                    "updatedAt": datetime.now()
                }
            }
        )
        print(f"Delete result: {result.modified_count}")  # 디버깅용 로그
        return result.modified_count > 0

    async def get_person_by_name(self, name: str) -> Optional[Person]:
        """이름으로 사용자를 조회합니다."""
        await self.connect()
        print(f"Searching for person with name: {name}")  # 디버깅용 로그
        query = {"name": name, "isDeleted": False}  # MongoDB에서는 False로 충분합니다
        print(f"Query: {query}")  # 디버깅용 로그
        person_data = await self.collection.find_one(query)
        print(f"Found person data: {person_data}")  # 디버깅용 로그
        if person_data:
            person_data["id"] = str(person_data.pop("_id"))
            return Person(**person_data)
        return None

    async def get_person_images(self, identifier: str, is_name: bool = False) -> Optional[List[ImageInfo]]:
        """사용자의 모든 이미지를 조회합니다."""
        await self.connect()
        
        # name으로 검색할 경우 id로 변환
        person_id = await self.get_person_id_by_name(identifier) if is_name else identifier
        if not person_id:
            return None
            
        person = await self.collection.find_one({"_id": person_id})
        if person and "images" in person:
            return [ImageInfo(**img) for img in person["images"]]
        return []

    async def delete_person_image(self, identifier: str, image_number: int, is_name: bool = False) -> bool:
        """사용자의 특정 이미지를 삭제합니다."""
        await self.connect()
        
        person_id = await self.get_person_id_by_name(identifier) if is_name else identifier
        if not person_id:
            return False
            
        result = await self.collection.update_one(
            {"_id": person_id},
            {
                "$pull": {"images": {"imageNumber": image_number}},
                "$set": {"updatedAt": datetime.utcnow()}
            }
        )
        return result.modified_count > 0