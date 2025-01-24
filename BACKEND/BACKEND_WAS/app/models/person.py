from beanie import Document
from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel

class ImageInfo(BaseModel):
    imageId: str
    url: str
    uploadedAt: datetime

class Person(Document):
    personId: int
    name: str
    department: Optional[str] = None
    position: Optional[str] = None
    email: Optional[str] = None
    phone: Optional[str] = None
    images: List[ImageInfo] = []
    createdAt: datetime
    updatedAt: Optional[datetime] = None

    class Settings:
        name = "persons"
        indexes = [
            "personId",
            "name",
            "department",
            "position"
        ]

# Form 데이터로 받을 생성 모델
class PersonCreate(BaseModel):
    name: str
    department: Optional[str] = None
    position: Optional[str] = None
    email: Optional[str] = None
    phone: Optional[str] = None 