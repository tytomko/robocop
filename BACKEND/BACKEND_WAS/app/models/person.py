from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Optional
from enum import Enum

class Department(str, Enum):
    RND = "연구개발부"
    SALES = "영업부"
    HR = "인사부"
    MANAGEMENT = "경영지원부"
    IT = "IT부서"
    SECURITY = "보안부"
    OTHER = "기타"

class Position(str, Enum):
    CEO = "대표이사"
    DIRECTOR = "이사"
    MANAGER = "부장"
    ASSISTANT_MANAGER = "과장"
    SENIOR = "대리"
    STAFF = "사원"
    INTERN = "인턴"
    OTHER = "기타"

class ImageInfo(BaseModel):
    image_id: str = Field(...)  # UUID로 생성될 이미지 ID
    url: str = Field(...)  # 저장된 이미지의 URL
    created_at: datetime = Field(default_factory=datetime.now)

class Person(BaseModel):
    person_id: int = Field(...)  # 자동 증가하는 ID
    name: str = Field(...)  # 이름 (필수)
    department: Optional[Department] = None  # 부서 (선택)
    position: Optional[Position] = None  # 직급 (선택)
    images: List[ImageInfo] = Field(default_factory=list)  # 학습용 이미지들
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: Optional[datetime] = None

    class Config:
        json_schema_extra = {
            "example": {
                "person_id": 1,
                "name": "홍길동",
                "department": "연구개발부",
                "position": "과장",
                "images": [
                    {
                        "image_id": "person_1_abc123",
                        "url": "/storage/persons/person_1_abc123.jpg",
                        "created_at": "2024-01-21T00:00:00"
                    }
                ],
                "created_at": "2024-01-21T00:00:00",
                "updated_at": "2024-01-21T00:00:00"
            }
        }

# Form 데이터로 받을 생성 모델
class PersonCreate(BaseModel):
    name: str = Field(..., description="사람의 이름")
    department: Optional[Department] = Field(None, description="부서 (선택사항)")
    position: Optional[Position] = Field(None, description="직급 (선택사항)")

# 업데이트용 모델
class PersonUpdate(PersonCreate):
    pass 