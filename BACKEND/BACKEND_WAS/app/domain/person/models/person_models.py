from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field

class ImageInfo(BaseModel):
    """이미지 정보 모델"""
    imageId: str = Field(..., description="이미지 파일명")  # person_1_1.jpg 형식
    url: str = Field(..., description="이미지 URL")  # 전체 경로
    uploadedAt: datetime = Field(default_factory=datetime.utcnow, description="업로드 시간")
    imageNumber: int = Field(..., description="동일 인물의 n번째 이미지")

    class Config:
        populate_by_name = True

class PersonBase(BaseModel):
    """직원 기본 모델"""
    name: str = Field(..., description="직원 이름")
    department: Optional[str] = Field(None, description="부서")
    position: Optional[str] = Field(None, description="직급")
    phone: Optional[str] = Field(None, description="연락처")

class PersonCreate(PersonBase):
    """직원 생성 모델"""
    pass

class PersonUpdate(BaseModel):
    """직원 정보 업데이트 모델"""
    name: Optional[str] = None
    department: Optional[str] = None
    position: Optional[str] = None
    phone: Optional[str] = None

class Person(PersonBase):
    """직원 모델"""
    id: str = Field(..., description="직원 ID")
    seq: int = Field(..., description="직원 시퀀스 번호")
    name: str = Field(..., description="직원 이름", unique=True)
    images: List[ImageInfo] = Field(default_factory=list, description="직원 얼굴 이미지 목록")
    createdAt: datetime = Field(default_factory=datetime.utcnow, alias="created_at", description="등록 시간")
    updatedAt: Optional[datetime] = Field(None, description="수정 시간")
    deletedAt: Optional[datetime] = Field(None, description="삭제 시간")
    isDeleted: bool = Field(default=False, description="삭제 여부")

    class Config:
        from_attributes = True
        populate_by_name = True