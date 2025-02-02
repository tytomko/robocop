from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field

class ImageInfo(BaseModel):
    """이미지 정보 모델"""
    image_id: str = Field(..., description="이미지 ID")
    file_path: str = Field(..., description="이미지 파일 경로")
    file_name: str = Field(..., description="이미지 파일 이름")
    file_size: int = Field(..., description="이미지 파일 크기")
    content_type: str = Field(..., description="이미지 콘텐츠 타입")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="생성 시간")

class PersonBase(BaseModel):
    """사용자 기본 모델"""
    name: str = Field(..., description="사용자 이름")
    email: str = Field(..., description="이메일 주소")
    role: str = Field(default="user", description="사용자 역할")
    is_active: bool = Field(default=True, description="활성화 상태")

class PersonCreate(PersonBase):
    """사용자 생성 모델"""
    password: str = Field(..., min_length=8, description="비밀번호")

class PersonUpdate(BaseModel):
    """사용자 업데이트 모델"""
    name: Optional[str] = None
    email: Optional[str] = None
    password: Optional[str] = None
    role: Optional[str] = None
    is_active: Optional[bool] = None

class Person(PersonBase):
    """사용자 모델"""
    id: str = Field(..., description="사용자 ID")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="생성 시간")
    updated_at: Optional[datetime] = Field(None, description="수정 시간")
    images: List[ImageInfo] = Field(default_factory=list, description="사용자 이미지 목록")

    class Config:
        from_attributes = True 