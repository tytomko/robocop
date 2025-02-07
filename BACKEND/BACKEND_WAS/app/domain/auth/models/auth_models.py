from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

class UserBase(BaseModel):
    """사용자 기본 모델"""
    username: str = Field(..., description="사용자 이름")
    role: str = Field(default="user", description="사용자 역할")

class UserCreate(UserBase):
    """사용자 생성 모델"""
    password: str = Field(..., description="비밀번호")

class UserLogin(BaseModel):
    """사용자 로그인 모델"""
    username: str = Field(..., description="사용자 이름")
    password: str = Field(..., description="비밀번호")

class User(UserBase):
    """사용자 모델"""
    id: str = Field(..., description="사용자 ID")
    hashedPassword: str = Field(..., alias="hashed_password", description="해시된 비밀번호")
    isActive: bool = Field(default=True, alias="is_active", description="활성화 여부")
    isDefaultPassword: bool = Field(default=True, alias="is_default_password", description="기본 비밀번호 사용 여부")
    createdAt: datetime = Field(default_factory=datetime.utcnow, alias="created_at", description="생성일")
    updatedAt: Optional[datetime] = Field(None, alias="updated_at", description="수정일")

    class Config:
        populate_by_name = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

class Token(BaseModel):
    """토큰 모델"""
    accessToken: str
    refreshToken: str
    tokenType: str = "bearer"

class TokenData(BaseModel):
    username: Optional[str] = None

class PasswordChange(BaseModel):
    """비밀번호 변경 모델"""
    currentPassword: str = Field(..., description="현재 비밀번호")
    newPassword: str = Field(..., description="새 비밀번호")
    confirmPassword: str = Field(..., description="새 비밀번호 확인")