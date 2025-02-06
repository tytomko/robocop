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
    is_active: bool = Field(default=True, description="활성화 여부")
    is_default_password: bool = Field(default=True, description="기본 비밀번호 사용 여부")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="생성일")
    updated_at: Optional[datetime] = Field(None, description="수정일")

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