from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime
from pydantic import validator

class UserBase(BaseModel):
    """사용자 기본 모델"""
    username: str = Field(..., description="사용자 이름")
    role: str = Field(default="user", description="사용자 역할")

class UserCreate(BaseModel):
    """사용자 생성 모델"""
    username: str
    password: str
    role: str = "user"
<<<<<<< HEAD
    isActive: bool = True
    isDefaultPassword: bool = False
=======
    is_active: bool = True
    is_default_password: bool = False
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23

class UserLogin(BaseModel):
    """사용자 로그인 모델"""
    username: str = Field(..., description="사용자 이름")
    password: str = Field(..., description="비밀번호")

<<<<<<< HEAD
=======
class User(UserBase):
    """사용자 모델"""
    id: Optional[str] = None  # 생성 시에는 None
    password: Optional[str] = None  # 생성 시에만 사용
    hashedPassword: Optional[str] = None  # DB 저장용
    isActive: bool = True
    isDefaultPassword: bool = Field(default=False)  # admin 계정은 True로 설정
    createdAt: datetime = Field(default_factory=datetime.utcnow)
    updatedAt: Optional[datetime] = None

    class Config:
        populate_by_name = True
        json_encoders = {datetime: lambda v: v.isoformat()}

    @validator('isDefaultPassword', pre=True)
    def set_default_password(cls, v, values):
        # admin 계정인 경우 default_password = True
        return True if values.get('username') == 'admin' else v

>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
class Token(BaseModel):
    """토큰 모델"""
    accessToken: str
    refreshToken: str
    tokenType: str = "bearer"

class User(UserBase):
    """사용자 모델"""
    id: Optional[str] = None  # 생성 시에는 None
    password: Optional[str] = None  # 생성 시에만 사용
    hashedPassword: Optional[str] = None  # DB 저장용
    isActive: bool = True
    isDefaultPassword: bool = Field(default=False)  # admin 계정은 True로 설정
    createdAt: datetime = Field(default_factory=datetime.utcnow)
    updatedAt: Optional[datetime] = None
    tokens: Optional[Token] = None  # 토큰 정보 포함

    class Config:
        populate_by_name = True
        json_encoders = {datetime: lambda v: v.isoformat()}

    @validator('isDefaultPassword', pre=True)
    def set_default_password(cls, v, values):
        # admin 계정인 경우 default_password = True
        return True if values.get('username') == 'admin' else v

class TokenData(BaseModel):
    username: Optional[str] = None

class PasswordChange(BaseModel):
    """비밀번호 변경 모델"""
    currentPassword: str = Field(..., description="현재 비밀번호")
    newPassword: str = Field(..., description="새 비밀번호")
    confirmPassword: str = Field(..., description="새 비밀번호 확인")