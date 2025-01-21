from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional
from enum import Enum

class UserRole(str, Enum):
    ADMIN = "admin"
    OPERATOR = "operator"
    VIEWER = "viewer"

class User(BaseModel):
    user_id: int = Field(...)
    username: str
    password: str
    role: UserRole
    is_active: bool = True
    is_default_password: bool = True  # 기본 비밀번호 사용 여부
    last_login: Optional[datetime] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: Optional[datetime] = None

    class Config:
        json_schema_extra = {
            "example": {
                "user_id": 1,
                "username": "admin",
                "password": "hashed_password",
                "role": "ADMIN",
                "is_active": True,
                "is_default_password": True,
                "last_login": "2024-01-20T00:00:00",
                "created_at": "2024-01-20T00:00:00",
                "updated_at": "2024-01-20T12:00:00"
            }
        }

class UserCreate(BaseModel):
    email: str
    password: str = Field(..., min_length=8)  # 최소 8자리 이상
    name: str
    role: Optional[UserRole] = UserRole.VIEWER

class UserUpdate(BaseModel):
    name: Optional[str] = None
    role: Optional[UserRole] = None
    is_active: Optional[bool] = None

class UserLogin(BaseModel):
    username: str
    password: str

class Token(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str

class TokenData(BaseModel):
    username: Optional[str] = None

class UserResponse(BaseModel):
    username: str
    name: str
    role: UserRole
    is_active: bool
    last_login: Optional[datetime]
    created_at: datetime
    updated_at: Optional[datetime]

class PasswordChange(BaseModel):
    current_password: str
    new_password: str
    confirm_password: str 