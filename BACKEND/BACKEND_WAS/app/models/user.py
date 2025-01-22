from beanie import Document
from typing import Optional
from datetime import datetime
from pydantic import BaseModel
from enum import Enum

class UserRole(str, Enum):
    ADMIN = "admin"
    OPERATOR = "operator"
    VIEWER = "viewer"

class User(Document):
    username: str
    email: str
    hashed_password: str
    is_active: bool = True
    is_admin: bool = False
    refresh_token: Optional[str] = None
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Settings:
        name = "users"
        indexes = [
            "username",
            "email",
            "refresh_token"
        ]

class UserCreate(BaseModel):
    username: str
    email: str
    password: str

class UserLogin(BaseModel):
    username: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str
    refresh_token: str

class TokenData(BaseModel):
    username: Optional[str] = None
    exp: Optional[datetime] = None

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