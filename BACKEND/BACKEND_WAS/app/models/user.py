from datetime import datetime
from typing import Optional
from beanie import Document
from enum import Enum
from pydantic import BaseModel

class UserRole(str, Enum):
    ADMIN = "admin"
    OPERATOR = "operator"
    VIEWER = "viewer"

class User(Document):
    username: str
    email: str
    hashedPassword: str
    isActive: bool = True
    isAdmin: bool = False
    refreshToken: Optional[str] = None
    createdAt: datetime
    updatedAt: Optional[datetime] = None

    class Settings:
        name = "users"
        indexes = [
            "username",
            "email",
            "refreshToken"
        ]

class UserCreate(BaseModel):
    username: str
    email: str
    password: str

class UserLogin(BaseModel):
    username: str
    password: str

class Token(BaseModel):
    accessToken: str
    tokenType: str
    refreshToken: str

class TokenData(BaseModel):
    username: Optional[str] = None
    exp: Optional[datetime] = None

class UserResponse(BaseModel):
    username: str
    name: str
    role: UserRole
    isActive: bool
    lastLogin: Optional[datetime]
    createdAt: datetime
    updatedAt: Optional[datetime]

class PasswordChange(BaseModel):
    currentPassword: str
    newPassword: str
    confirmPassword: str 