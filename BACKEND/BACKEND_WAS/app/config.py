from typing import Optional, ClassVar
from pydantic_settings import BaseSettings
from functools import lru_cache
import secrets
import os
from pydantic import Field

class Settings(BaseSettings):
    # MongoDB 설정
    MONGO_URL: str = Field(
        alias="MONGODB_URL",
        default="mongodb+srv://maybecold:OBK7Z5K3UYqTiEUp@cluster0.5idlh.mongodb.net/"
    )
    DATABASE_NAME: str = "robocop_db"
    
    # JWT 설정
    SECRET_KEY: str = os.getenv("SECRET_KEY", secrets.token_hex(32))
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 180  # 3시간
    REFRESH_TOKEN_EXPIRE_MINUTES: int = 360  # 6시간
    
    # 서버 설정
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    
    # 스토리지 설정
    STORAGE_PATH: str = os.getenv("STORAGE_PATH", "storage")
    
    # 기존 설정에 추가
    VIDEO_STORAGE_PATH: str = "storage/videos"
    FRAME_STORAGE_PATH: str = "storage/video_frames"
    
    # 다른 설정들도 타입 어노테이션 필요
    DATABASE_URL: str = "postgresql://postgres:postgres@localhost:5432/robocop"
    
    model_config = {
        "env_file": ".env",
        "extra": "allow"
    }

@lru_cache()
def get_settings():
    return Settings() 