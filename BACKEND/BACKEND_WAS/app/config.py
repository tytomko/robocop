from pydantic_settings import BaseSettings
from functools import lru_cache
import secrets
import os

class Settings(BaseSettings):
    # MongoDB 설정
    MONGO_URL: str = os.getenv("MONGODB_URL", "mongodb://localhost:27017")
    DATABASE_NAME: str = os.getenv("DATABASE_NAME", "robocop_db")
    
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
    
    class Config:
        env_file = ".env"

@lru_cache()
def get_settings():
    return Settings() 