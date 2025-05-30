from functools import lru_cache
from .base import BaseAppSettings
from .database import DatabaseSettings
from .security import SecuritySettings
from .storage import StorageSettings
from typing import List
from pydantic_settings import BaseSettings


class Settings:
    """설정 관리자"""
    class Base:
        ENV: str
        DEBUG: bool
        SECRET_KEY: str

    class Security:
        CORS_ORIGINS: List[str]
        CORS_METHODS: List[str]
        CORS_HEADERS: List[str]

    class Database:
        MONGODB_URL: str
        MONGODB_DB_NAME: str

    class JWT:
        SECRET_KEY: str
        ALGORITHM: str
        ACCESS_TOKEN_EXPIRE_MINUTES: int

    class Storage:
        MEDIA_SERVER_URL: str
        UPLOAD_API_URL: str
        VIDEO_STORAGE_PATH: str
        IMAGE_STORAGE_PATH: str
        
    class ROS:
        ROS_BRIDGE_HOST: str = "localhost"
        ROS_BRIDGE_PORT: int = 10000

    class ROS2:
        BRIDGE_URL: str = "ws://localhost:10000"
        RETRY_INTERVAL: int = 5
        MAX_RETRIES: int = 3

    REDIS_HOST: str = "127.0.0.1"
    REDIS_PORT: int = 6379

    base: Base
    security: Security
    database: Database
    jwt: JWT
    storage: Storage
    ros: ROS
    ros2: ROS2

    def __init__(self):
        self.base = BaseAppSettings()
        self.database = DatabaseSettings()
        self.security = SecuritySettings()
        self.storage = StorageSettings()
        self.ros2 = self.ROS2()

@lru_cache()
def get_settings() -> Settings:
    """설정 인스턴스를 반환합니다."""
    return Settings()