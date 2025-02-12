from functools import lru_cache
from .base import BaseAppSettings
from .database import DatabaseSettings
from .security import SecuritySettings
from .storage import StorageSettings
from typing import List

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
        ROS_BRIDGE_HOST: str = "172.30.1.78"
        ROS_BRIDGE_PORT: int = 9090

    base: Base
    security: Security
    database: Database
    jwt: JWT
    storage: Storage
    ros: ROS

    def __init__(self):
        self.base = BaseAppSettings()
        self.database = DatabaseSettings()
        self.security = SecuritySettings()
        self.storage = StorageSettings()

@lru_cache()
def get_settings() -> Settings:
    """설정 인스턴스를 반환합니다."""
    return Settings()