from functools import lru_cache
from pydantic_settings import BaseSettings
from pydantic import Field

class DatabaseSettings(BaseSettings):
    """데이터베이스 설정"""
    MONGODB_URL: str = "mongodb://localhost:27017"
    MONGODB_DB_NAME: str = "robot_db"
    POSTGRES_URL: str = "postgresql://user:password@localhost:5432/robot_db"
    
    class Config:
        env_file = ".env"
        extra = "allow"

class SecuritySettings(BaseSettings):
    """보안 설정"""
    SECRET_KEY: str = Field(default="your-secret-key")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 180
    REFRESH_TOKEN_EXPIRE_MINUTES: int = 360
    
    # CORS 설정
    CORS_ORIGINS: list = ["*"]
    CORS_METHODS: list = ["*"]
    CORS_HEADERS: list = ["*"]
    
    # 비밀번호 정책
    PASSWORD_MIN_LENGTH: int = 8
    PASSWORD_REQUIRE_SPECIAL: bool = True
    PASSWORD_REQUIRE_NUMBERS: bool = True
    PASSWORD_REQUIRE_UPPERCASE: bool = True
    
    class Config:
        env_file = ".env"
        extra = "allow"

class StorageSettings(BaseSettings):
    """스토리지 설정"""
    BASE_STORAGE_PATH: str = "storage"
    VIDEO_STORAGE_PATH: str = "storage/videos"
    FRAME_STORAGE_PATH: str = "storage/video_frames"
    MAX_VIDEO_SIZE: int = 100 * 1024 * 1024  # 100MB
    
    IMAGE_STORAGE_PATH: str = "storage/images"
    MAX_IMAGE_SIZE: int = 5 * 1024 * 1024  # 5MB
    ALLOWED_IMAGE_EXTENSIONS: list = [".jpg", ".jpeg", ".png"]
    
    LOG_STORAGE_PATH: str = "storage/logs"
    MAX_LOG_SIZE: int = 10 * 1024 * 1024  # 10MB
    MAX_LOG_BACKUP_COUNT: int = 5
    
    class Config:
        env_file = ".env"
        extra = "allow"

class Settings(BaseSettings):
    """애플리케이션 설정"""
    # 기본 설정
    APP_NAME: str = "Robot Control System"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = False
    
    # 하위 설정
    database: DatabaseSettings = DatabaseSettings()
    security: SecuritySettings = SecuritySettings()
    storage: StorageSettings = StorageSettings()
    
    class Config:
        env_file = ".env"
        extra = "allow"

@lru_cache()
def get_settings() -> Settings:
    """설정 가져오기 (캐시 적용)"""
    return Settings() 