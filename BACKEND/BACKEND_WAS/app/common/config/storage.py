from pydantic_settings import BaseSettings
from pydantic import Field
import os

class StorageSettings(BaseSettings):
    """스토리지 설정"""
    # 기본 스토리지 경로
    BASE_STORAGE_PATH: str = Field(default="storage")
    
    # 비디오 관련 설정
    VIDEO_STORAGE_PATH: str = Field(default="storage/videos")
    FRAME_STORAGE_PATH: str = Field(default="storage/video_frames")
    MAX_VIDEO_SIZE: int = Field(default=100 * 1024 * 1024)  # 100MB
    
    # 이미지 관련 설정
    IMAGE_STORAGE_PATH: str = Field(default="storage/images")
    MAX_IMAGE_SIZE: int = Field(default=5 * 1024 * 1024)  # 5MB
    ALLOWED_IMAGE_EXTENSIONS: list = Field(default=[".jpg", ".jpeg", ".png"])
    
    # 로그 관련 설정
    LOG_STORAGE_PATH: str = Field(default="storage/logs")
    MAX_LOG_SIZE: int = Field(default=10 * 1024 * 1024)  # 10MB
    MAX_LOG_BACKUP_COUNT: int = Field(default=5)
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._create_directories()
    
    def _create_directories(self):
        """필요한 디렉토리 생성"""
        directories = [
            self.BASE_STORAGE_PATH,
            self.VIDEO_STORAGE_PATH,
            self.FRAME_STORAGE_PATH,
            self.IMAGE_STORAGE_PATH,
            self.LOG_STORAGE_PATH
        ]
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
    
    model_config = {
        "env_file": ".env",
        "extra": "allow"
    }