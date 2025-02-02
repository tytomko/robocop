from typing import Optional
from pydantic_settings import BaseSettings
from pydantic import Field
import os
import secrets

class BaseAppSettings(BaseSettings):
    """기본 애플리케이션 설정"""
    # 서버 설정
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    DEBUG: bool = False
    
    # 환경 설정
    ENV: str = Field(default="development")
    
    # 스토리지 설정
    STORAGE_PATH: str = Field(default="storage")
    
    model_config = {
        "env_file": ".env",
        "extra": "allow"
    }