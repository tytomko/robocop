from pydantic_settings import BaseSettings
from pydantic import Field
import secrets
import os

class SecuritySettings(BaseSettings):
    """보안 설정"""
    # JWT 설정
    SECRET_KEY: str = Field(default_factory=lambda: os.getenv("SECRET_KEY", secrets.token_hex(32)))
    ALGORITHM: str = Field(default="HS256")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = Field(default=180)  # 3시간
    REFRESH_TOKEN_EXPIRE_MINUTES: int = Field(default=360)  # 6시간
    
    # CORS 설정
    CORS_ORIGINS: list = Field(default=["*"])
    CORS_METHODS: list = Field(default=["*"])
    CORS_HEADERS: list = Field(default=["*"])
    
    # 비밀번호 정책
    MIN_PASSWORD_LENGTH: int = Field(default=8)
    REQUIRE_SPECIAL_CHAR: bool = Field(default=True)
    REQUIRE_NUMBER: bool = Field(default=True)
    REQUIRE_UPPERCASE: bool = Field(default=True)
    
    model_config = {
        "env_file": ".env",
        "extra": "allow"
    }