import os
from pydantic_settings import BaseSettings
from pydantic import Field

class DatabaseSettings(BaseSettings):
    """데이터베이스 설정"""
    # MongoDB 설정

    MONGODB_URL: str
    MONGODB_DB_NAME: str
    
    
    REDIS_HOST: str
    REDIS_PORT: int = 6379

    

    # PostgreSQL 설정 (필요한 경우)
    POSTGRES_URL: str = Field(
        default="postgresql://postgres:postgres@localhost:5432/robocop"
    )
    
    model_config = {
        "env_file": ".env",
        "extra": "allow"
    }