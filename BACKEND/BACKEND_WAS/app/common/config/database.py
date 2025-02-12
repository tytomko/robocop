import os
from pydantic_settings import BaseSettings
from pydantic import Field

class DatabaseSettings(BaseSettings):
    """데이터베이스 설정"""
    # MongoDB 설정
    MONGODB_URL: str = os.getenv("MONGODB_URL")
    MONGODB_DB_NAME: str = os.getenv("DATABASE_NAME")
    

    # PostgreSQL 설정 (필요한 경우)
    POSTGRES_URL: str = Field(
        default="postgresql://postgres:postgres@localhost:5432/robocop"
    )
    
    model_config = {
        "env_file": ".env",
        "extra": "allow"
    }