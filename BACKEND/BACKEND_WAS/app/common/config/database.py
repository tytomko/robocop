import os
from pydantic_settings import BaseSettings
from pydantic import Field

class DatabaseSettings(BaseSettings):
    """데이터베이스 설정"""
    # MongoDB 설정
<<<<<<< HEAD

    MONGODB_URL: str
    MONGODB_DB_NAME: str
    
    
    REDIS_HOST: str
    REDIS_PORT: int = 6379

    
=======
    MONGODB_URL: str = os.getenv("MONGODB_URL")
    MONGODB_DB_NAME: str = os.getenv("DATABASE_NAME")
    
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23

    # PostgreSQL 설정 (필요한 경우)
    POSTGRES_URL: str = Field(
        default="postgresql://postgres:postgres@localhost:5432/robocop"
    )
    
    model_config = {
        "env_file": ".env",
        "extra": "allow"
    }