from pydantic_settings import BaseSettings
from functools import lru_cache

class Settings(BaseSettings):
    # MongoDB 설정
    MONGO_URL: str = "mongodb+srv://maybecold:OBK7Z5K3UYqTiEUp@cluster0.5idlh.mongodb.net/?retryWrites=true&w=majority&tls=true&tlsAllowInvalidCertificates=true"
    DATABASE_NAME: str = "robocop_db"
    
    # JWT 설정
    SECRET_KEY: str = "your-secret-key"  # 실제 운영환경에서는 안전한 키로 변경
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30
    
    # 서버 설정
    HOST: str = "0.0.0.0"
    PORT: int = 8080
    
    class Config:
        env_file = ".env"

@lru_cache()
def get_settings():
    return Settings() 