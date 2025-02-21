from datetime import datetime
from typing import Optional
from pydantic import BaseModel

class Token(BaseModel):
    """토큰 모델"""
    access_token: str
    refresh_token: str
    token_type: str = "bearer"

class TokenData(BaseModel):
    """토큰 데이터 모델"""
    username: Optional[str] = None
    exp: Optional[datetime] = None

class TokenConfig(BaseModel):
    """토큰 설정 모델"""
    secret_key: str
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30
    refresh_token_expire_minutes: int = 1440  # 24시간