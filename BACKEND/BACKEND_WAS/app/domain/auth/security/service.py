from datetime import datetime, timedelta
from typing import Optional
from jose import JWTError, jwt
from passlib.context import CryptContext
from .models import TokenConfig, TokenData
from ....common.config.manager import get_settings

settings = get_settings()

class SecurityService:
    """보안 서비스"""
    def __init__(self):
        self.config = TokenConfig(
            secret_key=settings.security.SECRET_KEY,
            algorithm=settings.security.ALGORITHM,
            access_token_expire_minutes=settings.security.ACCESS_TOKEN_EXPIRE_MINUTES,
            refresh_token_expire_minutes=settings.security.REFRESH_TOKEN_EXPIRE_MINUTES
        )
        self.pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

    def create_access_token(self, data: dict) -> str:
        """액세스 토큰을 생성합니다."""
        to_encode = data.copy()
        expire = datetime.utcnow() + timedelta(minutes=self.config.access_token_expire_minutes)
        to_encode.update({"exp": expire})
        return jwt.encode(to_encode, self.config.secret_key, algorithm=self.config.algorithm)

    def create_refresh_token(self, data: dict) -> str:
        """리프레시 토큰을 생성합니다."""
        to_encode = data.copy()
        expire = datetime.utcnow() + timedelta(minutes=self.config.refresh_token_expire_minutes)
        to_encode.update({"exp": expire})
        return jwt.encode(to_encode, self.config.secret_key, algorithm=self.config.algorithm)

    def verify_token(self, token: str) -> Optional[TokenData]:
        """토큰을 검증하고 데이터를 반환합니다."""
        try:
            payload = jwt.decode(token, self.config.secret_key, algorithms=[self.config.algorithm])
            username: str = payload.get("sub")
            exp: datetime = datetime.fromtimestamp(payload.get("exp"))
            if username is None:
                return None
            return TokenData(username=username, exp=exp)
        except JWTError:
            return None

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """비밀번호를 검증합니다."""
        return self.pwd_context.verify(plain_password, hashed_password)

    def get_password_hash(self, password: str) -> str:
        """비밀번호를 해시화합니다."""
        return self.pwd_context.hash(password)
        
    def hash_password(self, password: str) -> str:
        """비밀번호를 해시화합니다."""
        return self.get_password_hash(password)