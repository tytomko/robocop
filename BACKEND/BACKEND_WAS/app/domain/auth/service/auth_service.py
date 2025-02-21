from fastapi import HTTPException, status
from datetime import datetime, timedelta
from jose import JWTError, jwt
from passlib.context import CryptContext
from typing import Optional
from ..security.service import SecurityService
from ..models.auth_models import User, TokenData, UserCreate, Token
from ..repository.auth_repository import AuthRepository
from ....common.config.manager import get_settings
import logging

logger = logging.getLogger(__name__)

security_service = SecurityService()
settings = get_settings()
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

class AuthService:
    def __init__(self):
        self.repository = AuthRepository()

    def verify_password(self, plain_password: str, hashedPassword: str) -> bool:
        """비밀번호를 검증합니다."""
        return pwd_context.verify(plain_password, hashedPassword)

    def get_password_hash(self, password: str) -> str:
        """비밀번호를 해시화합니다."""
        return pwd_context.hash(password)

    async def create_user(self, user_data: User) -> User:
        try:
            if not user_data.password:
                raise HTTPException(status_code=400, detail="비밀번호는 필수입니다")
            
            # 비밀번호 해싱
<<<<<<< HEAD
            user_data.hashedPassword = security_service.hash_password(user_data.password)
=======
            user_data.hashedPassword = self.security_service.hash_password(user_data.password)
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
            user_data.password = None  # 평문 비밀번호 제거
            
            return await self.repository.create_user(user_data.dict(exclude_none=True))
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

    async def create_admin_user(self):
        admin = await self.repository.find_user_by_username("admin")
        if not admin:
            admin_user = User(
                username="admin",
                password="admin1234",
                role="admin"
            )
            return await self.create_user(admin_user)

    async def authenticate_user(self, username: str, password: str):
        user = await self.repository.find_user_by_username(username)
<<<<<<< HEAD
        if not user or not self.verify_password(password, user.hashedPassword):
=======
        if not user or not security_service.verify_password(password, user.hashedPassword):
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect username or password",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        # 토큰 생성 및 저장
        tokens = await self.create_tokens(username=user.username)
        await self.repository.update_user_refresh_token(user.username, tokens.refreshToken)
        
        user.tokens = tokens
        return user

    async def create_tokens(self, username: str) -> Token:
        """액세스 토큰과 리프레시 토큰을 생성합니다."""
        access_token = security_service.create_access_token({"sub": username})
        refresh_token = security_service.create_refresh_token({"sub": username})
        
        return Token(
            access_token=access_token,
            refresh_token=refresh_token,
            token_type="bearer"
        )

    async def refresh_tokens(self, refresh_token: str) -> Token:
        """리프레시 토큰을 사용하여 새로운 토큰 쌍을 생성합니다."""
        token_data = security_service.verify_token(refresh_token)
        if not token_data:

            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid refresh token"
            )
        
        return await self.create_tokens(token_data.username)

    async def change_password(self, username: str, current_password: str, new_password: str, confirm_password: str):
        user = await self.repository.find_user_by_username(username)
        
        if not security_service.verify_password(current_password, user.hashedPassword):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="현재 비밀번호가 일치하지 않습니다."
            )
        
        if new_password != confirm_password:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="새 비밀번호가 일치하지 않습니다."
            )
        
        hashed_password = security_service.get_password_hash(new_password)
        await self.repository.update_password(username, hashed_password)
        
        return {
            "success": True,
            "message": "비밀번호가 성공적으로 변경되었습니다."
        }

    async def logout(self, username: str):
        """사용자를 로그아웃하고 리프레시 토큰을 무효화합니다."""
        logger.info(f"리프레시 토큰 무효화 시도: 사용자 {username}")
        await self.repository.invalidate_refresh_token(username)
        logger.info(f"리프레시 토큰 무효화 성공: 사용자 {username}")
        return {
            "success": True,
            "message": "로그아웃 되었습니다."
        }

    async def verify_token(self, token: str) -> User:
        try:
            payload = jwt.decode(
                token,
                security_service.config.secret_key,
                algorithms=[security_service.config.algorithm]
            )
            username = payload.get("sub")
            if username is None:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Could not validate credentials"
                )
            
            user_data = await self.repository.find_user_by_username(username)
            if user_data is None:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="User not found"
                )
            
            # MongoDB 데이터를 User 모델에 맞게 변환
            return User(
<<<<<<< HEAD
                id=user_data.id,  # _id 대신 id 사용
                username=user_data.username,
                hashedPassword=user_data.hashedPassword,
                role=user_data.role,
                isActive=user_data.isActive,
                isDefaultPassword=user_data.isDefaultPassword,
                createdAt=user_data.createdAt
=======
                id=str(user_data._id),  # ObjectId를 문자열로 변환
                username=user_data.username,
                hashed_password=user_data.hashedPassword,  # 이미 alias로 처리됨
                role=user_data.role,
                is_active=user_data.isActive,  # 이미 alias로 처리됨
                is_default_password=user_data.isDefaultPassword,  # 이미 alias로 처리됨
                created_at=user_data.createdAt  # 이미 alias로 처리됨
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
            )
        except JWTError:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )