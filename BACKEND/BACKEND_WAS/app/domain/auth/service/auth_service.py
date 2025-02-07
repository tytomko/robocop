from fastapi import HTTPException, status
from datetime import datetime, timedelta
from jose import JWTError, jwt
from passlib.context import CryptContext
from typing import Optional
from ..security.service import SecurityService
from ..models.auth_models import User, TokenData, UserCreate, Token
from ..repository.auth_repository import AuthRepository
from ....common.config.manager import get_settings

security_service = SecurityService()
settings = get_settings()
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

class AuthService:
    def __init__(self):
        self.repository = AuthRepository()

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """비밀번호를 검증합니다."""
        return pwd_context.verify(plain_password, hashed_password)

    def get_password_hash(self, password: str) -> str:
        """비밀번호를 해시화합니다."""
        return pwd_context.hash(password)

    async def create_user(self, user_data: UserCreate) -> User:
        """새로운 사용자를 생성합니다."""
        # 비밀번호 해시화
        hashed_password = self.get_password_hash(user_data.password)
        
        # 사용자 데이터 준비
        user_dict = {
            "username": user_data.username,
            "hashedPassword": hashed_password,
            "role": user_data.role,
            "isActive": True,
            "isDefaultPassword": True,
            "createdAt": datetime.now()
        }
        del user_dict["password"]  # 원본 비밀번호 제거
        
        # 사용자 생성
        return await self.repository.create_user(user_dict)

    async def create_admin_user(self):
        admin = await self.repository.find_user_by_username("admin")
        if not admin:
            admin_user = {
                "username": "admin",
                "hashedPassword": security_service.get_password_hash("admin1234"),
                "role": "admin",
                "isActive": True,
                "isDefaultPassword": True,
                "createdAt": datetime.now()
            }
            await self.repository.create_user(admin_user)

    async def authenticate_user(self, username: str, password: str):
        user = await self.repository.find_user_by_username(username)
        if not user or not security_service.verify_password(password, user.hashedPassword):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect username or password",
                headers={"WWW-Authenticate": "Bearer"},
            )
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
        await self.repository.invalidate_refresh_token(username)
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
                id=str(user_data._id),  # ObjectId를 문자열로 변환
                username=user_data.username,
                hashed_password=user_data.hashedPassword,  # 이미 alias로 처리됨
                role=user_data.role,
                is_active=user_data.isActive,  # 이미 alias로 처리됨
                is_default_password=user_data.isDefaultPassword,  # 이미 alias로 처리됨
                created_at=user_data.createdAt  # 이미 alias로 처리됨
            )
        except JWTError:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )