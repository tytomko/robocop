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
        user_dict = user_data.dict()
        user_dict["hashed_password"] = hashed_password
        del user_dict["password"]  # 원본 비밀번호 제거
        
        # 사용자 생성
        return await self.repository.create_user(user_dict)

    async def create_admin_user(self):
        admin = await self.repository.find_user_by_username("admin")
        if not admin:
            admin_user = {
                "username": "admin",
                "hashedPassword": security_service.get_password_hash("admin"),
                "role": "admin",
                "isActive": True,
                "createdAt": datetime.now()
            }
            await self.repository.create_user(admin_user)

    async def authenticate_user(self, username: str, password: str):
        user = await self.repository.find_user_by_username(username)
        if not user or not security_service.verify_password(password, user["hashedPassword"]):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect username or password",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return user

    async def create_tokens(self, username: str):
        access_token = security_service.create_access_token(data={"sub": username})
        refresh_token = security_service.create_refresh_token(data={"sub": username})
        await self.repository.update_user_refresh_token(username, refresh_token)
        return {
            "accessToken": access_token,
            "refreshToken": refresh_token,
            "tokenType": "bearer"
        }

    async def refresh_tokens(self, current_token: str):
        try:
            payload = jwt.decode(current_token, security_service.SECRET_KEY, algorithms=[security_service.ALGORITHM])
            username = payload.get("sub")
            if username is None:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Invalid refresh token"
                )

            user = await self.repository.find_user_by_username(username)
            if not user or user.get("refreshToken") != current_token:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Invalid refresh token"
                )

            return await self.create_tokens(username)

        except JWTError:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid refresh token"
            )

    async def change_password(self, username: str, current_password: str, new_password: str, confirm_password: str):
        user = await self.repository.find_user_by_username(username)
        
        if not security_service.verify_password(current_password, user["hashedPassword"]):
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
            payload = jwt.decode(token, security_service.SECRET_KEY, algorithms=[security_service.ALGORITHM])
            username: str = payload.get("sub")
            if username is None:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Could not validate credentials"
                )
            token_data = TokenData(username=username)
        except JWTError:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )
        
        user = await self.repository.find_user_by_username(token_data.username)
        if user is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )
        return user