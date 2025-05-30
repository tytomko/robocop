from datetime import datetime
from ...auth.models.auth_models import User
from ....infrastructure.database.connection import DatabaseConnection
from motor.motor_asyncio import AsyncIOMotorCollection
import logging

class AuthRepository:
    def __init__(self):
        self.collection = None
        self.logger = logging.getLogger(__name__)

    async def connect(self):
        """데이터베이스에 연결합니다."""
        if self.collection is None:
            db = await DatabaseConnection.get_db()
            self.collection = db.users

    async def create_user(self, user_data: dict) -> User:
        """새로운 사용자를 생성합니다."""
        await self.connect()
        
        # 생성 시간 추가
        user_data["createdAt"] = datetime.utcnow()
        user_data["isDefaultPassword"] = True
        
        # 사용자 생성
        result = await self.collection.insert_one(user_data)
        user_data["id"] = str(result.inserted_id)
        
        return User(**user_data)

    async def find_user_by_username(self, username: str) -> User:
        """사용자 이름으로 사용자를 찾습니다."""
        await self.connect()
        user_data = await self.collection.find_one({"username": username})
        if user_data:
            print("Found user_data:", user_data)
            user_data["id"] = str(user_data.pop("_id"))
            # hashedPassword를 hashed_password로 변환
            if "hashedPassword" in user_data:
                user_data["hashed_password"] = user_data.pop("hashedPassword")
            return User(**user_data)
        return None

    async def update_user_refresh_token(self, username: str, refresh_token: str):
        """사용자의 리프레시 토큰을 업데이트합니다."""
        await self.connect()
        return await self.collection.update_one(
            {"username": username},
            {
                "$set": {
                    "refreshToken": refresh_token,
                    "lastLogin": datetime.now(),
                    "updatedAt": datetime.now()
                }
            }
        )

    async def update_password(self, username: str, hashed_password: str):
        return await self.collection.update_one(
            {"username": username},
            {
                "$set": {
                    "hashedPassword": hashed_password,
                    "isDefaultPassword": False,
                    "updatedAt": datetime.now()
                }
            }
        )

    async def invalidate_refresh_token(self, username: str):
        """사용자의 리프레시 토큰을 무효화합니다."""
        await self.connect()
        return await self.collection.update_one(
            {"username": username},
            {
                "$set": {
                    "refreshToken": None,
                    "lastLogout": datetime.now(),
                    "updatedAt": datetime.now()
                }
            }
        )