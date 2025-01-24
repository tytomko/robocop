from motor.motor_asyncio import AsyncIOMotorClient
from beanie import init_beanie
from .models.robot import Robot
from .models.person import Person
from .models.user import User
from .models.schedule import Schedule
from .config import get_settings
import logging
import certifi
from os import environ

settings = get_settings()

# 로거 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# MongoDB 클라이언트 생성
client = AsyncIOMotorClient(
    settings.MONGO_URL,
    serverSelectionTimeoutMS=5000,
    tlsCAFile=certifi.where()
)

# 데이터베이스 선택
db = client[settings.DATABASE_NAME]

# 컬렉션 정의
robots = db.robots
persons = db.persons
users = db.users
schedules = db.schedules

async def init_db():
    try:
        # # 기존 컬렉션 삭제 (초기화)
        collections = await db.list_collection_names()
        # 컬렉션 삭제 로직 제거 - 기존 데이터 유지
        
        # Beanie 초기화 (자동으로 인덱스 생성)
        await init_beanie(
            database=db,
            document_models=[
                Robot,
                Person,
                User,
                Schedule
            ]
        )

        # 시퀀스 컬렉션이 없으면 생성
        if "counters" not in collections:
            await db.create_collection("counters")
            # 각종 ID 시퀀스 초기화
            await db.counters.insert_many([
                {"_id": "robotId", "seq": 0},
                {"_id": "scheduleId", "seq": 0},
                {"_id": "personId", "seq": 0}
            ])
        else:
            # 시퀀스 초기화
            await db.counters.update_many(
                {"_id": {"$in": ["robotId", "scheduleId", "personId"]}},
                {"$set": {"seq": 0}}
            )

        logger.info("Database initialized successfully")
        return True
    except Exception as e:
        logger.error(f"Error initializing database: {str(e)}")
        return False

async def create_admin_if_not_exists():
    try:
        from .routers.auth import create_admin_user
        # admin 계정 존재 여부 확인
        admin_user = await User.find_one({"username": "admin"})
        
        if not admin_user:
            # auth.py의 create_admin_user 함수 사용
            await create_admin_user()
            logger.info("Admin account created successfully")
        return True
    except Exception as e:
        logger.error(f"Error creating admin account: {str(e)}")
        return False

# test_connection과 test_db_connection 함수를 하나로 통합
async def test_connection():
    try:
        await client.admin.command('ping')
        logger.info("MongoDB connection test successful")
        # 데이터베이스 초기화
        if await init_db():
            logger.info("Database structure initialized successfully")
            # admin 계정 확인 및 생성
            await create_admin_if_not_exists()
        return True
    except Exception as e:
        logger.error(f"MongoDB connection test failed: {e}")
        return False 