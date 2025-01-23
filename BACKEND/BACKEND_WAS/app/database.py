from motor.motor_asyncio import AsyncIOMotorClient
from beanie import init_beanie
from .models.robot import Robot
from .models.person import Person
from .models.user import User
from .models.schedule import Schedule
from .config import get_settings
import logging
import certifi

settings = get_settings()

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
        # 기존 컬렉션 삭제 (초기화)
        collections = await db.list_collection_names()
        for collection in collections:
            if collection != "counters":  # counters 컬렉션은 유지
                await db.drop_collection(collection)
        
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
                {"_id": "robot_id", "seq": 0},
                {"_id": "schedule_id", "seq": 0},
                {"_id": "person_id", "seq": 0}
            ])
        else:
            # 시퀀스 초기화
            await db.counters.update_many(
                {"_id": {"$in": ["robot_id", "schedule_id", "person_id"]}},
                {"$set": {"seq": 0}}
            )

        logging.info("Database initialized successfully")
        return True
    except Exception as e:
        logging.error(f"Error initializing database: {str(e)}")
        return False

async def test_connection():
    try:
        await client.admin.command('ping')
        print("MongoDB 연결 성공!")
        # 데이터베이스 초기화
        if await init_db():
            print("데이터베이스 구조 초기화 성공!")
        return True
    except Exception as e:
        print(f"MongoDB 연결 실패: {str(e)}")
        return False 