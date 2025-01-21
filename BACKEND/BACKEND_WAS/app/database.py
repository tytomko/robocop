from motor.motor_asyncio import AsyncIOMotorClient
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
identification_info = db.identification_info
maps = db.maps
routes = db.routes
users = db.users
cctvs = db.cctvs
schedules = db.schedules

async def init_db():
    try:
        # 시퀀스 컬렉션이 없으면 생성
        collections = await db.list_collection_names()
        if "counters" not in collections:
            await db.create_collection("counters")
            # 로봇 ID 시퀀스 초기화
            await db.counters.insert_one({"_id": "robot_id", "seq": 0})
            # 스케줄 ID 시퀀스 초기화
            await db.counters.insert_one({"_id": "schedule_id", "seq": 0})
            # 사람 ID 시퀀스 초기화
            await db.counters.insert_one({"_id": "person_id", "seq": 0})

        # 기존 컬렉션 생성
        if "robots" not in collections:
            await db.create_collection("robots")
        if "persons" not in collections:
            await db.create_collection("persons")
        if "identification_info" not in collections:
            await db.create_collection("identification_info")
        if "maps" not in collections:
            await db.create_collection("maps")
        if "routes" not in collections:
            await db.create_collection("routes")
        if "users" not in collections:
            await db.create_collection("users")
        if "cctvs" not in collections:
            await db.create_collection("cctvs")
        if "schedules" not in collections:
            await db.create_collection("schedules")

        # 인덱스 생성
        # robots 컬렉션 인덱스
        await robots.create_index("robot_id", unique=True)
        await robots.create_index([("location.latitude", 1), ("location.longitude", 1)])
        
        # persons 컬렉션 인덱스
        await persons.create_index("person_id", unique=True)
        await persons.create_index("name")
        await persons.create_index("department")  # 부서별 검색을 위한 인덱스
        await persons.create_index("position")    # 직급별 검색을 위한 인덱스
        
        # identification_info 컬렉션 인덱스
        await identification_info.create_index("id_info_id", unique=True)
        await identification_info.create_index("person_id")
        await identification_info.create_index("detect_time")
        
        # maps 컬렉션 인덱스
        await maps.create_index("map_id", unique=True)
        await maps.create_index([("location.latitude", 1), ("location.longitude", 1)])
        
        # routes 컬렉션 인덱스
        await routes.create_index("route_id", unique=True)
        
        # users 컬렉션 인덱스
        await users.create_index("username", unique=True)
        await users.create_index("refresh_token")
        
        # cctvs 컬렉션 인덱스
        await cctvs.create_index("cctv_id", unique=True)
        await cctvs.create_index([("time_range.start_time", 1), ("time_range.end_time", 1)])

        # schedules 컬렉션 인덱스
        await schedules.create_index("schedule_id", unique=True)
        await schedules.create_index([("start_time", 1), ("end_time", 1)])
        await schedules.create_index("is_active")

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