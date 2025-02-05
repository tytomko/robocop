from motor.motor_asyncio import AsyncIOMotorClient
from pymongo.database import Database
from ...common.config.manager import get_settings

settings = get_settings()

class DatabaseConnection:
    client: AsyncIOMotorClient = None
    db: Database = None

    @classmethod
    async def connect(cls):
        """데이터베이스 연결을 초기화합니다."""
        if cls.client is None:
            try:
                cls.client = AsyncIOMotorClient(
                    settings.database.MONGODB_URL,
                    serverSelectionTimeoutMS=5000
                )
                cls.db = cls.client[settings.database.MONGODB_DB_NAME]
                # 연결 테스트
                await cls.db.command("ping")
                print("MongoDB Atlas에 성공적으로 연결되었습니다.")
            except Exception as e:
                print(f"MongoDB 연결 실패: {str(e)}")
                cls.client = None
                cls.db = None
                raise e

    @classmethod
    async def disconnect(cls):
        """데이터베이스 연결을 종료합니다."""
        if cls.client is not None:
            cls.client.close()
            cls.client = None
            cls.db = None

    @classmethod
    async def get_db(cls) -> Database:
        """데이터베이스 인스턴스를 반환합니다."""
        if cls.db is None:
            await cls.connect()
        return cls.db

    @classmethod
    async def init_collections(cls):
        """필요한 컬렉션과 인덱스를 초기화합니다."""
        try:
            db = await cls.get_db()
            
            # 사용자 컬렉션 인덱스
            try:
                await db.users.create_index("username", unique=True)
            except Exception as e:
                if 'already exists' in str(e) or 'IndexKeySpecsConflict' in str(e):
                    print("Username 인덱스가 이미 존재합니다.")
                else:
                    raise e
            
            # 로봇 컬렉션 인덱스
            try:
                await db.robots.create_index("robotId", unique=True)
                await db.robots.create_index("name", unique=True)
            except Exception as e:
                if 'already exists' in str(e) or 'IndexKeySpecsConflict' in str(e):
                    print("Robot 인덱스가 이미 존재합니다.")
                else:
                    raise e
            
            # 비디오 세션 컬렉션 인덱스
            try:
                await db.videos.create_index("sessionId", unique=True)
                await db.videos.create_index([("cameraType", 1), ("createdAt", -1)])
            except Exception as e:
                if 'already exists' in str(e) or 'IndexKeySpecsConflict' in str(e):
                    print("Video 인덱스가 이미 존재합니다.")
                else:
                    raise e

        except Exception as e:
            print(f"인덱스 초기화 중 에러 발생: {str(e)}")
            # 인덱스 에러는 애플리케이션 실행에 치명적이지 않으므로 무시
            pass

    @classmethod
    async def test_connection(cls):
        """데이터베이스 연결을 테스트합니다."""
        try:
            db = await cls.get_db()
            await db.command("ping")
            return True
        except Exception as e:
            print(f"데이터베이스 연결 테스트 실패: {str(e)}")
            return False