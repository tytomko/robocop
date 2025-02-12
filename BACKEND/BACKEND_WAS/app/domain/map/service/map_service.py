import logging
from ..models.map_models import PointsData
from ....infrastructure.database.connection import DatabaseConnection

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PointService:
    @staticmethod
    async def get_points():
        try:
            # 데이터베이스 연결
            db = await DatabaseConnection.get_db()
            if db is None:
                raise Exception("데이터베이스 연결 실패")

            # 맵 데이터 가져오기
            map_data = await db.map.find_one()
            if map_data is None:
                logger.error("맵 데이터를 찾을 수 없습니다.")
                return PointsData(directed=False, multigraph=False, graph={}, nodes=[], links=[])

            # print(map_data)
            return PointsData(**map_data)  # 데이터베이스에서 가져온 데이터를 PointsData 모델로 변환
        except Exception as e:
            logger.error(f"맵 데이터 읽기 오류: {str(e)}")
            return PointsData(directed=False, multigraph=False, graph={}, nodes=[], links=[])