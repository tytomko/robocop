import json
import os
from ..models.map_models import PointsData
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PointService:
    @staticmethod
    def get_points():
        try:
            # 현재 파일의 디렉토리 경로를 기준으로 상대 경로 설정
            current_dir = os.path.dirname(os.path.abspath(__file__))
            json_path = os.path.join(current_dir, '..', 'global_map.json')
            
            with open(json_path, 'r') as file:
                data = json.load(file)
                return PointsData(**data)  # 전체 데이터를 PointsData 모델로 변환
        except FileNotFoundError:
            # 파일이 없을 경우 빈 데이터 반환
            logger.error(f"File not found: {json_path}")
            return PointsData(directed=False, multigraph=False, graph={}, nodes=[])
        except Exception as e:
            logger.error(f"Error reading map data: {str(e)}")
            return PointsData(directed=False, multigraph=False, graph={}, nodes=[])