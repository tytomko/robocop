from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional, Dict, Any
from enum import Enum
from .robot import Position

class LogType(str, Enum):
    ROUTE = "route"  # 경로 이동 관련 로그
    DETECTION = "detection"  # 사람 감지 관련 로그
    STATUS = "status"  # 로봇 상태 변경 로그
    ERROR = "error"  # 에러 로그

class RouteLog(BaseModel):
    route_id: str  # 경로 ID
    waypoint_order: int  # 현재 경유지 순서
    position: Position  # 현재 위치

class DetectionLog(BaseModel):
    person_id: Optional[str] = None  # 감지된 사람 ID (등록된 사람인 경우)
    confidence: float = Field(..., ge=0, le=1)  # 인식 신뢰도
    position: Position  # 감지 위치
    image_url: Optional[str] = None  # 감지 이미지 URL

class Log(BaseModel):
    log_id: str = Field(...)  # 로그 고유 ID
    robot_id: str = Field(...)  # 로봇 ID
    type: LogType  # 로그 유형
    data: Dict[str, Any]  # 로그 데이터 (RouteLog 또는 DetectionLog)
    created_at: datetime = Field(default_factory=datetime.now)

    class Config:
        json_schema_extra = {
            "example": {
                "log_id": "log_001",
                "robot_id": "robot_001",
                "type": "detection",
                "data": {
                    "person_id": "person_001",
                    "confidence": 0.95,
                    "position": {
                        "x": 10.5,
                        "y": 20.3,
                        "theta": 1.57
                    },
                    "image_url": "https://example.com/detections/det_001.jpg"
                },
                "created_at": "2024-01-20T15:30:00"
            }
        } 