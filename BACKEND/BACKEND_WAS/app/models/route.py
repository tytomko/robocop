from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Optional
from .robot import Position

class Waypoint(BaseModel):
    position: Position
    order: int = Field(..., ge=0)  # 경유지 순서
    wait_time: int = Field(default=0, ge=0)  # 대기 시간(초)

class Route(BaseModel):
    route_id: str = Field(...)  # 경로 고유 ID
    robot_id: str = Field(...)  # 로봇 ID
    name: str = Field(...)  # 경로 이름
    waypoints: List[Waypoint]  # 경유지 목록
    is_active: bool = Field(default=True)  # 활성화 여부
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: Optional[datetime] = None

    class Config:
        json_schema_extra = {
            "example": {
                "route_id": "route_001",
                "robot_id": "robot_001",
                "name": "First Floor Patrol",
                "waypoints": [
                    {
                        "position": {
                            "x": 10.5,
                            "y": 20.3,
                            "theta": 1.57
                        },
                        "order": 0,
                        "wait_time": 30
                    },
                    {
                        "position": {
                            "x": 15.7,
                            "y": 25.8,
                            "theta": 3.14
                        },
                        "order": 1,
                        "wait_time": 60
                    }
                ],
                "is_active": True,
                "created_at": "2024-01-20T00:00:00",
                "updated_at": "2024-01-20T12:00:00"
            }
        } 