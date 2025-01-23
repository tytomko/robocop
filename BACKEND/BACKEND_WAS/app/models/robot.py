from beanie import Document
from typing import Optional, Union
from datetime import datetime
from enum import Enum
from pydantic import BaseModel, Field

class Position(BaseModel):
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

class BatteryStatus(BaseModel):
    level: float = 100.0
    isCharging: bool = False

class RobotStatus(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    CHARGING = "charging"
    ERROR = "error"
    EMERGENCY = "emergency"

class RobotImage(BaseModel):
    imageId: str
    url: str
    createdAt: datetime

class Robot(Document):
    robotId: int
    name: str = Field(..., unique=True)
    ipAddress: str
    status: RobotStatus = RobotStatus.IDLE
    position: Position
    battery: BatteryStatus
    image: Optional[RobotImage] = None
    lastActive: datetime
    createdAt: datetime

    class Settings:
        name = "robots"
        indexes = [
            [("name", 1)],
            "robotId",
            ("position.x", "position.y")
        ]

# API 요청용 모델들
class RobotIdentifier(BaseModel):
    """로봇 식별을 위한 모델 - ID 또는 이름 사용 가능"""
    id: Optional[int] = None
    name: Optional[str] = None

    def get_query(self):
        """쿼리 조건 생성"""
        if self.id is not None:
            return {"robotId": self.id}
        if self.name is not None:
            return {"name": self.name}
        raise ValueError("Either id or name must be provided")

class RobotCreate(BaseModel):
    name: str = Field(..., description="로봇의 고유 이름")
    ipAddress: str

class RobotUpdate(BaseModel):
    name: Optional[str] = None
    ipAddress: Optional[str] = None
    status: Optional[RobotStatus] = None 