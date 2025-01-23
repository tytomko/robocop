from beanie import Document
from typing import Optional
from datetime import datetime
from enum import Enum
from pydantic import BaseModel

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
    name: str
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
            "robotId",
            "name",
            ("position.x", "position.y")
        ]

# Form 데이터로 받을 생성 모델
class RobotCreate(BaseModel):
    name: str
    ipAddress: str 