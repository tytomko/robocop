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
    is_charging: bool = False

class RobotStatus(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    CHARGING = "charging"
    ERROR = "error"
    EMERGENCY = "emergency"

class RobotImage(BaseModel):
    image_id: str
    url: str
    created_at: datetime

class Robot(Document):
    robot_id: int
    name: str
    ip_address: str
    status: RobotStatus = RobotStatus.IDLE
    position: Position
    battery: BatteryStatus
    image: Optional[RobotImage] = None
    last_active: datetime
    created_at: datetime

    class Settings:
        name = "robots"
        indexes = [
            "robot_id",
            "name",
            ("position.x", "position.y")
        ]

# Form 데이터로 받을 생성 모델
class RobotCreate(BaseModel):
    name: str
    ip_address: str 