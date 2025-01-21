from pydantic import BaseModel, Field, validator
from datetime import datetime
from typing import Optional, List, Dict
from enum import Enum
import re

class RobotStatus(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    CHARGING = "charging"
    ERROR = "error"
    OFFLINE = "offline"

class Position(BaseModel):
    x: float = Field(...)
    y: float = Field(...)
    theta: float = Field(...)

class BatteryStatus(BaseModel):
    level: float = Field(..., ge=0, le=100)  # 배터리 잔량 (%)
    is_charging: bool = Field(default=False)  # 충전 중 여부

class RobotImage(BaseModel):
    image_id: str
    url: str
    created_at: datetime = Field(default_factory=datetime.now)

class Robot(BaseModel):
    robot_id: int = Field(...)  # 로봇 고유 ID (자동 증가하는 정수)
    name: str = Field(...)  # 로봇 이름
    ip_address: str = Field(...)  # 로봇 IP 주소
    status: RobotStatus = Field(default=RobotStatus.IDLE)  # 로봇 상태
    position: Position  # 현재 위치
    battery: BatteryStatus  # 배터리 상태
    image: Optional[RobotImage] = None  # 로봇 이미지 (선택사항)
    last_active: datetime = Field(default_factory=datetime.now)  # 마지막 활성 시간
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: Optional[datetime] = None

    @validator('ip_address')
    def validate_ip_address(cls, v):
        pattern = r'^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'
        if not re.match(pattern, v):
            raise ValueError('유효하지 않은 IP 주소 형식입니다. (예: 192.168.1.100)')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "robot_id": 1,
                "name": "Security Bot 1",
                "ip_address": "192.168.1.100",
                "status": "idle",
                "position": {
                    "x": 10.5,
                    "y": 20.3,
                    "theta": 1.57
                },
                "battery": {
                    "level": 85.5,
                    "is_charging": False
                },
                "image": {
                    "image_id": "img_001",
                    "url": "/storage/robots/robot1.jpg",
                    "created_at": "2024-01-20T00:00:00"
                },
                "last_active": "2024-01-20T15:30:00",
                "created_at": "2024-01-20T00:00:00",
                "updated_at": "2024-01-20T15:30:00"
            }
        }

# Form 데이터로 받을 생성 모델
class RobotCreate(BaseModel):
    name: str
    ip_address: str 