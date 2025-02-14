from pydantic import BaseModel
from typing import List, Literal, Optional, Dict, Any
from datetime import datetime
from enum import Enum

class Position(BaseModel):
    x: float = 0
    y: float = 0
    z: float = 0
    orientation: float = 0

class BatteryStatus(BaseModel):
    level: float = 100.0
    isCharging: bool = False
    lastCharged: Optional[datetime] = None

class RobotImage(BaseModel):
    imageId: str
    url: str
    createdAt: datetime

class RobotStatus(str, Enum):
    WAITING = "waiting"
    HOMING = "homing"
    NAVIGATING = "navigating"
    EMERGENCYSTOPPED = "emergencyStopped"
    PATROLLING = "patrolling"
    CHARGING = "charging"
    ERROR = "error"

class NetworkStatus(str):
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    DISCONNECTING = "disconnecting"
    networkHealth: float = 100.0

class Waypoint(BaseModel):
    x: float
    y: float

class Robot(BaseModel):
    seq: int
    # robotID: str
    manufactureName: str
    nickname: str
    ipAddress: str
    networkStatus: str = NetworkStatus.CONNECTED
    status: str = RobotStatus.WAITING
    networkHealth: float = 100.0
    position: Position
    battery: BatteryStatus
    cpuTemp: float = 0.0
    image: Optional[RobotImage] = None
    waypoints: List[Waypoint] = []
    startAt: datetime
    IsActive: bool = True
    IsDeleted: bool = False
    DeletedAt: Optional[datetime] = None
    lastActive: datetime
    createdAt: datetime
    updatedAt: Optional[datetime] = None

    class Config:
        json_encoders = {datetime: lambda v: v.isoformat()}

class StatusUpdate(BaseModel):
    status: Literal["manual", "auto", "emergency_stop"]

class StatusResponse(BaseModel):
    robotId: str
    status: str
    timestamp: datetime
    location: Position

class NicknameResponse(BaseModel):
    seq: int
    robotID: str
    nickname: str
    status: str
    timestamp: datetime

class LogEntry(BaseModel):
    x: int
    y: int
    timestamp: datetime

class RouteLog(BaseModel):
    routeId: str
    startTime: datetime
    endTime: datetime
    waypoints: List[LogEntry]
    status: str

class DetectionInfo(BaseModel):
    detectionId: str
    timestamp: datetime
    location: Position
    personInfo: Dict[str, Any]
    imageUrl: str
    status: str

class LogResponse(BaseModel):
    seq: str
    routes: Optional[List[RouteLog]] = None
    detections: Optional[List[DetectionInfo]] = None
    pagination: Dict[str, Any]

class RobotCreate(BaseModel):
    name: str
    ipAddress: str

# ROS2 메시지 타입을 위한 모델
class ROS2RobotStatus(BaseModel):
    """robot_custom_interfaces/msg/Status 메시지 타입"""
    robot_id: str
    status: str
    battery_level: float
    battery_charging: bool
    position_x: float
    position_y: float
    position_z: float
    orientation: float
    cpu_temp: float
    error_code: Optional[int] = None
    error_message: Optional[str] = None
    timestamp: datetime


