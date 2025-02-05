from pydantic import BaseModel
from typing import List, Literal, Optional, Dict, Any
from datetime import datetime

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

class RobotStatus(str):
    IDLE = "idle"
    MOVING = "moving"
    CHARGING = "charging"
    ERROR = "error"
    EMERGENCY = "emergency"

class Robot(BaseModel):
    robotId: int
    name: str
    ipAddress: str
    status: str = RobotStatus.IDLE
    position: Position
    battery: BatteryStatus
    image: Optional[RobotImage] = None
    lastActive: datetime
    createdAt: datetime
    updatedAt: Optional[datetime] = None

class Waypoint(BaseModel):
    type: Literal["start", "mid", "end"]
    position: Position

class RouteRequest(BaseModel):
    sequence: int
    waypoints: List[Waypoint]

class RouteResponse(BaseModel):
    routeId: str
    robotId: str
    courseSequence: int
    waypoints: Dict[str, Dict[str, int]]
    createdAt: datetime

class MapResponse(BaseModel):
    robotId: str
    mapId: str
    currentLocation: Dict[str, Any]
    mapData: Dict[str, Any]

class StatusUpdate(BaseModel):
    status: Literal["manual", "auto", "emergency_stop"]

class StatusResponse(BaseModel):
    robotId: str
    status: str
    timestamp: datetime
    location: Position

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
    robotId: str
    routes: Optional[List[RouteLog]] = None
    detections: Optional[List[DetectionInfo]] = None
    pagination: Dict[str, Any]

class RobotCreate(BaseModel):
    name: str
    ipAddress: str