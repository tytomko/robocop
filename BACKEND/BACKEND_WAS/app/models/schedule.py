from beanie import Document, Link
from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel
from .robot import Robot

class Location(BaseModel):
    x: float
    y: float
    theta: float = 0.0

class ScheduleStatus(BaseModel):
    isActive: bool = True
    isCompleted: bool = False
    lastExecuted: Optional[datetime] = None
    nextExecution: Optional[datetime] = None

class Schedule(Document):
    scheduleId: int
    robotId: int  # 로봇 ID 직접 참조
    title: str
    description: Optional[str] = None
    startTime: datetime
    endTime: datetime
    repeatDays: List[int] = []  # 0: 월요일, 6: 일요일
    locations: List[Location]
    status: ScheduleStatus = ScheduleStatus()
    createdAt: datetime = datetime.now()
    updatedAt: Optional[datetime] = None

    class Settings:
        name = "schedules"
        indexes = [
            "scheduleId",
            "robotId",
            "startTime",
            "endTime",
            "status.isActive",
            "status.isCompleted"
        ]

# API 요청용 모델
class ScheduleCreate(BaseModel):
    title: str
    description: Optional[str] = None
    startTime: datetime
    endTime: datetime
    repeatDays: List[int] = []
    locations: List[Location]

class ScheduleUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    startTime: Optional[datetime] = None
    endTime: Optional[datetime] = None
    repeatDays: Optional[List[int]] = None
    locations: Optional[List[Location]] = None
    isActive: Optional[bool] = None 