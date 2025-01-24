from beanie import Document
from typing import Optional, List
from datetime import datetime, time
from pydantic import BaseModel, Field
from enum import Enum
from .robot import Robot

class DayOfWeek(str, Enum):
    MONDAY = "MONDAY"
    TUESDAY = "TUESDAY"
    WEDNESDAY = "WEDNESDAY"
    THURSDAY = "THURSDAY"
    FRIDAY = "FRIDAY"
    SATURDAY = "SATURDAY"
    SUNDAY = "SUNDAY"

class OperatingTime(BaseModel):
    start_time: time  # HH:MM 형식
    end_time: time    # HH:MM 형식
    active_days: List[DayOfWeek] = Field(
        default=[],
        description="운영 요일 (예: ['SATURDAY', 'SUNDAY'])"
    )

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
    robotId: Optional[int] = None  # None인 경우 모든 로봇에 적용
    title: str
    description: Optional[str] = None
    operatingTime: OperatingTime
    locations: List[Location]
    status: ScheduleStatus = ScheduleStatus()
    createdAt: datetime = datetime.now()
    updatedAt: Optional[datetime] = None

    class Settings:
        name = "schedules"
        indexes = [
            "scheduleId",
            "robotId",
            "status.isActive",
            "status.isCompleted"
        ]

# API 요청용 모델
class ScheduleCreate(BaseModel):
    title: str
    description: Optional[str] = None
    start_time: time
    end_time: time
    active_days: List[DayOfWeek] = Field(
        default=[],
        description="운영 요일 (예: ['SATURDAY', 'SUNDAY'])"
    )
    locations: List[Location]
    apply_to_all_robots: bool = False

class ScheduleUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    start_time: Optional[time] = None
    end_time: Optional[time] = None
    active_days: Optional[List[DayOfWeek]] = None
    locations: Optional[List[Location]] = None
    isActive: Optional[bool] = None 