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
    is_active: bool = True
    is_completed: bool = False
    last_executed: Optional[datetime] = None
    next_execution: Optional[datetime] = None

class Schedule(Document):
    schedule_id: int
    robot_id: int  # 로봇 ID 직접 참조
    title: str
    description: Optional[str] = None
    start_time: datetime
    end_time: datetime
    repeat_days: List[int] = []  # 0: 월요일, 6: 일요일
    locations: List[Location]
    status: ScheduleStatus = ScheduleStatus()
    created_at: datetime = datetime.now()
    updated_at: Optional[datetime] = None

    class Settings:
        name = "schedules"
        indexes = [
            "schedule_id",
            "robot_id",
            "start_time",
            "end_time",
            "status.is_active",
            "status.is_completed"
        ]

# API 요청용 모델
class ScheduleCreate(BaseModel):
    title: str
    description: Optional[str] = None
    start_time: datetime
    end_time: datetime
    repeat_days: List[int] = []
    locations: List[Location]

class ScheduleUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    repeat_days: Optional[List[int]] = None
    locations: Optional[List[Location]] = None
    is_active: Optional[bool] = None 