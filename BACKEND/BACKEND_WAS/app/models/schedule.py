from enum import Enum
from pydantic import BaseModel, Field
from datetime import time
from typing import List, Optional

class DayOfWeek(str, Enum):
    MONDAY = "MONDAY"
    TUESDAY = "TUESDAY"
    WEDNESDAY = "WEDNESDAY"
    THURSDAY = "THURSDAY"
    FRIDAY = "FRIDAY"
    SATURDAY = "SATURDAY"
    SUNDAY = "SUNDAY"

class Schedule(BaseModel):
    schedule_id: int = Field(...)  # 자동 증가하는 정수
    days: List[DayOfWeek]  # 실행할 요일 목록
    start_time: time  # 작업 시작 시간 (HH:MM)
    end_time: time  # 작업 종료 시간 (HH:MM)
    is_active: bool = True  # 스케줄 활성화 여부
    description: Optional[str] = None  # 스케줄 설명

    class Config:
        json_schema_extra = {
            "example": {
                "schedule_id": 1,
                "days": ["MONDAY", "WEDNESDAY", "FRIDAY"],
                "start_time": "09:00",
                "end_time": "18:00",
                "is_active": True,
                "description": "주간 순찰 스케줄"
            }
        }

class ScheduleCreate(BaseModel):
    days: List[DayOfWeek]
    start_time: time
    end_time: time
    description: Optional[str] = None 