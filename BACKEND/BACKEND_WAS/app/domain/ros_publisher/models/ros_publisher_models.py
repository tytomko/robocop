from pydantic import BaseModel
from typing import List

class PublishRequest(BaseModel):
    message: str

# class DirectionRequest(BaseModel):
#     direction: Direction

class Position(BaseModel):
    x: float
    y: float
    theta: float = 0.0

class NavigateRequest(BaseModel):
    goal: Position

class PatrolRequest(BaseModel):
    goals: List[Position]

class HomingServiceRequest(BaseModel):
    robot_id: str = "robot_1"  # 기본값 설정
