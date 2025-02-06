from pydantic import BaseModel

class PublishRequest(BaseModel):
    message: str

# class DirectionRequest(BaseModel):
#     direction: Direction

class HomingServiceRequest(BaseModel):
    robot_id: str = "robot_1"  # 기본값 설정
