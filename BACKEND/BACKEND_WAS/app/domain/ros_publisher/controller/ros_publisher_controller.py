from fastapi import APIRouter, BackgroundTasks
from ..models.ros_publisher_models import PublishRequest, NavigateRequest, PatrolRequest
from ..service.ros_publisher_service import *

router = APIRouter()

@router.post("/publish/")
async def publish_once(request: PublishRequest):
    """ROS 2 토픽에 단일 메시지 발행"""
    await publish_message(request.message)
    return {"status": "success", "message": f"Published: {request.message}"}

@router.post("/publish/up")
async def publish_up():
    """UP 토픽 발행"""
    command = "UP"
    await publish_message(command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/publish/down")
async def publish_down():
    """DOWN 토픽 발행"""
    command = "DOWN"
    await publish_message(command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/publish/left")
async def publish_left():
    """LEFT 토픽 발행"""
    command = "LEFT"
    await publish_message(command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/publish/right")
async def publish_right():
    """RIGHT 토픽 발행"""
    command = "RIGHT"
    await publish_message(command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/call_service/homing")    
def call_homing_endpoint():
    """로봇 호밍 서비스 호출"""
    try:
        response = call_homing_service()
        return {"status": "success", "message": "Homing service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/call_service/navigate")
def call_navigate_endpoint(request: NavigateRequest):
    """로봇 네비게이션 서비스 호출"""
    try:
        goal = {"goal": {"x": request.goal.x, "y": request.goal.y, "theta": request.goal.theta}}
        response = call_navigate_service(goal)
        return {"status": "success", "message": "Navigation service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/call_service/patrol")
def call_patrol_endpoint(request: PatrolRequest):
    """로봇 순찰 서비스 호출"""
    try:
        goals = {
            "goals": [
                {"x": goal.x, "y": goal.y, "theta": goal.theta} 
                for goal in request.goals
            ]
        }
        response = call_patrol_service(goals)
        return {"status": "success", "message": "Patrol service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}
