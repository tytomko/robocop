from fastapi import APIRouter, BackgroundTasks
from ..models.ros_publisher_models import PublishRequest, NavigateRequest, PatrolRequest
from ..service.ros_publisher_service import *

router = APIRouter()

@router.post("/{robot_id}/publish")
async def publish_once(robot_id: str, request: PublishRequest):
    """ROS 2 토픽에 단일 메시지 발행"""
    await publish_message(robot_id, request.message)
    return {"status": "success", "message": f"Published: {request.message}"}

@router.post("/{robot_id}/publish/up")
async def publish_up(robot_id: str):
    """UP 토픽 발행"""
    command = "UP"
    await publish_message(robot_id, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{robot_id}/publish/down")
async def publish_down(robot_id: str):
    """DOWN 토픽 발행"""
    command = "DOWN"
    await publish_message(robot_id, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{robot_id}/publish/left")
async def publish_left(robot_id: str):
    """LEFT 토픽 발행"""
    command = "LEFT"
    await publish_message(robot_id, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{robot_id}/publish/right")
async def publish_right(robot_id: str):
    """RIGHT 토픽 발행"""
    command = "RIGHT"
    await publish_message(robot_id, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{robot_id}/call-service/homing")
def call_homing_endpoint(robot_id: str):
    """로봇 호밍 서비스 호출"""
    try:
        response = call_homing_service(robot_id)
        return {"status": "success", "message": "Homing service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{robot_id}/call-service/navigate")
def call_navigate_endpoint(robot_id: str, request: NavigateRequest):
    """로봇 네비게이션 서비스 호출"""
    try:
        goal = {"goal": {"x": request.goal.x, "y": request.goal.y, "theta": request.goal.theta}}
        response = call_navigate_service(robot_id, goal)
        return {"status": "success", "message": "Navigation service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{robot_id}/call-service/patrol")
def call_patrol_endpoint(robot_id: str, request: PatrolRequest):
    """로봇 순찰 서비스 호출"""
    try:
        goals = {
            "goals": [
                {"x": goal.x, "y": goal.y, "theta": goal.theta} 
                for goal in request.goals
            ]
        }
        response = call_patrol_service(robot_id, goals)
        return {"status": "success", "message": "Patrol service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{robot_id}/call-service/estop")
def call_estop_endpoint(robot_id: str):
    """로봇 긴급 정지 서비스 호출"""
    try:
        response = call_estop_service(robot_id)
        return {"status": "success", "message": "E-stop service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{robot_id}/call-service/temp-stop")
def call_temp_stop_endpoint(robot_id: str):
    """로봇 일시정지 서비스 호출"""
    try:
        response = call_temp_stop_service(robot_id)
        return {"status": "success", "message": "Temporary stop service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{robot_id}/call-service/resume")
def call_resume_endpoint(robot_id: str):
    """로봇 재개 서비스 호출"""
    try:
        response = call_resume_service(robot_id)
        return {"status": "success", "message": "Resume service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{robot_id}/call-service/waiting")
def call_waiting_endpoint(robot_id: str):
    """로봇 대기 서비스 호출"""
    try:
        response = call_waiting_service(robot_id)
        return {"status": "success", "message": "Waiting service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{robot_id}/call-service/manual")
def call_manual_endpoint(robot_id: str):
    """로봇 매뉴얼 모드 변경 서비스 호출"""
    try:
        response = call_manual_service(robot_id)
        return {"status": "success", "message": "Manual mode service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}
