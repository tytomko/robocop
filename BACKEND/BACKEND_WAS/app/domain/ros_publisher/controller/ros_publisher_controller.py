from fastapi import APIRouter, BackgroundTasks
from ..models.ros_publisher_models import PublishRequest, HomingServiceRequest
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
async def publish_up():
    """DOWN 토픽 발행"""
    command = "DOWN"
    await publish_message(command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/publish/left")
async def publish_up():
    """LEFT 토픽 발행"""
    command = "LEFT"
    await publish_message(command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/publish/right")
async def publish_up():
    """RIGHT 토픽 발행"""
    command = "RIGHT"
    await publish_message(command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/start/")
async def start_publishing(background_tasks: BackgroundTasks):
    """ROS 2 토픽을 2초마다 발행하는 태스크 시작"""
    background_tasks.add_task(publish_loop)
    return {"status": "started", "message": "Publishing every 2 seconds"}

@router.post("/stop/")
async def stop_publishing_api():
    """발행 중지"""
    stop_publishing()
    return {"status": "stopped", "message": "Stopped publishing"}

@router.post("/call_service/")
async def call_homing_service(request: HomingServiceRequest):
    """로봇 호밍 서비스 호출"""
    try:
        await call_service()
        return {"status": "success", "message": "Homing service called successfully"}
    except Exception as e:
        return {"status": "error", "message": str(e)}
