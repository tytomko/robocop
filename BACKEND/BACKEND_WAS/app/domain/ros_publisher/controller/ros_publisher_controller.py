from fastapi import APIRouter, BackgroundTasks
from ..models.ros_publisher_models import PublishRequest, NavigateRequest, PatrolRequest
from ..service.ros_publisher_service import *
from ..service.ros_bridge_connection import RosBridgeConnection
import logging

router = APIRouter()
logger = logging.getLogger(__name__)

# ROS Bridge 연결 인스턴스 생성
ros_bridge = RosBridgeConnection()

@router.post("/{seq}/publish")
async def publish_once(seq: str, request: PublishRequest):
    """ROS 2 토픽에 단일 메시지 발행"""
    await publish_message(seq, request.message)
    return {"status": "success", "message": f"Published: {request.message}"}

@router.post("/{seq}/publish/up")
async def publish_up(seq: str):
    """UP 토픽 발행"""
    command = "UP"
    await publish_message(seq, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{seq}/publish/down")
async def publish_down(seq: str):
    """DOWN 토픽 발행"""
    command = "DOWN"
    await publish_message(seq, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{seq}/publish/left")
async def publish_left(seq: str):
    """LEFT 토픽 발행"""
    command = "LEFT"
    await publish_message(seq, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{seq}/publish/right")
async def publish_right(seq: str):
    """RIGHT 토픽 발행"""
    command = "RIGHT"
    await publish_message(seq, command)
    return {"status": "success", "message": f"Published: {command}"}

@router.post("/{seq}/call-service/homing")
def call_homing_endpoint(seq: str):
    """로봇 호밍 서비스 호출"""
    try:
        response = call_homing_service(seq)
        return {"status": "success", "mode": "homing", "message": "Homing service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/navigate")
def call_navigate_endpoint(seq: str, request: NavigateRequest):
    """로봇 네비게이션 서비스 호출"""
    try:
        goal = {"goal": {"x": request.goal.x, "y": request.goal.y, "theta": request.goal.theta}}
        response = call_navigate_service(seq, goal)
        return {"status": "success", "message": "Navigation service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/patrol")
def call_patrol_endpoint(seq: str, request: PatrolRequest):
    """로봇 순찰 서비스 호출"""
    try:
        goals = {
            "goals": [
                {"x": goal.x, "y": goal.y, "theta": goal.theta} 
                for goal in request.goals
            ]
        }
        response = call_patrol_service(seq, goals)
        return {"status": "success", "mode": "patrol", "message": "Patrol service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/estop")
def call_estop_endpoint(seq: str):
    """로봇 긴급 정지 서비스 호출"""
    try:
        response = call_estop_service(seq)
        return {"status": "success", "mode": "emergency_stop", "message": "E-stop service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/temp-stop")
def call_temp_stop_endpoint(seq: str):
    """로봇 일시정지 서비스 호출"""
    try:
        response = call_temp_stop_service(seq)
        return {"status": "success", "mode": "temp_stop", "message": "Temporary stop service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

## 이전 상태가 emergency_stop이면 => waiting 으로 바뀌고
## 이전 상태가 temp_stop이면 => before_mode
@router.post("/{seq}/call-service/resume")
def call_resume_endpoint(seq: str):
    """로봇 재개 서비스 호출"""
    try:
        response = call_resume_service(seq)
        return {"status": "success", "message": "Resume service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/waiting")
def call_waiting_endpoint(seq: str):
    """로봇 대기 서비스 호출"""
    try:
        response = call_waiting_service(seq)
        return {"status": "success", "mode": "waiting", "message": "Waiting service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/manual")
def call_manual_endpoint(seq: str):
    """로봇 매뉴얼 모드 변경 서비스 호출"""
    try:
        response = call_manual_service(seq)
        return {"status": "success", "message": "Manual mode service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/cmd_vel")
async def publish_cmd_vel(seq: str, direction: str):
    """cmd_vel 토픽에 속도 명령 발행"""
    logger.info(f"Publishing cmd_vel for robot {seq}, direction: {direction}")
    
    # Twist 메시지 생성 (ROS 메시지 형식에 맞춤)
    twist_msg = {
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }
    
    # 방향에 따른 속도 설정
    if direction == 'up':
        twist_msg['linear']['x'] = 0.2  # 전진
    elif direction == 'down':
        twist_msg['linear']['x'] = -0.2  # 후진
    elif direction == 'left':
        twist_msg['angular']['z'] = 0.5  # 좌회전
    elif direction == 'right':
        twist_msg['angular']['z'] = -0.5  # 우회전
    
    try:
        # cmd_vel 토픽 발행
        topic = f'/robot_{seq}/cmd_vel'
        await ros_bridge.publish(topic, 'geometry_msgs/msg/Twist', twist_msg)
        return {"status": "success", "message": f"Published cmd_vel: {direction}"}
    except Exception as e:
        logger.error(f"Failed to publish cmd_vel: {str(e)}")
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/reset")
def reset_robot(seq: str):
    """로봇 조작 리셋"""
    try:
        # 긴급 정지 서비스 호출
        estop_response = call_estop_service(seq)
        logger.info(f"E-stop service response: {estop_response}")

        # 대기 서비스 호출
        waiting_response = call_waiting_service(seq)
        logger.info(f"Waiting service response: {waiting_response}")

        return {
            "status": "success",
            "message": "Robot reset successfully",
            "responses": {
                "estop": estop_response,
                "waiting": waiting_response
            }
        }
    except Exception as e:
        logger.error(f"Failed to reset robot: {str(e)}")
        return {"status": "error", "message": str(e)}
