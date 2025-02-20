from fastapi import APIRouter
from ..models.ros_publisher_models import PublishRequest, NavigateRequest, PatrolRequest
from ..service.ros_publisher_service import RosPublisherService
import logging
from app.common.middleware.socket_service import broadcast_to_clients, send_people_images
import json

router = APIRouter()
logger = logging.getLogger(__name__)

# ROS Publisher 서비스 인스턴스 생성
ros_publisher = RosPublisherService()

@router.post("/{seq}/start")
async def start_robot(seq: str):
    """로봇 가동 시작"""
    try:
        response = ros_publisher.call_resume_service(seq)
        logger.info(f"Resume service response: {response}")

        # send_people_images 함수 호출
        logger.info("Calling send_people_images function")
        await send_people_images()
        logger.info("Sent people images to clients")

        return {"status": "success", "message": "Robot start service called successfully", "response": response}
    except Exception as e:
        logger.error(f"Failed to start robot: {str(e)}")
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/cmd_vel")
async def publish_cmd_vel(seq: str, direction: str):
    """cmd_vel 토픽에 속도 명령 발행"""
    logger.info(f"Publishing cmd_vel for robot {seq}, direction: {direction}")
    
    try:
        await ros_publisher.publish_cmd_vel(seq, direction)
        return {
            "status": "success", 
            "message": f"Published direction: {direction}"
        }
    except Exception as e:
        logger.error(f"Failed to publish direction: {str(e)}")
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/homing")
def call_homing_endpoint(seq: str):
    """로봇 호밍 서비스 호출"""
    try:
        estop_response = ros_publisher.call_estop_service(seq)
        logger.info(f"E-stop service response: {estop_response}")

        waiting_response = ros_publisher.call_waiting_service(seq)
        logger.info(f"Waiting service response: {waiting_response}")

        response = ros_publisher.call_homing_service(seq)
        return {"status": "success", "mode": "homing", "message": "Homing service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/navigate")
def call_navigate_endpoint(seq: str, request: NavigateRequest):
    """로봇 네비게이션 서비스 호출"""
    try:
        goal = {"goal": {"x": request.goal.x, "y": request.goal.y, "theta": request.goal.theta}}
        response = ros_publisher.call_navigate_service(seq, goal)
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
        response = ros_publisher.call_patrol_service(seq, goals)
        return {"status": "success", "mode": "patrol", "message": "Patrol service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/estop")
def call_estop_endpoint(seq: str):
    """로봇 긴급 정지 서비스 호출"""
    try:
        response = ros_publisher.call_estop_service(seq)
        return {"status": "success", "mode": "emergency_stop", "message": "E-stop service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/temp-stop")
def call_temp_stop_endpoint(seq: str):
    """로봇 일시정지 서비스 호출"""
    try:
        response = ros_publisher.call_temp_stop_service(seq)
        return {"status": "success", "mode": "temp_stop", "message": "Temporary stop service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/resume")
def call_resume_endpoint(seq: str):
    """로봇 재개 서비스 호출"""
    try:
        response = ros_publisher.call_resume_service(seq)
        return {"status": "success", "message": "Resume service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/waiting")
def call_waiting_endpoint(seq: str):
    """로봇 대기 서비스 호출"""
    try:
        response = ros_publisher.call_waiting_service(seq)
        return {"status": "success", "mode": "waiting", "message": "Waiting service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/call-service/manual")
def call_manual_endpoint(seq: str):
    """로봇 매뉴얼 모드 변경 서비스 호출"""
    try:
        estop_response = ros_publisher.call_estop_service(seq)
        logger.info(f"E-stop service response: {estop_response}")

        waiting_response = ros_publisher.call_waiting_service(seq)
        logger.info(f"Waiting service response: {waiting_response}")
        
        response = ros_publisher.call_manual_service(seq)
        return {"status": "success", "message": "Manual mode service called successfully", "response": response}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@router.post("/{seq}/reset")
def reset_robot(seq: str):
    """로봇 조작 리셋"""
    try:
        estop_response = ros_publisher.call_estop_service(seq)
        logger.info(f"E-stop service response: {estop_response}")

        waiting_response = ros_publisher.call_waiting_service(seq)
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

@router.post("/alert-off")
async def alert_off():
    """경보 해제"""
    try:
        message = {
            "response_type": "MODE_ALERT_STOP"
        }
        await broadcast_to_clients(json.dumps(message))
        logger.info(f"Alert off message sent")
        return {"status": "success", "message": "Alert off message sent successfully"}
    except Exception as e:
        logger.error(f"Failed to send alert off message: {str(e)}")
        return {"status": "error", "message": str(e)}
        
@router.post("/ai-init")
async def ai_init():
    """AI 초기화"""
    try:
        message = {
            "response_type": "MODE_INIT"
        }
        await broadcast_to_clients(json.dumps(message))
        logger.info(f"AI init message sent")
        return {"status": "success", "message": "AI init message sent successfully"}
    except Exception as e:
        logger.error(f"Failed to send ai init message: {str(e)}")
        return {"status": "error", "message": str(e)}
