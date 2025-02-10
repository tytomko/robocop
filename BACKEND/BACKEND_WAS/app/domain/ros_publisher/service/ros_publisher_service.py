import json
import asyncio
import logging
import roslibpy
from .ros_bridge_connection import RosBridgeConnection

TOPIC_NAME = "/ssafy/key_publisher"
PUBLISH_INTERVAL = 2  # 2초마다 발행

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 메시지를 발행하는 함수 (roslibpy 사용)
async def publish_message(message: str):
    """ROS 토픽에 단일 메시지 발행"""
    try:
        # 싱글톤 ROS Bridge 클라이언트 가져오기
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        # 토픽에 메시지 발행
        publisher = roslibpy.Topic(client, TOPIC_NAME, 'std_msgs/String')
        ros_message = roslibpy.Message({'data': message})
        
        # 메시지 발행
        publisher.publish(ros_message)
        logger.info(f"ROS 토픽에 메시지 발행: {message}")
        
        # 토픽 연결 해제
        publisher.unadvertise()
        
    except Exception as e:
        logger.error(f"메시지 발행 실패: {e}")
        raise e

# 서비스 호출 함수
def call_homing_service():
    """ROS 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, '/robot_1/homing', 'robot_custom_interfaces/srv/Homing')
        request = roslibpy.ServiceRequest()

        response = service.call(request)
        if response is not None:
            logger.info(f"Service Response: {response}")
            return response
        else:
            logger.error("Service response is None.")
            raise Exception("Service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call service: {e}")
        raise e

def call_navigate_service(goal: dict):
    """ROS Navigate 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, '/robot_1/navigate', 'robot_custom_interfaces/srv/Navigate')
        request = roslibpy.ServiceRequest(goal)

        response = service.call(request)
        if response is not None:
            logger.info(f"Navigate Service Response: {response}")
            return response
        else:
            logger.error("Navigate service response is None.")
            raise Exception("Navigate service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call navigate service: {e}")
        raise e

def call_patrol_service(goals: dict):
    """ROS Patrol 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, '/robot_1/patrol', 'robot_custom_interfaces/srv/Patrol')
        request = roslibpy.ServiceRequest(goals)

        response = service.call(request)
        if response is not None:
            logger.info(f"Patrol Service Response: {response}")
            return response
        else:
            logger.error("Patrol service response is None.")
            raise Exception("Patrol service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call patrol service: {e}")
        raise e
