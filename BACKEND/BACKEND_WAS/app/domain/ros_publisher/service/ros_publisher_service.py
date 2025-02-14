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
async def publish_message(seq: str, message: str):
    """ROS 토픽에 단일 메시지 발행"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        topic_name = f"/robot_{seq}/key_publisher"
        publisher = roslibpy.Topic(client, topic_name, 'std_msgs/String')
        ros_message = roslibpy.Message({'data': message})
        
        publisher.publish(ros_message)
        logger.info(f"ROS 토픽에 메시지 발행: {message}")
        
        publisher.unadvertise()
        
    except Exception as e:
        logger.error(f"메시지 발행 실패: {e}")
        raise e

# 서비스 호출 함수
def call_homing_service(seq: str):
    """ROS 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/homing', 'robot_custom_interfaces/srv/Homing')
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

def call_navigate_service(seq: str, goal: dict):
    """ROS Navigate 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/navigate', 'robot_custom_interfaces/srv/Navigate')
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

def call_patrol_service(seq: str, goals: dict):
    """ROS Patrol 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/patrol', 'robot_custom_interfaces/srv/Patrol')
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

def call_estop_service(seq: str):
    """ROS E-stop 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/stop', 'robot_custom_interfaces/srv/Estop')
        request = roslibpy.ServiceRequest()

        response = service.call(request)
        if response is not None:
            logger.info(f"E-stop Service Response: {response}")
            return response
        else:
            logger.error("E-stop service response is None.")
            raise Exception("E-stop service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call E-stop service: {e}")
        raise e

def call_temp_stop_service(seq: str):
    """ROS Temporary Stop 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/temp_stop', 'robot_custom_interfaces/srv/Estop')
        request = roslibpy.ServiceRequest()

        response = service.call(request)
        if response is not None:
            logger.info(f"Temp Stop Service Response: {response}")
            return response
        else:
            logger.error("Temp Stop service response is None.")
            raise Exception("Temp Stop service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call Temp Stop service: {e}")
        raise e

def call_resume_service(seq: str):
    """ROS Resume 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/resume', 'robot_custom_interfaces/srv/Estop')
        request = roslibpy.ServiceRequest()

        response = service.call(request)
        if response is not None:
            logger.info(f"Resume Service Response: {response}")
            return response
        else:
            logger.error("Resume service response is None.")
            raise Exception("Resume service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call Resume service: {e}")
        raise e

def call_waiting_service(seq: str):
    """ROS Waiting 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/waiting', 'robot_custom_interfaces/srv/Waiting')
        request = roslibpy.ServiceRequest()

        response = service.call(request)
        if response is not None:
            logger.info(f"Waiting Service Response: {response}")
            return response
        else:
            logger.error("Waiting service response is None.")
            raise Exception("Waiting service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call Waiting service: {e}")
        raise e

def call_manual_service(seq: str):
    """ROS Manual 서비스 호출"""
    try:
        ros_connection = RosBridgeConnection()
        client = ros_connection.client

        service = roslibpy.Service(client, f'/robot_{seq}/manual', 'robot_custom_interfaces/srv/Manual')
        request = roslibpy.ServiceRequest()

        response = service.call(request)
        if response is not None:
            logger.info(f"Manual Service Response: {response}")
            return response
        else:
            logger.error("Manual service response is None.")
            raise Exception("Manual service response is None")
            
    except Exception as e:
        logger.error(f"Failed to call Manual service: {e}")
        raise e

