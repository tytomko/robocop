import json
import asyncio
import logging
import roslibpy

HOST = "192.168.100.34"
# HOST = "localhost"
ROSBRIDGE_URI = f"ws://{HOST}:9090"
TOPIC_NAME = "/ssafy/key_publisher"
PUBLISH_INTERVAL = 2  # 2초마다 발행

running = True  # 루프 실행 플래그

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 메시지를 발행하는 함수 (roslibpy 사용)
async def publish_message(message: str):
    """ROS 토픽에 단일 메시지 발행"""
    try:
        # ROSBridge 연결
        client = roslibpy.Ros(host=HOST, port=9090)
        client.run()  # WebSocket 연결

        # 토픽에 메시지 발행
        publisher = roslibpy.Topic(client, TOPIC_NAME, 'std_msgs/String')
        ros_message = roslibpy.Message({'data': message})
        
        # 메시지 발행
        publisher.publish(ros_message)
        logger.info(f"ROS 토픽에 메시지 발행: {message}")

        client.terminate()  # WebSocket 종료
    except Exception as e:
        logger.error(f"메시지 발행 실패: {e}")

# 서비스 호출 함수
async def call_service():
    """ROS 서비스 호출"""
    try:
        # ROSBridge 연결
        client = roslibpy.Ros(host=HOST, port=9090)
        client.run()  # WebSocket 연결

        # 서비스 설정
        service = roslibpy.Service(client, '/robot_1/homing', 'robot_custom_interfaces/srv/Homing')
        request = roslibpy.ServiceRequest()  # 서비스에 전달할 요청

        # 서비스 호출 (비동기)
        response = service.call(request)
        if response is not None:
            logger.info(f"Service Response: {response}")
        else:
            logger.error("Service response is None.")
        
        client.terminate()  # WebSocket 종료
    except Exception as e:
        logger.error(f"Failed to call service: {e}")

# 2초마다 메시지를 ROS 토픽에 발행하는 비동기 루프
async def publish_loop():
    """2초마다 메시지를 ROS 토픽에 발행하는 비동기 루프"""
    global running
    running = True
    while running:
        await publish_message("UP")
        await asyncio.sleep(PUBLISH_INTERVAL)

def stop_publishing():
    """발행 중지"""
    global running
    running = False
