import json
import asyncio
import websockets
import logging


HOST = "192.168.100.34"
# HOST = "localhost"
ROSBRIDGE_URI = f"ws://{HOST}:9090"
TOPIC_NAME = "/ssafy/key_publisher"
PUBLISH_INTERVAL = 2  # 2초마다 발행

running = True  # 루프 실행 플래그

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def publish_message(message: str):
    """ROS 토픽에 단일 메시지 발행"""
    async with websockets.connect(ROSBRIDGE_URI) as websocket:
        ros_message = {
            "op": "publish",
            "topic": TOPIC_NAME,
            "msg": {"data": message}
        }
        logger.info("ROS 토픽에 단일 메시지 발행")
        await websocket.send(json.dumps(ros_message))


# 앞으로만 가는 테스트용
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
