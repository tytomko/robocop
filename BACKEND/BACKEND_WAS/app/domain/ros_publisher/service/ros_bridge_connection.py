import roslibpy
import asyncio
import logging
import time
from typing import Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RosBridgeConnection:
    _instance = None
    client = None
    
    HOST = "127.0.0.1"
    PORT = 10000
    MAX_RETRIES = 3
    RETRY_DELAY = 2

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RosBridgeConnection, cls).__new__(cls)
            cls._instance._connect()
        return cls._instance

    async def _connect(self):
        retries = 0
        while retries < self.MAX_RETRIES:
            if not self.client or not self.client.is_connected:
                try:
                    logger.info(f"ROS Bridge 연결 시도 중... (시도 {retries + 1}/{self.MAX_RETRIES})")
                    self.client = roslibpy.Ros(host=self.HOST, port=self.PORT)
                    self.client.run()
                    logger.info("ROS Bridge 서버에 연결되었습니다.")
                    return self.client
                except Exception as e:
                    logger.error(f"ROS Bridge 연결 실패: {e}")
                    self.client = None
                    retries += 1
                    if retries < self.MAX_RETRIES:
                        await asyncio.sleep(self.RETRY_DELAY)

    async def publish(self, topic_name: str, msg_type: str, message: dict):
        try:
            if not self.client or not self.client.is_connected:
                await self._connect()
            
            topic = roslibpy.Topic(self.client, topic_name, msg_type)
            topic.publish(message)
            logger.info(f"Published to {topic_name}: {message}")
            return True
        except Exception as e:
            logger.error(f"Failed to publish to {topic_name}: {str(e)}")
            return False

    def disconnect(self):
        """ROS Bridge 연결 종료"""
        if self.client and self.client.is_connected:
            self.client.terminate()
            logger.info("ROS Bridge 연결이 종료되었습니다.") 