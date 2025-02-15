import roslibpy
import asyncio
import logging
from typing import Optional, Callable, Dict, Any
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class ROSConfig(BaseModel):
    """ROS 연결 설정"""
    host: str = "127.0.0.1"
    port: int = 10000
    retry_interval: float = 5.0
    max_retries: int = 3

class ROSConnection:
    """ROS 웹소켓 브릿지 연결 관리"""
    def __init__(self, config: ROSConfig = ROSConfig()):
        self.config = config
        self.client: Optional[roslibpy.Ros] = None
        self.subscribers: Dict[str, roslibpy.Topic] = {}
        self.publishers: Dict[str, roslibpy.Topic] = {}
        self._retry_count = 0

    async def connect(self) -> bool:
        """ROS 브릿지에 연결"""
        try:
            if self.client and self.client.is_connected:
                return True

            self.client = roslibpy.Ros(
                host=self.config.host,
                port=self.config.port
            )
            self.client.run()

            retry_count = 0
            while not self.client.is_connected and retry_count < self.config.max_retries:
                await asyncio.sleep(self.config.retry_interval)
                retry_count += 1

            if not self.client.is_connected:
                raise Exception("ROS 브릿지 연결 실패")

            logger.info("ROS 브릿지 연결 성공")
            return True

        except Exception as e:
            logger.error(f"ROS 브릿지 연결 오류: {str(e)}")
            return False

    async def disconnect(self):
        """ROS 브릿지 연결 해제"""
        if self.client and self.client.is_connected:
            # 모든 구독 해제
            for topic in self.subscribers.values():
                topic.unsubscribe()
            self.subscribers.clear()

            # 모든 발행자 정리
            for topic in self.publishers.values():
                topic.unadvertise()
            self.publishers.clear()

            self.client.terminate()
            self.client = None
            logger.info("ROS 브릿지 연결 해제")

    async def subscribe(self, topic_name: str, message_type: str, callback: Callable, queue_size: int = 1):
        """토픽 구독"""
        if not self.client or not self.client.is_connected:
            raise Exception("ROS 브릿지가 연결되지 않았습니다")

        if topic_name in self.subscribers:
            self.subscribers[topic_name].unsubscribe()

        topic = roslibpy.Topic(
            ros=self.client,
            name=topic_name,
            message_type=message_type,
            queue_size=queue_size
        )
        topic.subscribe(callback)
        self.subscribers[topic_name] = topic
        logger.info(f"토픽 구독 시작: {topic_name}")

    async def publish(self, topic_name: str, message_type: str, message: Dict[str, Any]):
        """토픽 발행"""
        if not self.client or not self.client.is_connected:
            raise Exception("ROS 브릿지가 연결되지 않았습니다")

        if topic_name not in self.publishers:
            topic = roslibpy.Topic(
                ros=self.client,
                name=topic_name,
                message_type=message_type
            )
            self.publishers[topic_name] = topic

        self.publishers[topic_name].publish(message)
        logger.info(f"메시지 발행: {topic_name}")

    @property
    def is_connected(self) -> bool:
        """ROS 브릿지 연결 상태 확인"""
        return self.client is not None and self.client.is_connected