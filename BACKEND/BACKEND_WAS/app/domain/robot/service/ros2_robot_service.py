from typing import Optional, Dict, Callable
import websockets

class ROS2RobotClient:
    def __init__(self):
        self.url = "http://localhost:11311"  # 예시 설정값
        self.ws: Optional[websockets.WebSocketClientProtocol] = None
        self.connected = False
        self.subscriptions: Dict[str, Callable] = {}
        self.retry_count = 0
        self.max_retries = 3  # 예시 설정값

    def get_robot_topic(self, robot_id: int, topic_type: str) -> str:
        """로봇별 토픽 경로를 생성합니다."""
        return f"robot_{robot_id}/{topic_type}"

    async def subscribe_to_robot(self, robot_id: int, handler: Callable):
        """특정 로봇의 상태 토픽을 구독합니다."""
        topic = self.get_robot_topic(robot_id, "status")
        await self.add_message_handler(topic, handler)

    ... 