import json
import asyncio
import websockets
from typing import Optional, Dict, Any, Callable
from ....common.config.manager import get_settings
from fastapi import HTTPException

settings = get_settings()

class ROS2WebSocketClient:
    def __init__(self):
        self.url = settings.ros2.BRIDGE_URL
        self.ws: Optional[websockets.WebSocketClientProtocol] = None
        self.connected = False
        self.subscriptions: Dict[str, Callable] = {}
        self.retry_count = 0
        self.max_retries = settings.ros2.MAX_RETRIES

    async def connect(self):
        """ROS2 Bridge에 WebSocket 연결을 수립합니다."""
        try:
            self.ws = await websockets.connect(self.url)
            self.connected = True
            self.retry_count = 0
            print(f"ROS2 Bridge 연결 성공: {self.url}")
            
            # 기존 구독 복구
            for topic in self.subscriptions.keys():
                await self.subscribe_to_topic(topic)
                
        except Exception as e:
            self.connected = False
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                print(f"ROS2 Bridge 연결 재시도 {self.retry_count}/{self.max_retries}")
                await asyncio.sleep(settings.ros2.RETRY_INTERVAL)
                await self.connect()
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"ROS2 Bridge 연결 실패: {str(e)}"
                )

    async def disconnect(self):
        """WebSocket 연결을 종료합니다."""
        if self.ws:
            await self.ws.close()
            self.connected = False
            self.ws = None

    async def subscribe_to_topic(self, topic: str):
        """특정 토픽을 구독합니다."""
        if not self.connected:
            await self.connect()

        subscribe_msg = {
            "op": "subscribe",
            "topic": topic
        }
        await self.ws.send(json.dumps(subscribe_msg))

    async def unsubscribe_from_topic(self, topic: str):
        """토픽 구독을 해제합니다."""
        if not self.connected:
            return

        unsubscribe_msg = {
            "op": "unsubscribe",
            "topic": topic
        }
        await self.ws.send(json.dumps(unsubscribe_msg))
        if topic in self.subscriptions:
            del self.subscriptions[topic]

    async def add_message_handler(self, topic: str, handler: Callable):
        """토픽별 메시지 핸들러를 등록합니다."""
        self.subscriptions[topic] = handler
        if self.connected:
            await self.subscribe_to_topic(topic)

    async def start_listening(self):
        """메시지 수신을 시작합니다."""
        while True:
            try:
                if not self.connected:
                    await self.connect()

                async for message in self.ws:
                    data = json.loads(message)
                    topic = data.get("topic")
                    
                    if topic and topic in self.subscriptions:
                        handler = self.subscriptions[topic]
                        await handler(data["msg"])

            except websockets.ConnectionClosed:
                self.connected = False
                print("ROS2 Bridge 연결이 끊어졌습니다. 재연결 시도...")
                await asyncio.sleep(settings.ros2.RETRY_INTERVAL)
                continue
                
            except Exception as e:
                print(f"메시지 처리 중 오류 발생: {str(e)}")
                continue

    def is_connected(self) -> bool:
        """현재 연결 상태를 반환합니다."""
        return self.connected 