import asyncio
import json
import logging
import base64
import struct
import time
import websockets
from fastapi import WebSocket
from typing import Set, List, Optional, Dict
from ..models.lidar_models import Point, LidarConfig, LidarData, LidarStatus
from datetime import datetime
from ....common.config.manager import get_settings

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
settings = get_settings()

class LidarService:
    def __init__(self):
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.is_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 3
        self.reconnect_delay = 5  # 초
        self.clients: Set[WebSocket] = set()
        self.last_process_time = 0
        self.status = LidarStatus()
        self.config: Optional[LidarConfig] = None

    async def connect(self, host: str, port: int) -> bool:
        """라이다 웹소켓에 연결을 시도합니다."""
        while self.reconnect_attempts < self.max_reconnect_attempts:
            try:
                if self.websocket:
                    await self.websocket.close()
                
                self.websocket = await websockets.connect(
                    f"ws://{host}:{port}",
                    ping_interval=None,
                    ping_timeout=20,
                    close_timeout=5
                )
                self.is_connected = True
                self.reconnect_attempts = 0
                logger.info("라이다 웹소켓에 연결되었습니다.")
                return True
            
            except (websockets.exceptions.WebSocketException, 
                    ConnectionRefusedError, 
                    asyncio.TimeoutError) as e:
                self.reconnect_attempts += 1
                logger.warning(
                    f"라이다 연결 시도 {self.reconnect_attempts}/{self.max_reconnect_attempts} 실패: {str(e)}"
                )
                if self.reconnect_attempts < self.max_reconnect_attempts:
                    logger.info(f"{self.reconnect_delay}초 후 재연결을 시도합니다.")
                    await asyncio.sleep(self.reconnect_delay)
                else:
                    logger.error("최대 재연결 시도 횟수를 초과했습니다.")
                    self.is_connected = False
                    return False
        
        return False

    async def disconnect(self):
        """라이다 웹소켓 연결을 종료합니다."""
        if self.websocket:
            try:
                await self.websocket.close()
                logger.info("라이다 웹소켓 연결이 종료되었습니다.")
            except Exception as e:
                logger.error(f"라이다 연결 종료 중 오류 발생: {str(e)}")
            finally:
                self.websocket = None
                self.is_connected = False

    async def subscribe(self):
        """ROS2 웹소켓 브릿지에 연결하고 데이터를 구독"""
        try:
            subscribe_message = {
                "op": "subscribe",
                "topic": self.config.topic_name,
                "type": self.config.message_type
            }
            
            await self.websocket.send(json.dumps(subscribe_message))
            logger.info(f"{self.config.topic_name} 토픽 구독을 시작합니다.")
            
            self.status.is_connected = True
            
            while True:
                try:
                    message = await self.websocket.recv()
                    current_time = time.time()
                    
                    if current_time - self.last_process_time >= self.config.update_interval:
                        data = json.loads(message)
                        
                        if data["op"] == "publish":
                            scan_data = data["msg"]
                            points = self._convert_scan_to_points(scan_data)
                            await self._broadcast(points)
                            self.last_process_time = current_time
                            self.status.last_update = datetime.now()
                    else:
                        await asyncio.sleep(0.1)
                        
                except websockets.exceptions.ConnectionClosed:
                    logger.error("ROS 웹소켓 연결이 끊어졌습니다.")
                    self.status.is_connected = False
                    self.status.error_message = "ROS 웹소켓 연결이 끊어졌습니다."
                    break
                    
        except Exception as e:
            logger.error(f"연결 중 오류 발생: {str(e)}")
            self.status.is_connected = False
            self.status.error_message = str(e)
            raise

    def _convert_scan_to_points(self, point_cloud_data) -> List[Point]:
        """PointCloud2 데이터를 Point 객체 리스트로 변환"""
        try:
            binary_data = base64.b64decode(point_cloud_data.get('data', ''))
            point_step = point_cloud_data.get('point_step', 22)
            
            points = []
            step = max(1, len(binary_data) // point_step // self.config.max_points)
            
            for i in range(0, len(binary_data), point_step * step):
                if i + 22 <= len(binary_data):
                    try:
                        x, y, z, intensity = struct.unpack_from('ffff', binary_data, i)
                        
                        if not (abs(x) < 0.001 and abs(y) < 0.001 and abs(z) < 0.001):
                            # ROS2 좌표계를 Three.js 좌표계로 변환
                            points.append(Point(
                                x=-y,  # ROS의 y축을 Three.js의 -x축으로
                                y=z,   # ROS의 z축을 Three.js의 y축으로
                                z=x,   # ROS의 x축을 Three.js의 z축으로
                                intensity=intensity
                            ))
                    except struct.error:
                        continue
            
            self.status.points_count = len(points)
            return points
            
        except Exception as e:
            logger.error(f"포인트 클라우드 데이터 파싱 오류: {str(e)}")
            return []

    async def register_client(self, websocket: WebSocket):
        """새로운 웹 클라이언트 연결을 등록"""
        await websocket.accept()
        self.clients.add(websocket)
        self.status.client_count = len(self.clients)
        logger.info(f"새로운 클라이언트가 연결되었습니다. 현재 클라이언트 수: {self.status.client_count}")

    async def unregister_client(self, websocket: WebSocket):
        """웹 클라이언트 연결 해제"""
        self.clients.remove(websocket)
        self.status.client_count = len(self.clients)
        logger.info(f"클라이언트가 연결 해제되었습니다. 현재 클라이언트 수: {self.status.client_count}")

    async def _broadcast(self, points: List[Point]):
        """모든 연결된 클라이언트에게 포인트 클라우드 데이터 전송"""
        if not self.clients:
            return
            
        lidar_data = LidarData(
            points=points,
            timestamp=time.time()
        )
        
        for client in self.clients.copy():
            try:
                await client.send_json(lidar_data.dict())
            except Exception as e:
                logger.error(f"클라이언트 전송 실패: {str(e)}")
                await self.unregister_client(client)

    def get_status(self) -> LidarStatus:
        """현재 라이다 상태 반환"""
        return self.status