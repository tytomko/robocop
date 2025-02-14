import re
from datetime import datetime
from fastapi import HTTPException, UploadFile, WebSocket
from typing import List, Optional, Union, Set, Dict, Callable
from ..repository.robot_repository import RobotRepository
from ..models.robot_models import (
    Robot, Position, BatteryStatus, RobotStatus, RobotImage,
    StatusResponse, LogResponse, ROS2RobotStatus
)
from ....common.config.manager import get_settings
import aiohttp
import os
from .ros2_robot_service import ROS2RobotClient
import roslibpy
import json
import logging
from redis import Redis
import asyncio
import websockets

settings = get_settings()
logger = logging.getLogger(__name__)

class RobotService:
    """로봇 관련 비즈니스 로직과 프론트엔드 통신을 담당하는 서비스"""
    def __init__(self):
        self.repository = RobotRepository()
        self.ros2_client = ROS2RobotClient()  # ROS2와의 통신용
        self.media_server_url = settings.storage.MEDIA_SERVER_URL
        self.upload_api_url = settings.storage.UPLOAD_API_URL
        
        # 웹소켓 클라이언트 관리 (프론트엔드)
        self.frontend_clients: Set[WebSocket] = set()
        
        # Redis 연결
        self.redis_client = Redis(
            host=settings.REDIS_HOST,
            port=settings.REDIS_PORT,
            decode_responses=True
        )

    # === 프론트엔드 WebSocket 관리 ===
    async def register_frontend_client(self, websocket: WebSocket):
        """프론트엔드 웹소켓 클라이언트 등록"""
        self.frontend_clients.add(websocket)
        client_id = id(websocket)
        self.redis_client.sadd('frontend_clients', client_id)
        logger.info(f"프론트엔드 클라이언트 등록: {client_id}")

    async def unregister_frontend_client(self, websocket: WebSocket):
        """프론트엔드 웹소켓 클라이언트 제거"""
        self.frontend_clients.remove(websocket)
        client_id = id(websocket)
        self.redis_client.srem('frontend_clients', client_id)
        logger.info(f"프론트엔드 클라이언트 제거: {client_id}")

    async def broadcast_to_frontend(self, message: dict):
        """프론트엔드 클라이언트들에게 메시지 브로드캐스트"""
        disconnected = set()
        for client in self.frontend_clients:
            try:
                await client.send_json(message)
            except Exception as e:
                logger.error(f"프론트엔드 메시지 전송 실패: {str(e)}")
                disconnected.add(client)
        
        for client in disconnected:
            await self.unregister_frontend_client(client)

    # === 기존 REST API 관련 메서드들 ===
    async def create_robot(self, nickname: str, ip_address: str, image: Optional[UploadFile] = None) -> Robot:
        try:
            # IP 주소 형식 검증
            ip_pattern = r'^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'
            if not re.match(ip_pattern, ip_address):
                raise HTTPException(
                    status_code=400,
                    detail="유효하지 않은 IP 주소 형식입니다. (예: 192.168.1.100)"
                )

            # 로봇 이름(nickname) 중복 검사
            existing_robot = await self.repository.find_robot_by_nickname(nickname)
            if existing_robot:
                raise HTTPException(
                    status_code=400,
                    detail="이미 사용 중인 로봇 이름입니다."
                )

            # 새 로봇 seq 생성
            seq = await self.repository.get_next_robot_id()
            
            # 제조사 지정 이름 생성
            manufacturer_name = f"ROBOT_{seq:03d}"

            # 이미지 처리
            robot_image = None
            if image:
                image_id = f"robot_{seq}"
                # EC2 미디어 서버에 이미지 업로드
                ext = os.path.splitext(image.filename)[1].lower()
                content = await image.read()
                async with aiohttp.ClientSession() as session:
                    data = aiohttp.FormData()
                    data.add_field('file',
                                 content,
                                 filename=f"{image_id}{ext}",
                                 content_type=image.content_type)
                    
                    async with session.post(
                        f"{self.upload_api_url}/robots_image",
                        data=data
                    ) as response:
                        if response.status != 200:
                            raise HTTPException(status_code=500, detail="이미지 업로드 실패")

                image_url = f"{self.media_server_url}/robots_image/{image_id}{ext}"
                robot_image = RobotImage(
                    imageId=image_id,
                    url=image_url,
                    createdAt=datetime.now()
                )

            # 로봇 생성
            robot = Robot(
                seq=seq,
                manufactureName=manufacturer_name,
                nickname=nickname,
                ipAddress=ip_address,
                status=RobotStatus.WAITING,
                position=Position(),
                battery=BatteryStatus(),
                image=robot_image,
                startAt=datetime.now(),
                lastActive=datetime.now(),
                createdAt=datetime.now()
            )

            return await self.repository.create_robot(robot)
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 생성 중 오류 발생: {str(e)}"
            )

    async def get_all_robots(self) -> List[Robot]:
        try:
            robots = await self.repository.find_all_robots()
            if not robots:
                return []
            return robots
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 목록 조회 중 오류 발생: {str(e)}"
            )

    async def get_robot(self, identifier: Union[str, int]) -> Robot:
        """ID 또는 닉네임으로 로봇을 조회합니다."""
        robot = await self.repository.find_robot_by_identifier(identifier)
        if not robot:
            raise HTTPException(
                status_code=404,
                detail="로봇을 찾을 수 없습니다."
            )
        return robot

    async def get_robot_id(self, identifier: Union[str, int]) -> int:
        """ID 또는 닉네임으로 로봇 ID를 조회합니다."""
        if isinstance(identifier, int):
            return identifier
        
        robot_id = await self.repository.get_robot_id_by_nickname(identifier)
        if not robot_id:
            raise HTTPException(
                status_code=404,
                detail="로봇을 찾을 수 없습니다."
            )
        return robot_id

    async def delete_robot(self, identifier: Union[str, int]) -> bool:
        """로봇을 소프트 딜리트 처리합니다."""
        try:
            # 로봇 존재 여부 확인
            robot = await self.get_robot(identifier)
            if not robot:
                return False

            # 이미지가 있다면 EC2 미디어 서버에서 삭제
            if robot.image:
                try:
                    async with aiohttp.ClientSession() as session:
                        async with session.delete(
                            f"{self.upload_api_url}/robots_image/{robot.image.imageId}"
                        ) as response:
                            if response.status != 200:
                                print(f"이미지 삭제 실패: {robot.image.imageId}")
                except Exception as e:
                    print(f"이미지 삭제 중 오류 발생: {str(e)}")

            # 소프트 딜리트 처리
            return await self.repository.delete_robot(robot.seq)
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 삭제 중 오류 발생: {str(e)}"
            )

    async def update_status(self, identifier: Union[str, int], status: str) -> StatusResponse:
        """로봇의 상태를 업데이트합니다."""
        try:
            # 로봇 존재 여부 확인
            robot = await self.get_robot(identifier)
            if not robot:
                raise HTTPException(
                    status_code=404,
                    detail="로봇을 찾을 수 없습니다."
                )

            # 상태 유효성 검사
            if status not in [s.value for s in RobotStatus]:
                raise HTTPException(
                    status_code=400,
                    detail=f"유효하지 않은 상태입니다. 가능한 상태: {', '.join([s.value for s in RobotStatus])}"
                )
            
            update_data = {
                "status": status,
                "lastActive": datetime.now()
            }
            
            updated_robot = await self.repository.update_robot_status(robot.seq, update_data)
            if not updated_robot:
                raise HTTPException(
                    status_code=500,
                    detail="로봇 상태 업데이트 실패"
                )
            
            return StatusResponse(
                robotId=str(updated_robot.seq),
                status=updated_robot.status,
                timestamp=updated_robot.lastActive,
                location=updated_robot.position
            )
        except HTTPException as e:
            raise e
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 상태 업데이트 중 오류 발생: {str(e)}"
            )

    async def get_logs(self, robot_id: int, log_type: str) -> LogResponse:
        robot = await self.get_robot(robot_id)
        
        # 실제 로그 데이터를 가져오는 로직 구현 필요
        return LogResponse(
            robotId=str(robot.seq),
            routes=[],  # 실제 경로 로그 데이터
            detections=[],  # 실제 감지 로그 데이터
            pagination={"total": 0, "page": 1, "size": 10}
        )

    async def start_ros2_client(self):
        """ROS2 클라이언트를 시작합니다."""
        # 모든 활성 로봇에 대한 토픽 구독
        robots = await self.get_all_robots()
        for robot in robots:
            if not robot.IsDeleted:
                await self.ros2_client.subscribe_to_robot(
                    robot.seq, 
                    self._handle_robot_status
                )
        
        # 클라이언트 시작
        await self.ros2_client.start_listening()

    async def _handle_robot_status(self, message: dict):
        """로봇 상태 메시지를 처리합니다."""
        try:
            # ROS2 메시지를 모델로 변환
            ros2_status = ROS2RobotStatus(
                robot_id=message.get("robot_id"),
                status=message.get("status"),
                battery_level=message.get("battery_level"),
                battery_charging=message.get("battery_charging"),
                position_x=message.get("position_x"),
                position_y=message.get("position_y"),
                position_z=message.get("position_z"),
                orientation=message.get("orientation"),
                cpu_temp=message.get("cpu_temp"),
                error_code=message.get("error_code"),
                error_message=message.get("error_message"),
                timestamp=datetime.now()
            )

            # 로봇 상태 업데이트
            update_data = {
                "status": ros2_status.status,
                "position": Position(
                    x=ros2_status.position_x,
                    y=ros2_status.position_y,
                    z=ros2_status.position_z,
                    orientation=ros2_status.orientation
                ),
                "battery": BatteryStatus(
                    level=ros2_status.battery_level,
                    isCharging=ros2_status.battery_charging
                ),
                "cpuTemp": ros2_status.cpu_temp,
                "lastActive": ros2_status.timestamp
            }

            # DB 업데이트
            robot_id = int(ros2_status.robot_id.split('_')[1])  # robot_1 -> 1
            await self.repository.update_robot_status(robot_id, update_data)

            # WebSocket 클라이언트들에게 브로드캐스트
            # (이 부분은 WebSocket 구현 후 추가)

        except Exception as e:
            print(f"로봇 상태 처리 중 오류 발생: {str(e)}")

    async def update_nickname(self, seq: int, new_nickname: str) -> Robot:
        """로봇의 닉네임을 업데이트합니다."""
        try:
            # 닉네임 중복 검사
            existing_robot = await self.repository.find_robot_by_nickname(new_nickname)
            if existing_robot:
                raise HTTPException(
                    status_code=400,
                    detail="이미 사용 중인 로봇 닉네임입니다."
                )

            updated_robot = await self.repository.update_robot_nickname(seq, new_nickname)
            if not updated_robot:
                raise HTTPException(
                    status_code=500,
                    detail="로봇 닉네임 업데이트 실패"
                )
            
            return updated_robot
        except HTTPException as e:
            raise e
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 닉네임 업데이트 중 오류 발생: {str(e)}"
            )


class ROS2WebSocketClient:
    """ROS2 Bridge와의 웹소켓 통신을 담당하는 클라이언트"""
    def __init__(self):
        self.url = settings.ros2.BRIDGE_URL
        self.ws: Optional[websockets.WebSocketClientProtocol] = None
        self.connected = False
        self.subscriptions: Dict[str, Callable] = {}
        self.retry_count = 0
        self.max_retries = settings.ros2.MAX_RETRIES
        
        # 웹소켓 클라이언트 관리
        self.clients: Set[WebSocket] = set()
        
        # Redis 연결 설정
        self.redis_client = Redis(
            host=settings.REDIS_HOST,
            port=settings.REDIS_PORT,
            decode_responses=True
        )

    async def connect(self):
        """ROS2 Bridge에 WebSocket 연결을 수립합니다."""
        try:
            self.ws = await websockets.connect(self.url)
            self.connected = True
            self.retry_count = 0
            logger.info(f"ROS2 Bridge 연결 성공: {self.url}")
            
            # 기존 구독 복구
            for topic in self.subscriptions.keys():
                await self.subscribe_to_topic(topic)
                
        except Exception as e:
            self.connected = False
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                logger.info(f"ROS2 Bridge 연결 재시도 {self.retry_count}/{self.max_retries}")
                await asyncio.sleep(settings.ros2.RETRY_INTERVAL)
                await self.connect()
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"ROS2 Bridge 연결 실패: {str(e)}"
                )

    async def register_client(self, websocket: WebSocket):
        """새로운 웹소켓 클라이언트 등록"""
        self.clients.add(websocket)
        
        # Redis에 클라이언트 정보 저장
        client_id = id(websocket)
        self.redis_client.sadd('active_clients', client_id)
        logger.info(f"새로운 웹소켓 클라이언트 등록됨: {client_id}")

    async def unregister_client(self, websocket: WebSocket):
        """웹소켓 클라이언트 연결 해제"""
        self.clients.remove(websocket)
        
        # Redis에서 클라이언트 정보 제거
        client_id = id(websocket)
        self.redis_client.srem('active_clients', client_id)
        logger.info(f"웹소켓 클라이언트 연결 해제: {client_id}")

    async def broadcast_message(self, message: dict):
        """모든 연결된 클라이언트에게 메시지를 전송합니다."""
        disconnected_clients = set()
        for client in self.clients:
            try:
                await client.send_json(message)
            except Exception as e:
                logger.error(f"클라이언트 메시지 전송 실패: {str(e)}")
                disconnected_clients.add(client)
        
        # 끊어진 클라이언트 제거
        for client in disconnected_clients:
            await self.unregister_client(client)

    async def start_listening(self):
        """메시지 수신을 시작합니다."""
        while True:
            try:
                if not self.connected:
                    await self.connect()

                if self.ws:
                    async for message in self.ws:
                        data = json.loads(message)
                        logger.debug(f"ROS2 Bridge로부터 메시지 수신: {data}")
                        
                        # 테스트 메시지 처리
                        if "test_message" in data:
                            test_msg = data["test_message"]
                            logger.info(f"테스트 메시지 수신: {test_msg}")
                            await self.broadcast_message({
                                "type": "test_message",
                                "data": test_msg
                            })
                        
                        # 토픽 메시지 처리
                        topic = data.get("topic")
                        if topic and topic in self.subscriptions:
                            handler = self.subscriptions[topic]
                            await handler(data["msg"])

            except websockets.ConnectionClosed:
                self.connected = False
                logger.warning("ROS2 Bridge 연결이 끊어졌습니다. 재연결 시도...")
                await asyncio.sleep(settings.ros2.RETRY_INTERVAL)
                continue
                
            except Exception as e:
                logger.error(f"메시지 처리 중 오류 발생: {str(e)}")
                continue

    def is_connected(self) -> bool:
        """현재 연결 상태를 반환합니다."""
        return self.connected

    async def send_message(self, message: dict):
        """ROS2 Bridge로 메시지를 전송합니다."""
        try:
            if not self.connected or not self.ws:
                await self.connect()
            
            # 메시지를 JSON 문자열로 변환하여 전송
            message_str = json.dumps(message)
            await self.ws.send(message_str)
            logger.info(f"ROS2 Bridge로 메시지 전송 성공: {message}")
            
            # 메시지 전송 성공을 클라이언트들에게 알림
            status_message = {
                "type": "bridge_status",
                "data": {
                    "status": "success",
                    "message": "메시지가 ROS2 Bridge로 전송되었습니다",
                    "sent_data": message
                }
            }
            await self.broadcast_message(status_message)
            
        except websockets.ConnectionClosed:
            logger.error("ROS2 Bridge 연결이 끊어졌습니다")
            self.connected = False
            # 연결 재시도
            await self.connect()
            # 재시도 후 메시지 다시 전송
            await self.send_message(message)
            
        except Exception as e:
            logger.error(f"ROS2 Bridge 메시지 전송 실패: {str(e)}")
            error_message = {
                "type": "bridge_status",
                "data": {
                    "status": "error",
                    "message": f"메시지 전송 실패: {str(e)}",
                    "failed_data": message
                }
            }
            await self.broadcast_message(error_message)
            raise HTTPException(
                status_code=500,
                detail=f"ROS2 Bridge 메시지 전송 실패: {str(e)}"
            )

