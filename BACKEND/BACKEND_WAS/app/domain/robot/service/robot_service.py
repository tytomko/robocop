import re
from datetime import datetime
from fastapi import HTTPException, UploadFile, WebSocket
from typing import List, Optional, Union, Set, Dict, Callable
from ..repository.robot_repository import RobotRepository
from ..models.robot_models import (
    Robot, Position, BatteryStatus, RobotStatus, RobotImage,
    StatusResponse, LogResponse, Motion
)
from ....common.config.manager import get_settings
import aiohttp
import os
import roslibpy
import json
import logging
from redis import Redis
import asyncio
import websockets
import traceback
from ...ros_publisher.service.ros_bridge_connection import RosBridgeConnection
import math  # radian -> degree 변환을 위해 추가
import time

settings = get_settings()
logger = logging.getLogger(__name__)

class RobotService:
    _instances: Dict[int, "RobotService"] = {}
    
    @classmethod
    async def get_instance(cls, seq: int) -> "RobotService":
        if seq not in cls._instances:
            instance = cls()
            # 추가 초기화 작업이 필요하면 여기에 작성하세요.
            cls._instances[seq] = instance
        return cls._instances[seq]

    """로봇 관련 비즈니스 로직과 프론트엔드 통신을 담당하는 서비스"""
    def __init__(self):
        self.repository = RobotRepository()
        self.ros_bridge = RosBridgeConnection()  # 싱글톤 인스턴스 사용
        self.media_server_url = settings.storage.MEDIA_SERVER_URL
        self.upload_api_url = settings.storage.UPLOAD_API_URL
        
        # 웹소켓 클라이언트 관리 (프론트엔드)
        self.frontend_clients = set()
 
        self.topics = {}  # 구독 중인 토픽 관리를 위해 추가
        
        # Redis 연결
        self.redis_client = Redis(
            host=settings.REDIS_HOST,
            port=settings.REDIS_PORT,
            decode_responses=True
        )
 
        self.message_buffer = {}  # 로봇별 메시지 버퍼ㅣㅐ
        self.last_robot_states = {}  # 로봇별 마지막 상태 저장
        
        # 새로운 down_utm 메시지를 저장할 변수 추가
        self.last_down_utm = None

    # === 프론트엔드 WebSocket 관리 ===
    async def register_frontend_client(self, websocket: WebSocket):
        """프론트엔드 웹소켓 클라이언트 등록"""
        # 이미 등록된 클라이언트인지 확인
        if websocket in self.frontend_clients:
            logger.info(f"클라이언트가 이미 등록되어 있습니다: {id(websocket)}")
            return
        
        self.frontend_clients.add(websocket)
        client_id = id(websocket)
        self.redis_client.sadd('frontend_clients', client_id)
        logger.info(f"프론트엔드 클라이언트 등록: {client_id}")
        
        # 등록 확인 메시지 전송
        try:
            await websocket.send_json({
                "type": "connection_status",
                "status": "connected",
                "client_id": client_id,
                "message": "프론트엔드 연결 성공"
            })
            logger.info(f"클라이언트 {client_id}에 연결 확인 메시지 전송 성공")
        except Exception as e:
            logger.error(f"클라이언트 {client_id} 초기 메시지 전송 실패: {str(e)}")

    async def unregister_frontend_client(self, websocket: WebSocket):
        """프론트엔드 웹소켓 클라이언트 제거"""
        self.frontend_clients.discard(websocket)
        client_id = id(websocket)
        self.redis_client.srem('frontend_clients', client_id)
        logger.info(f"프론트엔드 클라이언트 제거: {client_id}")

    async def broadcast_to_frontend(self, message: dict):
        """프론트엔드 클라이언트들에게 메시지 브로드캐스트"""
        try:
            await asyncio.sleep(0.5)
            if not self.frontend_clients:
                logger.warning("No frontend clients registered")
                return
    
            logger.info(f"Broadcasting to {len(self.frontend_clients)} clients")
            disconnected = set()
            
            for client in self.frontend_clients:
                try:
                    logger.info(f"Sending to client {id(client)}")
                    await client.send_json(message)
                except Exception as e:
                    disconnected.add(client)
                
            # 끊어진 클라이언트 제거
            for client in disconnected:
                await self.unregister_frontend_client(client)
                
        except asyncio.CancelledError:
            # Task 취소 시 조용히 종료
            pass
        except Exception as e:
            logger.error(f"Broadcasting error: {str(e)}")
        finally:
            # 추가적인 정리 작업이 필요한 경우
            pass
        
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
            manufacturer_name = f"robot_{seq:02d}"
            sensor_name = f"sensor_{seq:02d}"
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
                sensorName=sensor_name,
                ipAddress=ip_address,
                status=RobotStatus.WAITING,
                position=Position(),
                motion=Motion(),
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
        # 연결 상태 확인
        if not self.ros_bridge.client or not self.ros_bridge.client.is_connected:
            logger.info("ROS Bridge 연결이 필요합니다.")
            await self.ros_bridge._connect()
        
        # 토픽 설정
        topics = {
            "/robot_1/utm_pose": "geometry_msgs/PoseStamped",
            "/robot_1/status": "robot_custom_interfaces/msg/Status",
        }
        
        # 토픽 구독
        for topic_name, msg_type in topics.items():
            if topic_name not in self.topics:
                try:
                    topic = roslibpy.Topic(self.ros_bridge.client, topic_name, msg_type)
                    
                    def callback(message):
                        processed_message = self.process_message_by_topic(topic_name, message)
                        if processed_message:
                            asyncio.create_task(self.broadcast_to_frontend(processed_message))
                    
                    topic.subscribe(callback)
                    self.topics[topic_name] = topic
                    logger.info(f"토픽 구독 성공: {topic_name}")
                except Exception as e:
                    logger.error(f"토픽 구독 실패 {topic_name}: {str(e)}")

    async def process_message_by_topic(self, topic_name: str, message: dict):
        """토픽별 메시지를 가공하고 필요시 DB 업데이트"""
        try:
            topic_parts = topic_name.split('/')
            
            if topic_parts[1] == "filtered_points":  # 센서 데이터
                sensor_name = topic_parts[0][1:]  # /ssafy -> ssafy
                
                header = message.get("header", {})
                points_data = {
                    "sensorName": sensor_name,
                    "header": {
                        "stamp": {
                            "sec": header.get("stamp", {}).get("sec", 0),
                            "nanosec": header.get("stamp", {}).get("nanosec", 0)
                        },
                        "frame_id": header.get("frame_id", "")
                    },
                    "points_info": {
                        "height": message.get("height", 0),
                        "width": message.get("width", 0),
                        "fields": message.get("fields", [])
                    },
                    "timestamp": datetime.now()
                }
                
                # 센서 데이터 버퍼 초기화 및 추가
                if sensor_name not in self.message_buffer:
                    self.message_buffer[sensor_name] = []
                
                self.message_buffer[sensor_name].append(points_data)
                
                # 반환값: 필터링된 포인트 데이터 (sensorName 포함)
                return {
                    "type": "filtered_points",
                    "sensorName": sensor_name,
                    "data": points_data
                }
            
            elif len(topic_parts) > 1:  # 로봇 관련 토픽 처리
                manufacturer_name = topic_parts[1]
                if manufacturer_name not in self.message_buffer:
                    self.message_buffer[manufacturer_name] = []
                
                if "utm_pose" in topic_name:
                    position = message.get("pose", {}).get("position", {})
                    
                    processed_data = {
                        "manufactureName": manufacturer_name,
                        "position": {
                            "x": round(position.get("x", 0), 4),
                            "y": round(position.get("y", 0), 4),
                            "z": round(position.get("z", 0), 4),
                            "orientation": self.last_robot_states.get(manufacturer_name, {}).get("orientation", 0)  # heading에서 업데이트된 값 사용
                        },
                        "timestamp": datetime.now()
                    }
                    
                    # 위치 변화 감지 (일정 거리 이상 변화가 있을 때만)
                    last_state = self.last_robot_states.get(manufacturer_name, {})
                    last_position = last_state.get("position", {})
                    
                    # 위치 변화량 계산
                    position_changed = False
                    if last_position:
                        dx = abs(last_position.get("x", 0) - position.get("x", 0))
                        dy = abs(last_position.get("y", 0) - position.get("y", 0))
                        dz = abs(last_position.get("z", 0) - position.get("z", 0))
                        
                        # 일정 거리 이상 변화가 있을 때만 업데이트 (예: 0.5m)
                        threshold = 0.5
                        position_changed = (dx > threshold or dy > threshold or dz > threshold)
                    
                    # 버퍼에 추가
                    self.message_buffer[manufacturer_name].append(processed_data)
                    
                    # 위치가 크게 변했을 때만 DB 업데이트
                    if position_changed:
                        await self._update_robot_position(manufacturer_name, processed_data)
                        self.last_robot_states[manufacturer_name]["position"] = {
                            "x": position.get("x", 0),
                            "y": position.get("y", 0),
                            "z": position.get("z", 0)
                        }
                    
                    # 반환값: 로봇 위치 데이터 (manufactureName 포함)
                    return {
                        "type": "robot_position",
                        "manufactureName": manufacturer_name,
                        "data": processed_data
                    }
                
                elif "heading" in topic_name:
                    heading_rad = message.get("data", 0)
                    heading_deg = math.degrees(heading_rad)
                    
                    if manufacturer_name not in self.last_robot_states:
                        self.last_robot_states[manufacturer_name] = {}
                    
                    last_state = self.last_robot_states[manufacturer_name]
                    last_state["orientation"] = heading_deg
                    
                    # 방향이 크게 변했을 때만 DB 업데이트 (5도 이상)
                    orientation_changed = (
                        abs(last_state.get("last_orientation", 0) - heading_deg) > 5
                    )
                    
                    if orientation_changed:
                        processed_data = {
                            "manufactureName": manufacturer_name,
                            "position": {
                                "orientation": heading_deg
                            },
                            "timestamp": datetime.now()
                        }
                        
                        await self._update_robot_position(manufacturer_name, processed_data)
                        last_state["last_orientation"] = heading_deg
                    
                    # 반환값: 로봇 방향 데이터 (manufactureName 포함)
                    return {
                        "type": "robot_heading",
                        "manufactureName": manufacturer_name,
                        "data": {
                            "orientation": heading_deg,
                            "timestamp": datetime.now()
                        }
                    }
                
                elif topic_name.endswith(('/speed_kph', '/speed_mps')):
                    current_time = datetime.now()
                    
                    if manufacturer_name not in self.last_robot_states:
                        self.last_robot_states[manufacturer_name] = {}
                    
                    last_state = self.last_robot_states[manufacturer_name]
                    
                    data_value = message.get("data", 0)
                    if "speed_kph" in topic_name:
                        last_state["speed_kph"] = round(data_value, 2)
                    elif "speed_mps" in topic_name:
                        last_state["speed_mps"] = round(data_value, 2)
                    
                    # 모든 속도 정보가 있는 경우에만 처리
                    if all(key in last_state for key in ["speed_kph", "speed_mps"]):
                        processed_data = {
                            "manufactureName": manufacturer_name,
                            "motion": {
                                "kph": last_state["speed_kph"],
                                "mps": last_state["speed_mps"]
                            },
                            "timestamp": current_time
                        }
                        
                        self.message_buffer[manufacturer_name].append(processed_data)
                        
                        # 속도가 크게 변했을 때만 DB 업데이트
                        motion_changed = (
                            abs(last_state.get("last_speed_kph", 0) - last_state["speed_kph"]) > 0.5
                        )
                        
                        if motion_changed:
                            await self._update_robot_motion(manufacturer_name, processed_data)
                            last_state["last_speed_kph"] = last_state["speed_kph"]
                        
                        # 반환값: 로봇 모션 데이터 (manufactureName 포함)
                        return {
                            "type": "robot_motion",
                            "manufactureName": manufacturer_name,
                            "data": processed_data
                        }

        except Exception as e:
            logger.error(f"메시지 처리 중 오류: {str(e)}")
            logger.error(traceback.format_exc())
            return None  # 에러 발생 시 None 반환

    async def _update_robot_and_logs(self, manufacturer_name: str, data: dict):
        """로봇 상태 업데이트 및 로그 저장"""
        try:
            # 로봇 정보 업데이트
            update_data = {
                "status": data["status"],
                "networkHealth": data["networkHealth"],
                "battery": data["battery"],
                "cpuTemp": data["cpuTemp"],
                "IsActive": data["IsActive"],
                "lastActive": data["timestamp"]
            }
            
            robot = await self.repository.find_robot_by_name(manufacturer_name)
            if robot:
                await self.repository.update_robot_status(robot.seq, update_data)
                
                # 버퍼의 로그들을 DB에 저장
                if manufacturer_name in self.message_buffer:
                    logs = self.message_buffer[manufacturer_name]
                    if logs:
                        await self._save_robot_logs(robot.seq, logs)
                        self.message_buffer[manufacturer_name] = []  # 버퍼 비우기
                        
        except Exception as e:
            logger.error(f"로봇 상태 업데이트 중 오류: {str(e)}")

    async def _save_robot_logs(self, robot_seq: int, logs: list):
        """로봇 로그를 형식화하고 저장"""
        try:
            formatted_logs = []
            for log in logs:
                log_type = log.get("type")
                
                if log_type == "status":
                    formatted_log = {
                        "robotSeq": robot_seq,
                        "type": "status",
                        "data": {
                            "status": log["status"],
                            "networkHealth": log["networkHealth"],
                            "battery": log["battery"],
                            "cpuTemp": log["cpuTemp"],
                            "isActive": log["IsActive"]
                        },
                        "timestamp": log["timestamp"],
                        "createdAt": datetime.now()
                    }
                elif log_type == "position":
                    formatted_log = {
                        "robotSeq": robot_seq,
                        "type": "position",
                        "data": {
                            "position": log["position"],
                            "motion": log.get("motion", {}),
                        },
                        "timestamp": log["timestamp"],
                        "createdAt": datetime.now()
                    }
                elif log_type == "error":
                    formatted_log = {
                        "robotSeq": robot_seq,
                        "type": "error",
                        "data": {
                            "code": log.get("error_code"),
                            "message": log.get("error_message"),
                            "status": log.get("status")
                        },
                        "timestamp": log["timestamp"],
                        "createdAt": datetime.now()
                    }
                
                formatted_logs.append(formatted_log)
            
            if formatted_logs:
                await self.repository.save_robot_logs(formatted_logs)
                
        except Exception as e:
            logger.error(f"로그 형식화 및 저장 중 오류: {str(e)}")
            logger.error(traceback.format_exc())

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

    async def _update_robot_position(self, manufacturer_name: str, data: dict):
        """로봇 위치 업데이트"""
        try:
            robot = await self.repository.find_robot_by_name(manufacturer_name)
            if robot:
                update_data = {
                    "position": data["position"],
                    "lastActive": data["timestamp"]
                }
                await self.repository.update_robot_status(robot.seq, update_data)
                
        except Exception as e:
            logger.error(f"로봇 위치 업데이트 중 오류: {str(e)}")

    async def _update_robot_motion(self, manufacturer_name: str, data: dict):
        """로봇 모션(속도/방향) 정보 업데이트"""
        try:
            robot = await self.repository.find_robot_by_name(manufacturer_name)
            if robot:
                update_data = {
                    "motion": data["motion"],
                    "lastActive": data["timestamp"]
                }
                await self.repository.update_robot_status(robot.seq, update_data)
                
        except Exception as e:
            logger.error(f"로봇 모션 업데이트 중 오류: {str(e)}")

    async def subscribe_down_utm(self, seq: int) -> None:
        """
        /robot_{seq}/down_utm 토픽(메시지 타입: geometry_msgs/msg/PoseStamped)을 구독합니다.
        """
        topic_name = f"/robot_{seq}/down_utm"
        msg_type = "geometry_msgs/msg/PoseStamped"
        # 이미 해당 토픽을 구독 중이면 재구독하지 않음
        if topic_name in self.topics:
            logger.info(f"Already subscribed to {topic_name}")
            return
        
        try:
            # ROS Bridge 연결이 되어 있지 않다면 재연결 시도
            if not self.ros_bridge.client or not self.ros_bridge.client.is_connected:
                self.ros_bridge.ensure_connected()
            topic = roslibpy.Topic(self.ros_bridge.client, topic_name, msg_type)
            topic.subscribe(self._on_down_utm_message)
            self.topics[topic_name] = topic
            logger.info(f"Subscribed to {topic_name} with type {msg_type}")
        except Exception as e:
            logger.error(f"Failed to subscribe to {topic_name}: {str(e)}")
            raise

    def _on_down_utm_message(self, message: dict) -> None:
        """
        /robot_{seq}/down_utm 토픽에서 수신한 메시지를 처리합니다.
        """
        try:
            #logger.info(f"Received down_utm message: {message}")
            header = message.get("header", {})
            pose = message.get("pose", {})
            position = pose.get("position", {})
            orientation = pose.get("orientation", {})
            down_utm_data = {
                "header": header,
                "position": position,
                "orientation": orientation
            }
            # 마지막으로 수신한 메시지를 저장합니다.
            self.last_down_utm = down_utm_data
            #logger.info(f"Updated last_down_utm: {down_utm_data}")
        except Exception as e:
            logger.error(f"Error processing down_utm message: {str(e)}")

