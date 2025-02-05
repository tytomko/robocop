import asyncio
import json
import base64
import cv2
import av
import numpy as np
from aiortc import VideoStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc import RTCConfiguration, RTCIceServer
import roslibpy
import time
import traceback
from typing import Dict, Optional, Set
from datetime import datetime
from pathlib import Path
from ..repository.camera_repository import CameraRepository
from ..models.camera_models import CameraConfig, WebRTCOffer, WebRTCAnswer, CameraStatus
from ....common.config.manager import get_settings

settings = get_settings()

class CameraVideoTrack(VideoStreamTrack):
    def __init__(self, camera_name: str, repository: CameraRepository):
        super().__init__()
        self.camera_name = camera_name
        self.repository = repository
        self.latest_frame = None
        self.frame_count = 0
        self.pts_time = 0
        self._started = True
        self._stopped = False
        self._start_time = time.time()
        self.data_channel = None
        
        # 레코딩 설정
        self.recording = True
        self.current_session = None
        self.frame_dir = None
        self._initialize_recording()

    def _initialize_recording(self):
        """레코딩 초기화"""
        try:
            self.current_session = datetime.now().strftime(f"{self.camera_name}_%Y%m%d_%H%M%S")
            self.frame_dir = self.repository.create_frame_directory(self.current_session)
            asyncio.create_task(self.repository.create_video_session(self.camera_name))
        except Exception as e:
            print(f"[{self.camera_name}] 레코딩 초기화 에러: {str(e)}")
            self.recording = False

    @property
    def stopped(self):
        return self._stopped

    async def recv(self):
        if self._stopped:
            return None
            
        if self.latest_frame is None:
            await asyncio.sleep(0.1)
            return await self.recv()
            
        frame = self.latest_frame
        self.latest_frame = None
        return frame

    def set_data_channel(self, channel):
        self.data_channel = channel

    def update_frame(self, image_data: bytes):
        if self._stopped:
            return
            
        try:
            np_arr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                return
                
            # 프레임 저장 (레코딩 활성화된 경우)
            if self.recording and self.frame_dir:
                frame_path = self.frame_dir / f"frame_{self.frame_count:06d}.jpg"
                cv2.imwrite(str(frame_path), frame)
            
            # 프레임을 비디오 트랙에 전달
            frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
            frame.pts = self.pts_time
            frame.time_base = fractions.Fraction(1, 1000)
            self.pts_time += 33  # 30fps 가정
            
            self.latest_frame = frame
            self.frame_count += 1
            
        except Exception as e:
            print(f"[{self.camera_name}] 프레임 업데이트 에러: {str(e)}")

    def stop(self):
        if self._stopped:
            return
            
        self._stopped = True
        super().stop()
        
        if self.recording and self.current_session:
            asyncio.create_task(
                self.repository.update_video_session_status(
                    self.current_session,
                    "completed"
                )
            )

class CameraService:
    _shared_ros_clients = {}  # 공유 ROS 클라이언트

    def __init__(self):
        self.repository = CameraRepository()
        self.cameras: Dict[str, Dict] = {}
        self.video_tracks: Dict[str, Set[CameraVideoTrack]] = {}

    async def initialize_camera(self, config: CameraConfig) -> CameraStatus:
        """카메라 초기화 및 ROS 연결"""
        camera_name = config.camera_name
        
        if camera_name not in self.cameras:
            self.cameras[camera_name] = {
                "config": config,
                "ros_client": None,
                "topic_subscriber": None,
                "status": CameraStatus(
                    camera_name=camera_name,
                    is_connected=False,
                    connection_status="connecting"
                )
            }
            self.video_tracks[camera_name] = set()
        
        try:
            await self._connect_ros(camera_name)
            return self.cameras[camera_name]["status"]
        except Exception as e:
            self.cameras[camera_name]["status"].connection_status = "failed"
            self.cameras[camera_name]["status"].error_message = str(e)
            return self.cameras[camera_name]["status"]

    async def _connect_ros(self, camera_name: str):
        """ROS 브릿지 연결 및 토픽 구독"""
        camera = self.cameras[camera_name]
        config = camera["config"]
        
        try:
            # ROS 클라이언트 공유
            client_key = f"{config.ros_bridge_host}:{config.ros_bridge_port}"
            if client_key not in self._shared_ros_clients:
                ros_client = roslibpy.Ros(
                    host=config.ros_bridge_host,
                    port=config.ros_bridge_port
                )
                ros_client.run()
                if not ros_client.is_connected:
                    raise Exception("ROS 브릿지 연결 실패")
                self._shared_ros_clients[client_key] = ros_client
            else:
                ros_client = self._shared_ros_clients[client_key]
            
            topic_subscriber = roslibpy.Topic(
                ros_client,
                config.camera_topic,
                'sensor_msgs/CompressedImage',
                queue_size=1
            )
            
            topic_subscriber.subscribe(
                lambda msg: self._handle_image_message(camera_name, msg)
            )
            
            camera["ros_client"] = ros_client
            camera["topic_subscriber"] = topic_subscriber
            camera["status"].is_connected = True
            camera["status"].connection_status = "connected"
            
        except Exception as e:
            camera["status"].connection_status = "failed"
            camera["status"].error_message = str(e)
            raise

    def _handle_image_message(self, camera_name: str, message: dict):
        """카메라 이미지 메시지 처리"""
        try:
            if 'data' not in message:
                return
                
            raw_data = message['data']
            camera = self.cameras[camera_name]
            camera["status"].last_frame_time = datetime.now()
            
            active_tracks = [track for track in self.video_tracks[camera_name] if not track.stopped]
            camera["status"].active_tracks = len(active_tracks)
            
            for track in active_tracks:
                try:
                    track.update_frame(base64.b64decode(raw_data))
                except Exception as e:
                    print(f"트랙 업데이트 실패: {str(e)}")
                    
        except Exception as e:
            print(f"메시지 처리 에러: {str(e)}")

    async def create_webrtc_connection(self, camera_name: str, offer: WebRTCOffer) -> WebRTCAnswer:
        """WebRTC 피어 연결 생성"""
        if camera_name not in self.cameras:
            raise Exception("카메라를 찾을 수 없습니다")
            
        try:
            config = RTCConfiguration([
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun1.l.google.com:19302"])
            ])
            
            pc = RTCPeerConnection(configuration=config)
            
            # 비디오 트랙 생성 및 추가
            video_track = CameraVideoTrack(camera_name, self.repository)
            self.video_tracks[camera_name].add(video_track)
            pc.addTrack(video_track)
            
            # 데이터 채널 설정
            channel = pc.createDataChannel("stats")
            video_track.set_data_channel(channel)
            
            # 이벤트 핸들러 설정
            @pc.on("connectionstatechange")
            async def on_connectionstatechange():
                if pc.connectionState == "failed":
                    self.video_tracks[camera_name].discard(video_track)
                    video_track.stop()
            
            # Remote Description 설정
            await pc.setRemoteDescription(
                RTCSessionDescription(sdp=offer.sdp, type=offer.type)
            )
            
            # Answer 생성
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            
            return WebRTCAnswer(
                sdp=pc.localDescription.sdp,
                type=pc.localDescription.type
            )
            
        except Exception as e:
            traceback.print_exc()
            raise Exception(f"WebRTC 연결 실패: {str(e)}")

    def get_camera_status(self, camera_name: str) -> Optional[CameraStatus]:
        """카메라 상태 조회"""
        if camera_name in self.cameras:
            return self.cameras[camera_name]["status"]
        return None

    async def save_image(self, image_data: bytes, filename: str):
        """이미지를 로컬 스토리지에 저장"""
        try:
            storage_path = Path(settings.storage.IMAGE_STORAGE_PATH)
            storage_path.mkdir(parents=True, exist_ok=True)
            
            file_path = storage_path / filename
            with open(file_path, 'wb') as f:
                f.write(image_data)
                
            return str(file_path)
        except Exception as e:
            print(f"이미지 저장 실패: {str(e)}")
            raise