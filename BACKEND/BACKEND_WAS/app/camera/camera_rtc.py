import asyncio
import json
import base64
import cv2
import av
import numpy as np
from aiortc import VideoStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc import RTCConfiguration, RTCIceServer
from datetime import datetime
import fractions
import roslibpy
from pathlib import Path
from ..config import get_settings
from ..database import SessionLocal
from ..models.video import Video
import subprocess
import time

settings = get_settings()

class CameraRTC:
    def __init__(self, camera_topic, camera_name, ros_bridge_host='192.168.100.104', ros_bridge_port=9090):
        """카메라 RTC 클래스 초기화"""
        print(f"\n=== 카메라 초기화: {camera_name} ===")
        print(f"토픽: {camera_topic}")
        print(f"ROS 브릿지: {ros_bridge_host}:{ros_bridge_port}")
        
        self.camera_topic = camera_topic
        self.camera_name = camera_name
        self.ros_bridge_host = ros_bridge_host
        self.ros_bridge_port = ros_bridge_port
        self.ros_client = None
        self.video_tracks = set()
        self.topic_subscriber = None
        
        # 스토리지 경로 설정
        self.storage_path = Path(settings.VIDEO_STORAGE_PATH)
        self.frame_path = Path(settings.FRAME_STORAGE_PATH)
        self.storage_path.mkdir(parents=True, exist_ok=True)
        self.frame_path.mkdir(parents=True, exist_ok=True)

    async def connect_ros(self):
        """ROS 브릿지 연결 및 토픽 구독"""
        try:
            print(f"\n=== [{self.camera_name}] ROS 연결 시작 ===")
            self.ros_client = roslibpy.Ros(
                host=self.ros_bridge_host, 
                port=self.ros_bridge_port
            )
            self.ros_client.run()
            print(f"[{self.camera_name}] ROS 브릿지 연결 성공")
            
            # 토픽 구독 전 연결 상태 확인
            if not self.ros_client.is_connected:
                print(f"[{self.camera_name}] ROS 브릿지 연결 실패")
                return False

            # 토픽 구독
            self.topic_subscriber = roslibpy.Topic(
                self.ros_client,
                self.camera_topic,
                'sensor_msgs/CompressedImage',
                queue_size=1  # 큐 사이즈 추가
            )
            
            # throttle_rate 제거하고 직접 구독
            self.topic_subscriber.subscribe(self._handle_image_message)
            print(f"[{self.camera_name}] ROS 토픽 구독 시작: {self.camera_topic}")
            return True
            
        except Exception as e:
            print(f"ROS 연결 에러 ({self.camera_name}): {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def _handle_image_message(self, message):
        """카메라 이미지 메시지 처리"""
        try:
            print(f"\n=== [{self.camera_name}] 메시지 수신 ===")
            if 'data' not in message:
                print(f"[{self.camera_name}] 에러: 메시지에 'data' 필드가 없음")
                print(f"메시지 내용: {message}")
                return
                
            raw_data = message['data']
            print(f"[{self.camera_name}] 이미지 데이터 수신 성공: {len(raw_data)} bytes")
            
            active_tracks = [track for track in self.video_tracks if not track.stopped]
            print(f"활성 트랙 수: {len(active_tracks)}")
            
            if not active_tracks:
                print(f"[{self.camera_name}] 활성 트랙 없음")
                return
                
            for track in active_tracks:
                try:
                    track.update_frame(raw_data)
                    print(f"[{self.camera_name}] 프레임 업데이트 성공")
                except Exception as e:
                    print(f"[{self.camera_name}] 트랙 업데이트 실패: {str(e)}")
                    traceback.print_exc()
                
        except Exception as e:
            print(f"[{self.camera_name}] 메시지 처리 에러: {str(e)}")
            traceback.print_exc()

    async def create_peer_connection(self, offer_sdp):
        """WebRTC 피어 연결 생성"""
        try:
            print(f"\n=== [{self.camera_name}] WebRTC 연결 시작 ===")
            print(f"[{self.camera_name}] 기존 트랙 수: {len(self.video_tracks)}")
            
            config = RTCConfiguration([
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun1.l.google.com:19302"])
            ])
            
            pc = RTCPeerConnection(configuration=config)
            print(f"[{self.camera_name}] PeerConnection 생성 성공")
            
            # 비디오 트랙 생성 및 추가
            video_track = self.CameraVideoTrack(self.camera_name)
            self.video_tracks.add(video_track)
            pc.addTrack(video_track)
            print(f"[{self.camera_name}] 비디오 트랙 추가됨 (ID: {id(video_track)})")
            print(f"[{self.camera_name}] 현재 트랙 수: {len(self.video_tracks)}")
            
            # 데이터 채널 생성
            channel = pc.createDataChannel("stats")
            
            @channel.on("open")
            def on_open():
                print(f"[{self.camera_name}] 데이터 채널 열림")
            
            # CameraVideoTrack에 데이터 채널 전달
            video_track.set_data_channel(channel)
            
            # 연결 상태 모니터링
            @pc.on("connectionstatechange")
            async def on_connectionstatechange():
                print(f"\n=== [{self.camera_name}] 연결 상태 변경: {pc.connectionState} ===")
                if pc.connectionState == "failed":
                    print(f"[{self.camera_name}] 연결 실패 - 트랙 정리")
                    self.video_tracks.discard(video_track)
                    video_track.stop()
                elif pc.connectionState == "connected":
                    print(f"[{self.camera_name}] 연결 성공")
                    print(f"[{self.camera_name}] 연결 후 트랙 수: {len(self.video_tracks)}")

            # ICE 연결 상태 모니터링 추가
            @pc.on("iceconnectionstatechange")
            async def on_iceconnectionstatechange():
                print(f"[{self.camera_name}] ICE 연결 상태: {pc.iceConnectionState}")

            # 트랙 이벤트 모니터링 추가
            @pc.on("track")
            def on_track(track):
                print(f"[{self.camera_name}] 트랙 이벤트 발생: {track.kind}")

            # Remote Description 설정
            await pc.setRemoteDescription(RTCSessionDescription(sdp=offer_sdp["sdp"], type=offer_sdp["type"]))
            print(f"[{self.camera_name}] Remote Description 설정 완료")
            
            # Answer 생성
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            print(f"[{self.camera_name}] Answer 생성 및 설정 완료")
            
            return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            
        except Exception as e:
            print(f"[{self.camera_name}] WebRTC 연결 에러: {str(e)}")
            import traceback
            traceback.print_exc()
            raise

    class CameraVideoTrack(VideoStreamTrack):
        def __init__(self, camera_name):
            super().__init__()
            self.camera_name = camera_name
            self.latest_frame = None
            self.frame_count = 0
            self.pts_time = 0
            self._started = True
            self._stopped = False
            self._start_time = time.time()  # 시작 시간 기록
            print(f"[{camera_name}] 비디오 트랙 초기화")
            
            # recording 관련 초기화는 선택적으로
            self.recording = True
            self.current_session = datetime.now().strftime(f"{camera_name}_%Y%m%d_%H%M%S")
            self.data_channel = None
            
            try:
                self.frame_dir = Path(settings.FRAME_STORAGE_PATH) / self.current_session
                self.frame_dir.mkdir(parents=True, exist_ok=True)
                
                with SessionLocal() as db:
                    video_record = Video(
                        session_id=self.current_session,
                        file_path=str(Path(settings.VIDEO_STORAGE_PATH) / f"{self.current_session}.mp4"),
                        camera_type=camera_name
                    )
                    db.add(video_record)
                    db.commit()
            except Exception as e:
                print(f"[{camera_name}] 레코딩 초기화 에러: {str(e)}")
                self.recording = False

        @property
        def stopped(self):
            """트랙이 중지되었는지 확인"""
            return self._stopped

        async def recv(self):
            """프레임 수신"""
            print(f"[{self.camera_name}] recv 호출됨, stopped: {self._stopped}")
            
            if self._stopped:
                print(f"[{self.camera_name}] 트랙이 stopped 상태")
                return None
                
            if self.latest_frame is None:
                print(f"[{self.camera_name}] 프레임 대기 중")
                await asyncio.sleep(0.1)
                return await self.recv()
                
            frame = self.latest_frame
            self.latest_frame = None
            return frame

        def set_data_channel(self, channel):
            self.data_channel = channel

        def update_frame(self, image_data):
            if self._stopped:
                return
                
            try:
                print(f"[{self.camera_name}] 프레임 업데이트 시작")
                
                # Base64 디코딩 및 이미지 처리
                if isinstance(image_data, str):
                    print(f"[{self.camera_name}] Base64 문자열 디코딩")
                    image_data = base64.b64decode(image_data)
                
                print(f"[{self.camera_name}] 이미지 데이터 크기: {len(image_data)} bytes")
                np_arr = np.frombuffer(image_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is None:
                    print(f"[{self.camera_name}] 이미지 디코딩 실패")
                    return
                    
                print(f"[{self.camera_name}] 이미지 크기: {frame.shape}")
                
                # VideoFrame 생성
                video_frame = av.VideoFrame.from_ndarray(frame, format='bgr24')
                video_frame.time_base = fractions.Fraction(1, 30)
                self.pts_time += 1
                video_frame.pts = self.pts_time
                
                self.latest_frame = video_frame
                self.frame_count += 1
                
                current_time = time.time()
                elapsed_time = current_time - self._start_time
                
                # FPS 계산 및 전송 (매 프레임마다)
                fps = min(30, self.frame_count / elapsed_time)  # 30fps로 제한
                if self.data_channel and self.data_channel.readyState == "open":
                    try:
                        self.data_channel.send(json.dumps({
                            "type": "stats",
                            "fps": round(fps, 1),
                            "frameCount": self.frame_count
                        }))
                        print(f"[{self.camera_name}] FPS 데이터 전송: {round(fps, 1)}")
                    except Exception as e:
                        print(f"[{self.camera_name}] FPS 데이터 전송 실패: {str(e)}")
                
                # 프레임 저장
                if self.recording:
                    frame_path = self.frame_dir / f"frame_{self.frame_count:06d}.jpg"
                    cv2.imwrite(str(frame_path), frame)
                    
            except Exception as e:
                print(f"[{self.camera_name}] 프레임 업데이트 에러: {str(e)}")
                import traceback
                traceback.print_exc()

        def stop(self):
            """트랙 중지"""
            if not self._stopped:
                print(f"[{self.camera_name}] 트랙 중지 시작")
                self._stopped = True
                if self.recording:
                    self.recording = False
                    self._convert_frames_to_video()
                    with SessionLocal() as db:
                        video_record = db.query(Video).filter_by(session_id=self.current_session).first()
                        if video_record:
                            video_record.end_time = datetime.utcnow()
                            video_record.frame_count = self.frame_count
                            video_record.status = "completed"
                            db.commit()
                print(f"[{self.camera_name}] 트랙 중지 완료")

        def _convert_frames_to_video(self):
            """프레임을 비디오로 변환"""
            try:
                output_path = Path(settings.VIDEO_STORAGE_PATH) / f"{self.current_session}.mp4"
                output_path.parent.mkdir(parents=True, exist_ok=True)
                
                cmd = [
                    'ffmpeg',
                    '-framerate', '30',
                    '-i', f'{self.frame_dir}/frame_%06d.jpg',
                    '-c:v', 'libx264',
                    '-pix_fmt', 'yuv420p',
                    str(output_path)
                ]
                subprocess.run(cmd, check=True)
                
            except Exception as e:
                print(f"비디오 변환 에러 ({self.camera_name}): {str(e)}")
                with SessionLocal() as db:
                    video_record = db.query(Video).filter_by(session_id=self.current_session).first()
                    if video_record:
                        video_record.status = "error"
                        db.commit() 