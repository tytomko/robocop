from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import roslibpy
import cv2
import base64
import numpy as np
import time
import logging

app = FastAPI()

logger = logging.getLogger(__name__)

class CameraService:
    def __init__(self):
        self.latest_frame = None
        self.fps = 30
        self.last_frame_time = 0
        self.jpeg_quality = 80
        self.frame_size = (640, 480)
        
        # 기본 ROS 클라이언트 설정 (일반 로봇용)
        self.client = roslibpy.Ros(host='127.0.0.1', port=10000)
        self.topic = None
        
        # Isaac 로봇용 별도 클라이언트
        self.isaac_client = roslibpy.Ros(host='127.0.0.1', port=10001)
        
    def set_robot_connection(self, is_isaac=False):
        """로봇 타입에 따라 적절한 ROS 클라이언트 선택"""
        if is_isaac:
            if not self.isaac_client.is_connected:
                self.isaac_client.run()
            return self.isaac_client
        else:
            if not self.client.is_connected:
                self.client.run()
            return self.client

    def set_topic(self, topic_name: str, topic_type: str, is_isaac=False):
        """토픽 설정 - 로봇 타입에 따라 다른 클라이언트 사용"""
        client = self.set_robot_connection(is_isaac)
        
        # 기존 토픽이 있다면 구독 해제
        if self.topic:
            try:
                self.topic.unsubscribe()
            except:
                pass
        
        # 새로운 토픽 설정
        self.topic = roslibpy.Topic(
            client,
            topic_name,
            topic_type
        )
        self.topic.subscribe(self._on_image_message)

    def _on_image_message(self, message):
        """이미지 메시지 수신 처리"""
        try:
            # 메시지 타입에 따른 처리
            format_str = message.get('format', '')
            
            if 'compressed' in format_str.lower():  # CompressedImage 타입 (jpeg 압축된 경우)
                image_data = base64.b64decode(message['data'])
                np_arr = np.frombuffer(image_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
            elif 'rgb8' in format_str.lower() or 'bgr8' in format_str.lower():  # 일반 Image 타입
                # ROS Image 메시지를 numpy 배열로 변환
                width = message.get('width', 0)
                height = message.get('height', 0)
                
                # 이미지 데이터를 numpy 배열로 변환
                np_arr = np.frombuffer(base64.b64decode(message['data']), np.uint8)
                
                if 'rgb8' in format_str.lower():
                    frame = np_arr.reshape(height, width, 3)
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:  # bgr8
                    frame = np_arr.reshape(height, width, 3)
            else:
                logger.error(f"Unsupported format: {format_str}")
                return
            
            # 프레임이 유효한지 확인
            if frame is not None and frame.size > 0:
                # 프레임 크기 조정
                frame = cv2.resize(frame, self.frame_size)
                self.latest_frame = frame
            else:
                logger.error("Invalid frame received")
            
        except Exception as e:
            logger.error(f"이미지 처리 에러: {str(e)}")
            logger.error(f"Message content: {message}")
            logger.error(f"Format string: {format_str}")

    def get_frame(self):
        """현재 프레임 스트리밍"""
        while True:
            current_time = time.time()
            
            # FPS 제한
            if current_time - self.last_frame_time < 1.0/self.fps:
                time.sleep(0.001)  # CPU 사용량 감소
                continue
                
            if self.latest_frame is not None:
                # JPEG 압축 품질 설정
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                _, buffer = cv2.imencode('.jpg', self.latest_frame, encode_param)
                frame_bytes = buffer.tobytes()
                
                self.last_frame_time = current_time
                
                yield (b'--frame\r\n'
                      b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# 싱글톤 인스턴스 생성
camera_service = CameraService()

# 초기 연결 시도 제거 (실제 요청이 올 때 연결하도록)

# @app.get("/video_feed")
# async def video_feed():
#     return StreamingResponse(
#         get_frame(),
#         media_type="multipart/x-mixed-replace; boundary=frame"
#     )