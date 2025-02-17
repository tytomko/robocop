from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import roslibpy
import cv2
import base64
import numpy as np
import time
import logging

logger = logging.getLogger(__name__)

class CameraService:
    def __init__(self):
        self.front_frame = None
        self.rear_frame = None
        self.fps = 30
        self.last_front_frame_time = 0
        self.last_rear_frame_time = 0
        self.jpeg_quality = 80
        self.frame_size = (640, 480)
        
        # 기본 ROS 클라이언트 설정
        self.client = roslibpy.Ros(host='127.0.0.1', port=10000)
        self.isaac_client = roslibpy.Ros(host='127.0.0.1', port=10001)
        
        self.front_topic = None
        self.rear_topic = None
        
    def set_robot_connection(self, is_isaac=False):
        if is_isaac:
            if not self.isaac_client.is_connected:
                self.isaac_client.run()
            return self.isaac_client
        else:
            if not self.client.is_connected:
                self.client.run()
            return self.client

    def set_front_topic(self, topic_name: str, topic_type: str, is_isaac=False):
        client = self.set_robot_connection(is_isaac)
        
        if self.front_topic:
            try:
                self.front_topic.unsubscribe()
            except:
                pass
        
        self.front_topic = roslibpy.Topic(
            client,
            topic_name,
            topic_type
        )
        self.front_topic.subscribe(self._on_front_image_message)

    def set_rear_topic(self, topic_name: str, topic_type: str, is_isaac=False):
        client = self.set_robot_connection(is_isaac)
        
        if self.rear_topic:
            try:
                self.rear_topic.unsubscribe()
            except:
                pass
        
        self.rear_topic = roslibpy.Topic(
            client,
            topic_name,
            topic_type
        )
        self.rear_topic.subscribe(self._on_rear_image_message)

    def _process_image_message(self, message):
        try:
            format_str = message.get('format', '')
            
            if 'compressed' in format_str.lower():
                image_data = base64.b64decode(message['data'])
                np_arr = np.frombuffer(image_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
            elif 'rgb8' in format_str.lower() or 'bgr8' in format_str.lower():
                width = message.get('width', 0)
                height = message.get('height', 0)
                np_arr = np.frombuffer(base64.b64decode(message['data']), np.uint8)
                
                if 'rgb8' in format_str.lower():
                    frame = np_arr.reshape(height, width, 3)
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:
                    frame = np_arr.reshape(height, width, 3)
            else:
                logger.error(f"Unsupported format: {format_str}")
                return None
            
            if frame is not None and frame.size > 0:
                return cv2.resize(frame, self.frame_size)
            return None
            
        except Exception as e:
            logger.error(f"이미지 처리 에러: {str(e)}")
            return None

    def _on_front_image_message(self, message):
        frame = self._process_image_message(message)
        if frame is not None:
            self.front_frame = frame

    def _on_rear_image_message(self, message):
        frame = self._process_image_message(message)
        if frame is not None:
            self.rear_frame = frame

    def get_front_frame(self):
        while True:
            current_time = time.time()
            
            if current_time - self.last_front_frame_time < 1.0/self.fps:
                time.sleep(0.001)
                continue
                
            if self.front_frame is not None:
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                _, buffer = cv2.imencode('.jpg', self.front_frame, encode_param)
                frame_bytes = buffer.tobytes()
                
                self.last_front_frame_time = current_time
                
                yield (b'--frame\r\n'
                      b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    def get_rear_frame(self):
        while True:
            current_time = time.time()
            
            if current_time - self.last_rear_frame_time < 1.0/self.fps:
                time.sleep(0.001)
                continue
                
            if self.rear_frame is not None:
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                _, buffer = cv2.imencode('.jpg', self.rear_frame, encode_param)
                frame_bytes = buffer.tobytes()
                
                self.last_rear_frame_time = current_time
                
                yield (b'--frame\r\n'
                      b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

camera_service = CameraService()
