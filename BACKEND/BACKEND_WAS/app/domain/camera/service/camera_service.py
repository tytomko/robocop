from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import roslibpy
from ...ros_publisher.service.ros_bridge_connection import RosBridgeConnection
import cv2
import base64
import numpy as np
import time
import logging
import asyncio
from typing import Optional, Dict
import time

logger = logging.getLogger(__name__)

class CameraService:
    _instances: Dict[int, 'CameraService'] = {}

    @classmethod
    async def get_instance(cls, seq: int) -> 'CameraService':
        """??? seq? ?? CameraService ????? ?????."""
        if seq not in cls._instances:
            instance = cls()
            await instance.initialize(seq)
            cls._instances[seq] = instance
        return cls._instances[seq]

    def __init__(self):
        """CameraService ????? ??????."""
        self.seq: Optional[int] = None
        self.front_frame: Optional[np.ndarray] = None
        self.rear_frame: Optional[np.ndarray] = None
        self.fps: int = 30
        self.last_front_frame_time: float = 0
        self.last_rear_frame_time: float = 0
        self.jpeg_quality: int = 80
        self.frame_size: tuple = (640, 480)
        
        self.ros_bridge: Optional[RosBridgeConnection] = None
        self.isaac_bridge: Optional[RosBridgeConnection] = None
        self.client_id: Optional[str] = None
        
        self.front_topic: Optional[roslibpy.Topic] = None
        self.rear_topic: Optional[roslibpy.Topic] = None

    async def initialize(self, seq: int) -> None:
        """??? ????? ??????."""
        try:
            self.seq = seq
            self.client_id = f"camera_service_{seq}"
            
            # ROS Bridge ?? ???
            self.ros_bridge = RosBridgeConnection(port=10000)
            
            # Isaac ??? ???? isaac_bridge ???
            if seq == 11:
                self.isaac_bridge = RosBridgeConnection(port=10001)
                self.isaac_bridge.register_client(self.client_id)
                logger.info(f"Isaac Bridge ????? ?? ?? (seq: {seq})")
            else:
                self.isaac_bridge = None
            
            # ?? ROS Bridge ????? ??
            self.ros_bridge.register_client(self.client_id)
            logger.info(f"ROS Bridge ????? ?? ?? (seq: {seq})")
            
        except Exception as e:
            logger.error(f"Camera service initialization failed: {e}")
            raise

    async def cleanup(self) -> None:
        """??? ????? ???? ?????."""
        try:
            if self.front_topic:
                self.front_topic.unsubscribe()
            if self.rear_topic:
                self.rear_topic.unsubscribe()
            
            if self.ros_bridge:
                await self.ros_bridge.unregister_client(self.client_id)
            
            # Isaac ??? ???? isaac_bridge ??
            if self.isaac_bridge:
                await self.isaac_bridge.unregister_client(self.client_id)
                
        except Exception as e:
            logger.error(f"Cleanup error for camera service {self.seq}: {e}")

    @classmethod
    async def remove_instance(cls, seq: int) -> None:
        """????? ???? ???? ?????."""
        if seq in cls._instances:
            instance = cls._instances[seq]
            await instance.cleanup()
            del cls._instances[seq]

    def set_robot_connection(self, is_isaac: bool = False) -> roslibpy.Ros:
        """??? ROS Bridge ????? ?????."""
        if is_isaac:
            return self.isaac_bridge.client
        return self.ros_bridge.client

    async def set_front_topic(self, topic_name: str, topic_type: str, is_isaac: bool = False) -> None:
        """?? ??? ??? ?????."""
        try:
            client = self.set_robot_connection(is_isaac)
            
            if self.front_topic:
                try:
                    self.front_topic.unsubscribe()
                except Exception as e:
                    logger.warning(f"Failed to unsubscribe from front topic: {e}")
            
            self.front_topic = roslibpy.Topic(
                client,
                topic_name,
                topic_type
            )
            self.front_topic.subscribe(self._on_front_image_message)
        except Exception as e:
            logger.error(f"Error setting front topic: {e}")
            raise

    async def set_rear_topic(self, topic_name: str, topic_type: str, is_isaac: bool = False) -> None:
        """?? ??? ??? ?????."""
        try:
            client = self.set_robot_connection(is_isaac)
            
            if self.rear_topic:
                try:
                    self.rear_topic.unsubscribe()
                except Exception as e:
                    logger.warning(f"Failed to unsubscribe from rear topic: {e}")
            
            self.rear_topic = roslibpy.Topic(
                client,
                topic_name,
                topic_type
            )
            self.rear_topic.subscribe(self._on_rear_image_message)
        except Exception as e:
            logger.error(f"Error setting rear topic: {e}")
            raise

    def _process_image_message(self, message: dict) -> Optional[np.ndarray]:
        """??? ???? ???? ????? ?????."""
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
            logger.error(f"??? ?? ??: {str(e)}")
            return None

    def _on_front_image_message(self, message: dict) -> None:
        """?? ??? ??? ???? ?????."""
        frame = self._process_image_message(message)
        if frame is not None:
            self.front_frame = frame

    def _on_rear_image_message(self, message: dict) -> None:
        """?? ??? ??? ???? ?????."""
        frame = self._process_image_message(message)
        if frame is not None:
            self.rear_frame = frame

    def get_front_frame(self):
        """?? ??? ??? ???? ?????."""
        while True:
            current_time = time.time()
            
            if current_time - self.last_front_frame_time < 1.0/self.fps:
                time.sleep(0.001)
                continue
                
            if self.front_frame is not None:
                try:
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    _, buffer = cv2.imencode('.jpg', self.front_frame, encode_param)
                    frame_bytes = buffer.tobytes()
                    
                    self.last_front_frame_time = current_time
                    
                    yield (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                except Exception as e:
                    logger.error(f"Error in front frame streaming: {e}")
                    time.sleep(0.1)

    def get_rear_frame(self):
        """?? ??? ??? ???? ?????."""
        while True:
            current_time = time.time()
            
            if current_time - self.last_rear_frame_time < 1.0/self.fps:
                time.sleep(0.001)
                continue
                
            if self.rear_frame is not None:
                try:
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    _, buffer = cv2.imencode('.jpg', self.rear_frame, encode_param)
                    frame_bytes = buffer.tobytes()
                    
                    self.last_rear_frame_time = current_time
                    
                    yield (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                except Exception as e:
                    logger.error(f"Error in rear frame streaming: {e}")
                    time.sleep(0.1)

    async def __aenter__(self):
        """??? ???? ??? ??"""
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """??? ???? ??? ??"""
        await self.cleanup()
