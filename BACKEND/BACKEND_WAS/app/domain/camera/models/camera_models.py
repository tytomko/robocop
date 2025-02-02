from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

class CameraConfig(BaseModel):
    camera_topic: str
    camera_name: str = "robot1"
    ros_bridge_host: str = "192.168.100.104"
    ros_bridge_port: int = 9090

class WebRTCOffer(BaseModel):
    sdp: str
    type: str

class WebRTCAnswer(BaseModel):
    sdp: str
    type: str

class VideoSession(BaseModel):
    session_id: str
    file_path: str
    camera_type: str
    created_at: datetime = datetime.now()
    status: str = "recording"  # recording, completed, failed
    metadata: Optional[Dict[str, Any]] = None

class CameraStatus(BaseModel):
    camera_name: str
    is_connected: bool
    last_frame_time: Optional[datetime] = None
    active_tracks: int = 0
    connection_status: str = "disconnected"  # disconnected, connecting, connected, failed
    error_message: Optional[str] = None