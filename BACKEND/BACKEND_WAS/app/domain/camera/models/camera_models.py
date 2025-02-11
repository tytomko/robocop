from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

class WebRTCOffer(BaseModel):
    sdp: str
    type: str

class WebRTCAnswer(BaseModel):
    sdp: str
    type: str

class CameraStatus(BaseModel):
    camera_name: str
    is_connected: bool
    connection_status: str

class RosConfig(BaseModel):
    host: str
    port: int
    topic: str

class VideoSession(BaseModel):
    session_id: str
    file_path: str
    camera_type: str
    created_at: datetime = datetime.now()
    status: str = "recording"  # recording, completed, failed
    metadata: Optional[Dict[str, Any]] = None

class IceCandidate(BaseModel):
    candidate: str
    sdpMLineIndex: int
    sdpMid: str