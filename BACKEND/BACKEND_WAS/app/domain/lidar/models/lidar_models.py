from pydantic import BaseModel
from typing import List, Dict, Optional
from datetime import datetime

class Point(BaseModel):
    x: float
    y: float
    z: float
    intensity: float

class LidarConfig(BaseModel):
    ros_bridge_host: str = "127.0.0.1"
    ros_bridge_port: int = 10000
    topic_name: str = "/ssafy/velodyne_points"
    message_type: str = "sensor_msgs/PointCloud2"
    max_points: int = 10000
    update_interval: float = 1.0  # 초단위

class LidarData(BaseModel):
    type: str = "lidar_points"
    points: List[Point]
    timestamp: float

class LidarStatus(BaseModel):
    is_connected: bool = False
    last_update: Optional[datetime] = None
    client_count: int = 0
    points_count: int = 0
    error_message: Optional[str] = None