class ROS2Settings(BaseSettings):
    BRIDGE_URL: str = "ws://127.0.0.1:10000"
    RETRY_INTERVAL: int = 5  # 재연결 시도 간격 (초)
    MAX_RETRIES: int = 3     # 최대 재연결 시도 횟수

class Settings(BaseSettings):
    # ... 기존 설정들 ...
    ros2: ROS2Settings = ROS2Settings() 