class ROS2Settings(BaseSettings):
<<<<<<< HEAD
    BRIDGE_URL: str = "ws://127.0.0.1:10000"
=======
    BRIDGE_URL: str = "ws://192.168.100.104:9090"
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
    RETRY_INTERVAL: int = 5  # 재연결 시도 간격 (초)
    MAX_RETRIES: int = 3     # 최대 재연결 시도 횟수

class Settings(BaseSettings):
    # ... 기존 설정들 ...
    ros2: ROS2Settings = ROS2Settings() 