from enum import Enum

class UserRole(str, Enum):
    ADMIN = "admin"
    USER = "user"

class RobotStatus(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    CHARGING = "charging"
    ERROR = "error"
    EMERGENCY = "emergency"

class VideoStatus(str, Enum):
    RECORDING = "recording"
    COMPLETED = "completed"
    FAILED = "failed"

class ConnectionStatus(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    FAILED = "failed"

# 파일 관련 상수
ALLOWED_IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}
MAX_IMAGE_SIZE = 5 * 1024 * 1024  # 5MB

# 데이터베이스 관련 상수
DEFAULT_PAGE_SIZE = 10
MAX_PAGE_SIZE = 100

# ROS 관련 상수
ROS_BRIDGE_HOST = "192.168.100.104"
ROS_BRIDGE_PORT = 9090
ROS_RETRY_INTERVAL = 5.0
ROS_MAX_RETRIES = 3

# 웹소켓 관련 상수
WS_HEARTBEAT_INTERVAL = 30  # 초
WS_CLOSE_CODES = {
    1000: "정상 종료",
    1001: "엔드포인트 종료",
    1002: "프로토콜 오류",
    1003: "잘못된 데이터",
    1006: "비정상 종료",
    1007: "잘못된 메시지 형식",
    1008: "정책 위반",
    1009: "메시지가 너무 큼",
    1010: "확장 기능 누락",
    1011: "예기치 않은 오류",
    1015: "TLS 핸드셰이크 실패"
}