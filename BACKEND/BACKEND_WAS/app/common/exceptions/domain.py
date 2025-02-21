from .base import AppException

# Auth 도메인 예외
class InvalidCredentialsError(AppException):
    """잘못된 인증 정보 예외"""
    def __init__(self, message: str = "아이디 또는 비밀번호가 잘못되었습니다"):
        super().__init__(status_code=401, message=message)

class TokenExpiredError(AppException):
    """토큰 만료 예외"""
    def __init__(self, message: str = "토큰이 만료되었습니다"):
        super().__init__(status_code=401, message=message)

# Robot 도메인 예외
class RobotConnectionError(AppException):
    """로봇 연결 예외"""
    def __init__(self, message: str = "로봇과의 연결에 실패했습니다"):
        super().__init__(status_code=503, message=message)

class RobotOperationError(AppException):
    """로봇 작업 예외"""
    def __init__(self, message: str = "로봇 작업 수행 중 오류가 발생했습니다"):
        super().__init__(status_code=500, message=message)

# Camera 도메인 예외
class CameraConnectionError(AppException):
    """카메라 연결 예외"""
    def __init__(self, message: str = "카메라와의 연결에 실패했습니다"):
        super().__init__(status_code=503, message=message)

class VideoStreamError(AppException):
    """비디오 스트림 예외"""
    def __init__(self, message: str = "비디오 스트림 처리 중 오류가 발생했습니다"):
        super().__init__(status_code=500, message=message)

# Lidar 도메인 예외
class LidarConnectionError(AppException):
    """라이다 연결 예외"""
    def __init__(self, message: str = "라이다와의 연결에 실패했습니다"):
        super().__init__(status_code=503, message=message)

class PointCloudError(AppException):
    """포인트 클라우드 처리 예외"""
    def __init__(self, message: str = "포인트 클라우드 처리 중 오류가 발생했습니다"):
        super().__init__(status_code=500, message=message)

# Person 도메인 예외
class PersonImageError(AppException):
    """사용자 이미지 처리 예외"""
    def __init__(self, message: str = "이미지 처리 중 오류가 발생했습니다"):
        super().__init__(status_code=400, message=message)

class PersonNotFoundError(AppException):
    """사용자를 찾을 수 없는 예외"""
    def __init__(self, message: str = "해당 사용자를 찾을 수 없습니다"):
        super().__init__(status_code=404, message=message)