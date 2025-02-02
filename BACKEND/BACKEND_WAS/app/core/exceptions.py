from typing import Any, Dict, Optional

class AppException(Exception):
    """애플리케이션 기본 예외"""
    def __init__(
        self,
        message: str,
        status_code: int = 500,
        error_code: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None
    ):
        self.message = message
        self.status_code = status_code
        self.error_code = error_code
        self.details = details
        super().__init__(self.message)

class ValidationError(AppException):
    """유효성 검사 예외"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            message=message,
            status_code=400,
            error_code="VALIDATION_ERROR",
            details=details
        )

class AuthenticationError(AppException):
    """인증 예외"""
    def __init__(self, message: str):
        super().__init__(
            message=message,
            status_code=401,
            error_code="AUTHENTICATION_ERROR"
        )

class AuthorizationError(AppException):
    """권한 예외"""
    def __init__(self, message: str):
        super().__init__(
            message=message,
            status_code=403,
            error_code="AUTHORIZATION_ERROR"
        )

class NotFoundError(AppException):
    """리소스 없음 예외"""
    def __init__(self, message: str):
        super().__init__(
            message=message,
            status_code=404,
            error_code="NOT_FOUND_ERROR"
        )

class ConflictError(AppException):
    """리소스 충돌 예외"""
    def __init__(self, message: str):
        super().__init__(
            message=message,
            status_code=409,
            error_code="CONFLICT_ERROR"
        )

class DatabaseError(AppException):
    """데이터베이스 예외"""
    def __init__(self, message: str):
        super().__init__(
            message=message,
            status_code=500,
            error_code="DATABASE_ERROR"
        )

class RobotConnectionError(AppException):
    """로봇 연결 예외"""
    def __init__(self, message: str):
        super().__init__(
            message=message,
            status_code=503,
            error_code="ROBOT_CONNECTION_ERROR"
        ) 