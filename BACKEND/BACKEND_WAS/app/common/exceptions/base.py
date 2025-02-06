from fastapi import HTTPException
from typing import Any, Dict, Optional

class AppException(HTTPException):
    """애플리케이션 기본 예외"""
    def __init__(
        self,
        status_code: int,
        message: str,
        detail: Optional[Any] = None,
        headers: Optional[Dict[str, str]] = None
    ):
        super().__init__(status_code=status_code, detail=detail)
        self.message = message
        self.headers = headers

class ValidationError(AppException):
    """유효성 검사 예외"""
    def __init__(self, message: str, detail: Optional[Any] = None):
        super().__init__(
            status_code=400,
            message=message,
            detail=detail
        )

class AuthenticationError(AppException):
    """인증 예외"""
    def __init__(self, message: str = "인증에 실패했습니다"):
        super().__init__(
            status_code=401,
            message=message,
            headers={"WWW-Authenticate": "Bearer"}
        )

class AuthorizationError(AppException):
    """권한 예외"""
    def __init__(self, message: str = "권한이 없습니다"):
        super().__init__(
            status_code=403,
            message=message
        )

class NotFoundError(AppException):
    """리소스를 찾을 수 없는 예외"""
    def __init__(self, message: str = "리소스를 찾을 수 없습니다"):
        super().__init__(
            status_code=404,
            message=message
        )

class ConflictError(AppException):
    """리소스 충돌 예외"""
    def __init__(self, message: str = "리소스가 이미 존재합니다"):
        super().__init__(
            status_code=409,
            message=message
        )

class DatabaseError(AppException):
    """데이터베이스 예외"""
    def __init__(self, message: str = "데이터베이스 오류가 발생했습니다"):
        super().__init__(
            status_code=500,
            message=message
        )