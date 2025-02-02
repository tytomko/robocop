from typing import Generic, TypeVar, Optional, List
from pydantic import BaseModel
from datetime import datetime

T = TypeVar('T')

class ErrorDetail(BaseModel):
    """에러 상세 정보 모델"""
    field: str
    message: str

class PaginationInfo(BaseModel):
    """페이지네이션 정보 모델"""
    page: int
    size: int
    total: int
    totalPages: int

class BaseResponse(BaseModel, Generic[T]):
    """기본 응답 모델"""
    success: bool = True
    status: int = 200
    message: Optional[str] = None
    data: Optional[T] = None
    errors: Optional[List[ErrorDetail]] = None
    pagination: Optional[PaginationInfo] = None
    timestamp: datetime = datetime.now()

    class Config:
        json_schema_extra = {
            "example": {
                "success": True,
                "status": 200,
                "message": "요청이 성공적으로 처리되었습니다",
                "data": None,
                "errors": None,
                "pagination": {
                    "page": 1,
                    "size": 10,
                    "total": 100,
                    "totalPages": 10
                },
                "timestamp": "2024-01-20T00:00:00"
            }
        }