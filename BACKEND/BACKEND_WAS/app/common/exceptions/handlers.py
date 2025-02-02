from fastapi import Request
from fastapi.responses import JSONResponse
from .base import AppException
from ..models.responses import BaseResponse, ErrorDetail
from typing import Union
import traceback
import logging

logger = logging.getLogger(__name__)

async def app_exception_handler(request: Request, exc: AppException) -> JSONResponse:
    """애플리케이션 예외 핸들러"""
    logger.error(f"애플리케이션 예외 발생: {exc.message}")
    if exc.detail:
        logger.error(f"상세 정보: {exc.detail}")
    
    return JSONResponse(
        status_code=exc.status_code,
        content=BaseResponse(
            success=False,
            status=exc.status_code,
            message=exc.message,
            errors=[ErrorDetail(field="", message=str(exc.detail))] if exc.detail else None
        ).dict()
    )

async def validation_exception_handler(request: Request, exc: Union[ValueError, TypeError]) -> JSONResponse:
    """유효성 검사 예외 핸들러"""
    logger.error(f"유효성 검사 예외 발생: {str(exc)}")
    
    return JSONResponse(
        status_code=400,
        content=BaseResponse(
            success=False,
            status=400,
            message="유효성 검사 오류가 발생했습니다",
            errors=[ErrorDetail(field="", message=str(exc))]
        ).dict()
    )

async def internal_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """내부 서버 예외 핸들러"""
    logger.error(f"내부 서버 오류 발생: {str(exc)}")
    logger.error(traceback.format_exc())
    
    return JSONResponse(
        status_code=500,
        content=BaseResponse(
            success=False,
            status=500,
            message="내부 서버 오류가 발생했습니다",
            errors=[ErrorDetail(field="", message="서버 내부 오류가 발생했습니다")]
        ).dict()
    )