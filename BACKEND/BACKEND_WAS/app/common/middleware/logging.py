import time
import logging
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

logger = logging.getLogger(__name__)

class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """요청/응답 로깅 미들웨어"""
    def __init__(self, app: ASGIApp):
        super().__init__(app)

    async def dispatch(self, request: Request, call_next):
        start_time = time.time()
        
        # 요청 로깅
        logger.info(f"Request: {request.method} {request.url}")
        if request.query_params:
            logger.info(f"Query Params: {dict(request.query_params)}")
            
        try:
            # 요청 처리
            response = await call_next(request)
            
            # 응답 로깅
            process_time = (time.time() - start_time) * 1000
            formatted_process_time = '{0:.2f}'.format(process_time)
            logger.info(f"Response: {response.status_code} Completed in {formatted_process_time}ms")
            
            return response
            
        except Exception as e:
            # 에러 로깅
            logger.error(f"Request failed: {str(e)}")
            raise