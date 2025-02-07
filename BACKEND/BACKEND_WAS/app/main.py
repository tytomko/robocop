from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .domain.auth.controller.auth_controller import router as auth_router, create_admin_user
from .domain.robot.controller.robot_controller import router as robot_router
from .domain.camera.controller.camera_controller import router as camera_router, initialize_camera
from .domain.lidar.controller.lidar_controller import router as lidar_router
from .domain.person.controller.person_controller import router as person_router
from .domain.ros_publisher.controller.ros_publisher_controller import router as ros_publisher_router

from .common.config.manager import get_settings
from .common.exceptions.base import AppException
from .common.exceptions.handlers import (
    app_exception_handler,
    validation_exception_handler,
    internal_exception_handler
)
from .common.middleware.logging import RequestLoggingMiddleware
from .common.models.responses import BaseResponse
from .domain.camera.models.camera_models import CameraConfig
from .domain.lidar.controller.lidar_controller import start_lidar_subscriber
from .infrastructure.database.connection import DatabaseConnection
import uvicorn
import os
import logging
import asyncio

logger = logging.getLogger(__name__)
settings = get_settings()

# FastAPI 앱 생성
app = FastAPI(
    title="Robot Management System API",
    description="로봇 관리 시스템 API",
    version="1.0.0",
    debug=settings.base.DEBUG
)

# CORS 미들웨어 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 실제 운영 환경에서는 구체적인 origin으로 변경
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 로깅 미들웨어 추가
app.add_middleware(RequestLoggingMiddleware)

# 예외 핸들러 등록
app.add_exception_handler(AppException, app_exception_handler)
app.add_exception_handler(ValueError, validation_exception_handler)
app.add_exception_handler(Exception, internal_exception_handler)

# 라우터 등록
app.include_router(
    robot_router,
    prefix="/api/v1/robots",
    tags=["robots"]
)

app.include_router(
    auth_router,
    prefix="/api/v1/auth",
    tags=["auth"]
)

app.include_router(camera_router, prefix="/api/v1/cameras", tags=["cameras"])
app.include_router(lidar_router, prefix="/api/v1/lidar", tags=["lidar"])
app.include_router(person_router, prefix="/api/v1/persons", tags=["persons"])
app.include_router(ros_publisher_router, tags=["ros_publisher"]) # 0206 test

@app.on_event("startup")
async def startup_event():
    """애플리케이션 시작 시 실행되는 이벤트 핸들러"""
    logger.info("Starting up application...")
    
    try:
        # 데이터베이스 초기화
        await DatabaseConnection.connect()
        logger.info("데이터베이스 연결 성공")
        
        # DB 연결 테스트
        db = await DatabaseConnection.get_db()
        if db is None:
            logger.error("데이터베이스 객체를 가져올 수 없습니다.")
            raise Exception("데이터베이스 연결 실패")
            
        # 연결 테스트를 위한 간단한 쿼리 실행
        try:
            await db.command("ping")
            logger.info("데이터베이스 연결 테스트 성공")
        except Exception as e:
            logger.error(f"데이터베이스 연결 테스트 실패: {str(e)}")
            raise
            
        # counters 컬렉션 초기화
        try:
            counter = await db.counters.find_one({"_id": "robot_id"})
            if counter is None:
                await db.counters.insert_one({
                    "_id": "robot_id",
                    "seq": 0
                })
                logger.info("Robot ID 시퀀스가 초기화되었습니다.")
        except Exception as e:
            logger.error(f"시퀀스 초기화 실패: {str(e)}")
            raise

        find_person = db.persons.find_one({"name": "신동욱"})
        print(find_person)
        # 컬렉션 인덱스 초기화
        await DatabaseConnection.init_collections()
        
        # 관리자 계정 생성
        await create_admin_user()
        
        # 라이다 서비스 시작
        # try:
        #     asyncio.create_task(start_lidar_subscriber())
        #     logger.info("라이다 서비스가 시작되었습니다.")
        # except Exception as e:
        #     logger.warning(f"라이다 서비스 시작 실패: {str(e)}")
        
        # 카메라 초기화
        # camera_configs = [
        #     CameraConfig(
        #         camera_topic='/ssafy/tb3_front_camera/image_raw/compressed',
        #         camera_name='robot1_front',
        #         ros_bridge_host='172.30.1.10',
        #         ros_bridge_port=9090
        #     ),
        #     CameraConfig(
        #         camera_topic='/ssafy/tb3_rear_camera/image_raw/compressed',
        #         camera_name='robot1_rear',
        #         ros_bridge_host='172.30.1.10',
        #         ros_bridge_port=9090
        #     )
        # ]
        
        # for config in camera_configs:
        #     try:
        #         success = await initialize_camera(config)
        #         if success:
        #             logger.info(f"카메라 초기화 성공: {config.camera_name}")
        #         else:
        #             logger.warning(f"카메라 초기화 실패: {config.camera_name}")
        #     except Exception as e:
        #         logger.warning(f"카메라 초기화 실패 ({config.camera_name}): {str(e)}")
        #         logger.warning("카메라 기능이 비활성화된 상태로 실행됩니다.")

        # topic publish 작업 (test)
        

        logger.info("모든 초기화 작업이 완료되었습니다.")
    except Exception as e:
        logger.error(f"애플리케이션 시작 중 오류 발생: {str(e)}")
        raise e

@app.on_event("shutdown")
async def shutdown_event():
    """애플리케이션 종료 시 실행되는 이벤트 핸들러"""
    logger.info("Shutting down application...")
    await DatabaseConnection.disconnect()

@app.get("/", response_model=BaseResponse)
async def root():
    """루트 엔드포인트"""
    return BaseResponse(
        success=True,
        message="Welcome to the Robot Management System API",
        data={
            "version": "1.0.0",
            "environment": settings.base.ENV
        }
    )

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port, reload=False) 