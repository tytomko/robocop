from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from .domain.auth.controller.auth_controller import router as auth_router, create_admin_user
from .domain.robot.controller.robot_controller import router as robot_router
from .domain.camera.controller.camera_controller import router as camera_router
from .domain.lidar.controller.lidar_controller import router as lidar_router
from .domain.person.controller.person_controller import router as person_router
from .domain.ros_publisher.controller.ros_publisher_controller import router as ros_publisher_router
from .domain.map.controller.map_controller import router as map_router

from .common.config.manager import get_settings
from .common.exceptions.base import AppException
from .common.exceptions.handlers import (
    app_exception_handler,
    validation_exception_handler,
    internal_exception_handler
)
from .common.middleware.logging import RequestLoggingMiddleware
from .common.models.responses import BaseResponse
from .domain.lidar.controller.lidar_controller import start_lidar_subscriber
from .infrastructure.database.connection import DatabaseConnection
import uvicorn
import os
import logging
import asyncio
from fastapi import WebSocketDisconnect
from pathlib import Path
from dotenv import load_dotenv
from app.common.middleware.socket_service import start_socket_server
import threading
from .domain.robot.service.robot_service import ROS2WebSocketClient, RobotService
import time
from datetime import datetime
import json
import traceback


logger = logging.getLogger(__name__)
settings = get_settings()

# FastAPI 앱 생성
app = FastAPI(
    title="Robot Management System API",
    description="Robot Management System API",
    version="1.0.0",
    debug=settings.base.DEBUG
)

# CORS 미들웨어 설정
# origins = [
#     "https://robocopbackendssafy.duckdns.org",
#     "http://localhost:5173",
#     "*"  # 개발 중에는 모든 origin 허용
# ]

app.add_middleware(
    CORSMiddleware,
    # allow_origins=["https://frontend-web-one-omega.vercel.app"],  # 실제 운영 환경에서는 구체적인 origin으로 변경
    allow_origins=["*"],
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
app.include_router(robot_router, prefix="/api/v1/robots", tags=["robots"])
app.include_router(auth_router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(camera_router, prefix="/api/v1/cameras", tags=["cameras"])
app.include_router(lidar_router, prefix="/api/v1/lidar", tags=["lidar"])
app.include_router(person_router, prefix="/api/v1/persons", tags=["persons"])
app.include_router(ros_publisher_router, prefix="/api/v1", tags=["ros_publisher"])
app.include_router(map_router, prefix="/api/v1", tags=["map"])

# 프로젝트 루트 디렉토리 찾기
base_dir = Path(__file__).resolve().parent.parent

# .env 파일 경로 설정
env_path = os.path.join(base_dir, '.env')

# .env 파일 로드
load_dotenv(env_path)
# ros_bridge_client = ROS2WebSocketClient(
#     ros_host=settings.ros2.BRIDGE_HOST,
#     ros_port=settings.ros2.BRIDGE_PORT
# )
ros_bridge_client = ROS2WebSocketClient()
robot_service = RobotService()

@app.on_event("startup")
async def startup_event():
    logger.info("Starting up application...")
    
    try:
        # Database initialization
        await DatabaseConnection.connect()
        logger.info("Database connection successful")
        
        # DB connection test
        db = await DatabaseConnection.get_db()
        if db is None:
            logger.error("Unable to get database object")
            raise Exception("Database connection failed")
            
        # Simple query for connection test
        try:
            await db.command("ping")
            logger.info("Database connection test successful")
        except Exception as e:
            logger.error(f"Database connection test failed: {str(e)}")
            raise
            
        # Initialize counters collection
        try:
            counter = await db.counters.find_one({"_id": "robot_id"})
            if counter is None:
                await db.counters.insert_one({
                    "_id": "robot_id",
                    "seq": 0
                })
                logger.info("Robot ID sequence initialized")
        except Exception as e:
            logger.error(f"Sequence initialization failed: {str(e)}")
            raise

        # Initialize collection indexes
        await DatabaseConnection.init_collections()
        
        # Create admin account
        await create_admin_user()
        
        # Start lidar service
        try:
            asyncio.create_task(start_lidar_subscriber())
            logger.info("Lidar service started")
        except Exception as e:
            logger.warning(f"Failed to start lidar service: {str(e)}")
        
        # Start socket server
        threading.Thread(target=start_socket_server, daemon=True).start()
        logger.info("Socket server started")

        # Attempt ROS2 websocket connection
        try:
            await ros_bridge_client.connect()
            logger.info("ROS2 websocket connection successful")
        except Exception as e:
            logger.warning(f"ROS2 websocket connection failed: {str(e)}")
            logger.warning("Continuing server operation without ROS2 Bridge")
        
        logger.info("All initialization tasks completed")
    except Exception as e:
        logger.error(f"Error during application startup: {str(e)}")
        logger.warning("Some services failed to initialize, continuing server operation")

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Shutting down application...")
    await DatabaseConnection.disconnect()

@app.get("/", response_model=BaseResponse)
async def root():
    return BaseResponse(
        success=True,
        message="Welcome to the Robot Management System API",
        data={
            "version": "1.0.0",
            "environment": settings.base.ENV
        }
    )


# WebSocket 미들웨어 추가
@app.middleware("http")
async def websocket_middleware(request, call_next):
    if request.url.path.startswith('/ws'):
        # WebSocket 요청에 대한 CORS 헤더 추가
        response = await call_next(request)
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response
    return await call_next(request)

# WebSocket 연결 테스트용 엔드포인트
@app.websocket("/ws/test")
async def websocket_test(websocket: WebSocket):
    try:
        await websocket.accept()
        logger.info("New websocket client connected")
        
        await robot_service.register_frontend_client(websocket)
        await ros_bridge_client.start_listening()
        
        # 토픽 정의
        topics = {
            "/robot_1/utm_pose": "geometry_msgs/PoseStamped",
            "/robot_1/status": "robot_custom_interfaces/msg/Status",
        }

        # 구독 및 브로드캐스트 루프
        while True:
            try:
                for topic_name, msg_type in topics.items():
                    # 토픽 구독 및 메시지 수신
                    message = await ros_bridge_client.subscribe_to_topic(topic_name, msg_type)
                    logger.info(f"Got message from subscribe_to_topic: {message}")
                    
                    # 메시지가 있으면 프론트엔드로 전송
                    if message:
                        logger.info("Broadcasting message to frontend")
                        await robot_service.broadcast_to_frontend(message)
                        logger.info("Broadcast completed")
                    else:
                        logger.info(f"No message to broadcast for topic: {topic_name}")
                        
                await asyncio.sleep(1)  # 1초 대기
                
            except Exception as e:
                logger.error(f"Loop error: {str(e)}")
                logger.error(f"Error traceback: {traceback.format_exc()}")
                await asyncio.sleep(1)
                
    except WebSocketDisconnect:
        logger.info("Websocket client disconnected")
        await robot_service.unregister_frontend_client(websocket)


if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port, reload=False) 
