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
from .domain.robot.service.robot_service import RobotService
import time
from datetime import datetime
import json
import traceback
from app.domain.ros_publisher.service.ros_bridge_connection import RosBridgeConnection
import roslibpy


logger = logging.getLogger(__name__)
settings = get_settings()

# FastAPI ?? ????
app = FastAPI(
    title="Robot Management System API",
    description="Robot Management System API",
    version="1.0.0",
    debug=settings.base.DEBUG
)

origins = [
    "https://robocopbackendssafy.duckdns.org/",
    "http://52.79.51.253",
]

# CORS ???? ??
app.add_middleware(
    CORSMiddleware,
    #allow_origins=[origins], # ?? ?? ??? ?? ??
    allow_origins=["*"], # ?? ????
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
# ?a? ?????? ???
app.add_middleware(RequestLoggingMiddleware)

# ???? ??? ???
app.add_exception_handler(AppException, app_exception_handler)
app.add_exception_handler(ValueError, validation_exception_handler)
app.add_exception_handler(Exception, internal_exception_handler)

# ????? ???
app.include_router(robot_router, prefix="/api/v1/robots", tags=["robots"])
app.include_router(auth_router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(camera_router, prefix="/api/v1/cameras", tags=["cameras"])
app.include_router(lidar_router, prefix="/api/v1/lidar", tags=["lidar"])
app.include_router(person_router, prefix="/api/v1/persons", tags=["persons"])
app.include_router(ros_publisher_router, prefix="/api/v1", tags=["ros_publisher"])
app.include_router(map_router, prefix="/api/v1", tags=["map"])

# ??????T ??T ???? ???
base_dir = Path(__file__).resolve().parent.parent

# .env ???? ??? ????
env_path = os.path.join(base_dir, '.env')

# .env ???? ?e?
load_dotenv(env_path)
# ros_bridge_client = ROS2WebSocketClient(
#     ros_host=settings.ros2.BRIDGE_HOST,
#     ros_port=settings.ros2.BRIDGE_PORT
# )
# ROS2WebSocketClient는 더 이상 사용하지 않음
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

        # ROS Bridge 연결 시도 (실패해도 서버는 계속 동작)
        try:
            _ = RosBridgeConnection()  # 변수 할당 없이 초기화만
            logger.info("ROS Bridge connection attempted")
        except Exception as e:
            logger.warning(f"ROS Bridge connection failed: {str(e)}")
            logger.warning("Server will continue running without ROS Bridge")

        logger.info("All initialization tasks completed")
    except Exception as e:
        logger.error(f"Error during application startup: {str(e)}")
        logger.error(traceback.format_exc())
        raise  # DB 연결 실패 등 중요한 에러는 서버 시작을 중단

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


# WebSocket ?????? ???
@app.middleware("http")
async def websocket_middleware(request, call_next):
    if request.url.path.startswith('/ws'):
        # WebSocket ????? ???? CORS ??? ???
        response = await call_next(request)
        response.headers['Access-Control-Allow-Origin'] = '*'
        return response
    return await call_next(request)

@app.websocket("/ws/test")
async def websocket_test(websocket: WebSocket):
    ros_bridge = None
    client_id = f"ws_test_{id(websocket)}"
    active_topics = {}  # 활성 토픽 추적을 위한 딕셔너리
    
    try:
        await websocket.accept()
        logger.info("New websocket client connected")
        await robot_service.register_frontend_client(websocket)
        
        # ROS Bridge 연결
        ros_bridge = RosBridgeConnection()
        if not ros_bridge.client or not ros_bridge.client.is_connected:
            await websocket.close(code=1013, reason="ROS Bridge not connected")
            return

        ros_bridge.register_client(client_id)

        # 토픽 설정
        topics = {
            "/robot_1/utm_pose": "geometry_msgs/PoseStamped",
            "/robot_1/status": "robot_custom_interfaces/msg/Status",
        }
        
        # 토픽 구독 설정
        for topic_name, msg_type in topics.items():
            try:
                topic = roslibpy.Topic(ros_bridge.client, topic_name, msg_type)
                
                def create_callback(topic_name):
                    def callback(msg):
                        try:
                            # 메시지 타입별 처리
                            if topic_name == "/robot_1/utm_pose":
                                robot_state = {
                                    "type": "robot_position",
                                    "data": {
                                        "position": {
                                            "x": msg.get("pose", {}).get("position", {}).get("x", 0),
                                            "y": msg.get("pose", {}).get("position", {}).get("y", 0),
                                            "z": msg.get("pose", {}).get("position", {}).get("z", 0)
                                        },
                                        "orientation": {
                                            "x": msg.get("pose", {}).get("orientation", {}).get("x", 0),
                                            "y": msg.get("pose", {}).get("orientation", {}).get("y", 0),
                                            "z": msg.get("pose", {}).get("orientation", {}).get("z", 0),
                                            "w": msg.get("pose", {}).get("orientation", {}).get("w", 0)
                                        }
                                    },
                                    "timestamp": datetime.now().isoformat()
                                }
                            elif topic_name == "/robot_1/status":
                                robot_state = {
                                    "type": "robot_status",
                                    "data": {
                                        "battery": msg.get("battery", 0),
                                        "status": msg.get("status", ""),
                                        "mode": msg.get("mode", "")
                                    },
                                    "timestamp": datetime.now().isoformat()
                                }
                            
                            # 로깅 및 메시지 전송
                            logger.info(f"Processing message from {topic_name}: {robot_state}")
                            try:
                                loop = asyncio.get_running_loop()
                            except RuntimeError:
                                loop = asyncio.new_event_loop()
                                asyncio.set_event_loop(loop)
                            
                            loop.create_task(robot_service.broadcast_to_frontend(robot_state))

                        except Exception as e:
                            logger.error(f"Error processing message from {topic_name}: {str(e)}")
                    
                    return callback
                
                topic.subscribe(create_callback(topic_name))
                active_topics[topic_name] = topic
                logger.info(f"Subscribed to topic: {topic_name}")
                
            except Exception as e:
                logger.error(f"Failed to subscribe to topic {topic_name}: {str(e)}")

        # 메인 루프
        while True:
            try:
                # 웹소켓 메시지 수신 대기
                await asyncio.sleep(0.5)
                data = await websocket.receive_text()
                logger.info(f"Received message from client: {data}")
                
                # 클라이언트로부터 받은 메시지 처리 (필요한 경우)
                try:
                    message = json.loads(data)
                    if message.get("type") == "ping":
                        await websocket.send_json({"type": "pong"})
                        await asyncio.sleep(0.5)
                except json.JSONDecodeError:
                    logger.warning(f"Invalid JSON received: {data}")
                
            except WebSocketDisconnect:
                logger.info("WebSocket disconnected normally")
                break
            except Exception as e:
                logger.error(f"Error in websocket communication: {str(e)}")
                break
            
            await asyncio.sleep(1)

    except Exception as e:
        logger.error(f"WebSocket connection error: {str(e)}")
        logger.error(f"Traceback: {traceback.format_exc()}")
    finally:
        # 정리 작업
        try:
            # 활성 토픽 구독 해제
            for topic_name, topic in active_topics.items():
                try:
                    topic.unsubscribe()
                    logger.info(f"Unsubscribed from topic: {topic_name}")
                except Exception as e:
                    logger.error(f"Error unsubscribing from topic {topic_name}: {str(e)}")
            
            if ros_bridge:
                ros_bridge.unregister_client(client_id)
            await robot_service.unregister_frontend_client(websocket)
            
        except Exception as e:
            logger.error(f"Cleanup error: {str(e)}")

# WebSocket ?? ???? ?????




# 토픽 가용성 확인 API
@app.get("/api/v1/ros/check-topics", response_model=BaseResponse)
async def check_ros_topics():
    try:
        ros_bridge = RosBridgeConnection()
        if not ros_bridge.client or not ros_bridge.client.is_connected:
            return BaseResponse(
                success=False,
                message="ROS Bridge not connected",
                data={"status": "disconnected"}
            )

        topics = {
            "/robot_1/utm_pose": "geometry_msgs/PoseStamped",
            "/robot_1/status": "robot_custom_interfaces/msg/Status",
        }
        
        available_topics = []
        for topic_name, msg_type in topics.items():
            try:
                topic = roslibpy.Topic(ros_bridge.client, topic_name, msg_type)
                message_received = False
                timeout = time.time() + 3
                
                def callback(msg):
                    nonlocal message_received
                    message_received = True
                
                topic.subscribe(callback)
                while time.time() < timeout and not message_received:
                    await asyncio.sleep(0.1)
                topic.unsubscribe()
                
                if message_received:
                    available_topics.append(topic_name)
                
            except Exception as e:
                logger.warning(f"Topic {topic_name} not available: {str(e)}")

        return BaseResponse(
            success=True,
            message="ROS topics checked successfully",
            data={
                "status": "connected",
                "available_topics": available_topics,
                "can_use_websocket": len(available_topics) > 0  # 웹소켓 사용 가능 여부
            }
        )
        
    except Exception as e:
        logger.error(f"Error checking ROS topics: {str(e)}")
        return BaseResponse(
            success=False,
            message="Error checking ROS topics",
            data={"error": str(e)}
        )

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port, reload=False) 
