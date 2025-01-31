from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from .routers import auth, robots, persons
from .database import init_db, test_connection, Base, engine
import uvicorn
import os
import logging
import asyncio
from app.lidarsub import websocket_endpoint, start_lidar_subscriber
from app.camera.camera_rtc import CameraRTC
from fastapi import HTTPException
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)
app = FastAPI()

# CORS 설정 수정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # 프론트엔드 주소 명시
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"]
)

# 라우터 등록
app.include_router(auth.router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(robots.router, prefix="/api/v1/robots", tags=["robots"])
app.include_router(persons.router, tags=["persons"])

# 웹소켓 엔드포인트 추가
@app.websocket("/ws")
async def lidar_websocket(websocket: WebSocket):
    await websocket_endpoint(websocket)

# 카메라 인스턴스 생성
robot1_front_camera = CameraRTC(
    camera_topic='/ssafy/tb3_front_camera/image_raw/compressed',
    camera_name='robot1',
    ros_bridge_host='192.168.100.104',
    ros_bridge_port=9090
)

robot1_rear_camera = CameraRTC(
    camera_topic='/ssafy/tb3_rear_camera/image_raw/compressed',
    camera_name='robot1',
    ros_bridge_host='192.168.100.104',
    ros_bridge_port=9090
)

@app.on_event("startup")
async def startup_event():
    logger.info("Starting up application...")
    # SQLAlchemy 테이블 생성
    Base.metadata.create_all(bind=engine)
    
    await init_db()
    await auth.create_admin_user()
    await test_connection()
    asyncio.create_task(start_lidar_subscriber())
    
    # 각 로봇의 카메라 ROS 연결
    await robot1_front_camera.connect_ros()
    await robot1_rear_camera.connect_ros()

@app.get("/")
async def root():
    return {"message": "Robot Management API"}

@app.post("/webrtc/{robot_id}/{camera_type}/offer")
async def camera_offer(robot_id: str, camera_type: str, params: dict):
    try:
        print(f"Received offer request - robot_id: {robot_id}, camera_type: {camera_type}")
        
        cameras = {
            "robot1": {
                "front": robot1_front_camera,
                "rear": robot1_rear_camera
            }
        }
        
        camera = cameras.get(robot_id, {}).get(camera_type)
        print(f"Selected camera: {camera}")
        
        if not camera:
            raise HTTPException(status_code=404, detail=f"Camera not found for robot_id: {robot_id}, camera_type: {camera_type}")
            
        return await camera.create_peer_connection(params)
    except Exception as e:
        print(f"Camera offer error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

# 에러 핸들링 추가
@app.exception_handler(500)
async def internal_error_handler(request, exc):
    print(f"서버 에러 발생: {str(exc)}")
    return JSONResponse(
        status_code=500,
        content={"detail": str(exc)}
    )

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port, reload=False) 