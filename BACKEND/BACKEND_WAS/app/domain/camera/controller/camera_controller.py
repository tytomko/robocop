from fastapi import APIRouter, WebSocket, HTTPException
from fastapi.responses import StreamingResponse
from ....common.models.responses import BaseResponse
from ..service.camera_service import camera_service
from ...robot.service.robot_service import RobotService  # robot 도메인에서 가져오기
import logging
import roslibpy

logger = logging.getLogger(__name__)
router = APIRouter()

# ROS 토픽 정보 상수 정의
CAMERA_TOPICS = [
    {
        "name_pattern": "/robot_{seq}/{direction}/img_compressed",
        "type": "sensor_msgs/CompressedImage",
        "description": "로봇 카메라 압축 이미지 스트림"
    }
]

# Isaac 로봇 관련 토픽 정의
ISAAC_TOPICS = [
    {
        "name": "/Isaacbot/front/img_compressed",
        "type": "sensor_msgs/CompressedImage",
        "description": "Isaac 로봇 전면 카메라 이미지"
    },
    {
        "name": "/Isaacbot/rear/img_compressed",
        "type": "sensor_msgs/CompressedImage",
        "description": "Isaac 로봇 후면 카메라 이미지"
    },
    {
        "name": "/Isaacbot/velodyne_points",
        "type": "sensor_msgs/PointCloud2",
        "description": "Velodyne LiDAR 포인트 클라우드 데이터"
    },
    {
        "name": "/Isaacbot/cmd_vel",
        "type": "geometry_msgs/Twist",
        "description": "로봇 속도 명령"
    },
    {
        "name": "/Isaacbot/imu",
        "type": "sensor_msgs/Imu",
        "description": "IMU 센서 데이터"
    }
]

@router.get("/video_feed/{seq}/{direction}")
async def video_feed(seq: int, direction: str):
    """로봇 카메라 영상 스트리밍 엔드포인트"""
    try:
        # IsaacSim 로봇인 경우 (seq = 11)
        if seq == 11:
            if direction not in ['front', 'rear']:
                raise HTTPException(status_code=400, detail="유효하지 않은 카메라 방향입니다")
            
            # Isaac 토픽 찾기
            topic_info = next(
                (topic for topic in ISAAC_TOPICS 
                 if topic["name"] == f"/Isaacbot/{direction}/img_compressed"),
                None
            )
            
            if not topic_info:
                raise HTTPException(status_code=400, detail="해당하는 카메라 토픽을 찾을 수 없습니다")
            
            # Isaac 로봇용 토픽 설정
            camera_service.set_topic(
                topic_info["name"],
                topic_info["type"],
                is_isaac=True
            )
        else:
            # 일반 로봇인 경우
            robot = await RobotService.get_robot(seq)
            if not robot:
                raise HTTPException(status_code=404, detail="로봇을 찾을 수 없습니다")
            
            topic_name = f"/{robot.manufactureName}/{direction}/img_compressed"
            # 일반 로봇용 토픽 설정
            camera_service.set_topic(
                topic_name,
                CAMERA_TOPICS[0]["type"],
                is_isaac=False
            )
        
        return StreamingResponse(
            camera_service.get_frame(),
            media_type="multipart/x-mixed-replace; boundary=frame"
        )
    except Exception as e:
        logger.error(f"카메라 스트리밍 에러: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/isaac/video_feed/{direction}")
async def isaac_video_feed(direction: str):
    """Isaac 로봇 카메라 영상 스트리밍 엔드포인트"""
    try:
        if direction not in ['front', 'rear']:
            raise HTTPException(status_code=400, detail="유효하지 않은 카메라 방향입니다")
        
        # Isaac 토픽 찾기
        topic_info = next(
            (topic for topic in ISAAC_TOPICS 
             if topic["name"] == f"/Isaacbot/{direction}/img_compressed"),
            None
        )
        
        if not topic_info:
            raise HTTPException(status_code=400, detail="해당하는 카메라 토픽을 찾을 수 없습니다")
        
        # camera_service 토픽 설정
        camera_service.topic = roslibpy.Topic(
            camera_service.client,
            topic_info["name"],
            topic_info["type"]
        )
        
        # 새로운 토픽 구독 시작
        camera_service.topic.subscribe(camera_service._on_image_message)
        
        return StreamingResponse(
            camera_service.get_frame(),
            media_type="multipart/x-mixed-replace; boundary=frame"
        )
    except Exception as e:
        logger.error(f"Isaac 카메라 스트리밍 에러: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

# @router.websocket("/ws")
# async def websocket_endpoint(websocket: WebSocket):
#     """WebSocket 연결을 처리하는 엔드포인트"""
#     try:
#         await websocket.accept()
#         while True:
#             data = await websocket.receive_text()
#             await websocket.send_text(f"Message received: {data}")
#     except Exception as e:
#         logger.error(f"WebSocket 에러: {str(e)}")
#     finally:
#         try:
#             await websocket.close()
#         except:
#             pass