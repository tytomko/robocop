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
        "name_pattern": "/ssafy/tb3_{direction}_camera/image_raw/compressed",
        "type": "sensor_msgs/CompressedImage",
        "description": "로봇 카메라 압축 이미지 스트림"
    }
]

# Isaac 로봇 관련 토픽 정의
ISAAC_TOPICS = [
    {
        "name": "/Isaacbot/front/img_compressed",
        "type": "sensor_msgs/msg/Image",
        "description": "Isaac 로봇 전면 카메라 이미지"
    },
    {
        "name": "/Isaacbot/rear/img_compressed",
        "type": "sensor_msgs/msg/Image",
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
            robot_service = RobotService()  # 인스턴스 생성
            robot = await robot_service.get_robot(identifier=seq)  # identifier로 seq 전달
            if not robot:
                raise HTTPException(status_code=404, detail="로봇을 찾을 수 없습니다")
            
            # CAMERA_TOPICS의 패턴을 사용하여 토픽 이름 생성
            topic_pattern = CAMERA_TOPICS[0]["name_pattern"]
            topic_name = topic_pattern.format(seq=seq, direction=direction)
            topic_type = CAMERA_TOPICS[0]["type"]
            
            # 일반 로봇용 토픽 설정
            camera_service.set_topic(
                topic_name,
                topic_type,
                is_isaac=False
            )
        
        return StreamingResponse(
            camera_service.get_frame(),
            media_type="multipart/x-mixed-replace; boundary=frame"
        )
    except Exception as e:
        logger.error(f"카메라 스트리밍 에러: {str(e)}")
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