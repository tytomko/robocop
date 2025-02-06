from fastapi import APIRouter, WebSocket, HTTPException
from ..service.camera_service import CameraService
from ..models.camera_models import CameraConfig, WebRTCOffer, WebRTCAnswer, CameraStatus
from ....common.models.responses import BaseResponse
import logging

logger = logging.getLogger(__name__)
router = APIRouter()

# 카메라 서비스 인스턴스 저장
camera_services = {}

# 모듈 레벨로 이동
async def initialize_camera(config: CameraConfig) -> bool:
    """카메라 초기화 및 ROS 연결"""
    try:
        camera_service = CameraService()
        status = await camera_service.initialize_camera(config)
        
        if status.is_connected:
            camera_services[config.camera_name] = camera_service
            logger.info(f"카메라 초기화 성공: {config.camera_name}")
            return True
        else:
            logger.warning(f"카메라 초기화 실패: {config.camera_name} - {status.error_message}")
            return False
    except Exception as e:
        logger.error(f"카메라 초기화 중 오류 발생: {str(e)}")
        return False

@router.post("/initialize", response_model=BaseResponse[CameraStatus])
async def initialize_camera_endpoint(config: CameraConfig):
    """카메라 초기화 API 엔드포인트"""
    success = await initialize_camera(config)
    if success:
        camera_service = camera_services[config.camera_name]
        status = camera_service.get_status()
        return BaseResponse[CameraStatus](
            status=200,
            success=True,
            message="카메라 초기화 성공",
            data=status
        )
    else:
        raise HTTPException(status_code=500, detail="카메라 초기화 실패")

@router.post("/webrtc/{robot_id}/{camera_type}/offer", response_model=BaseResponse[WebRTCAnswer])
async def create_webrtc_connection(robot_id: str, camera_type: str, offer: WebRTCOffer):
    """WebRTC 연결 생성"""
    try:
        camera_name = f"{robot_id}_{camera_type}"
        if camera_name not in camera_services:
            return BaseResponse[WebRTCAnswer](
                success=False,
                message="카메라가 연결되지 않았습니다.",
                data=WebRTCAnswer(
                    sdp="",
                    type="disconnected"
                )
            )
            
        camera_service = camera_services[camera_name]
        answer = await camera_service.create_webrtc_connection(camera_name, offer)
        
        return BaseResponse[WebRTCAnswer](
            status=200,
            success=True,
            message="WebRTC 연결 성공",
            data=answer
        )
    except Exception as e:
        logger.error(f"WebRTC 연결 생성 실패: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/status/{camera_name}", response_model=BaseResponse)
async def get_camera_status(camera_name: str):
    """카메라 상태를 조회합니다."""
    try:
        if camera_name not in camera_services:
            return BaseResponse(
                success=False,
                message="카메라가 연결되지 않았습니다.",
                data={"status": "disconnected"}
            )
        
        camera_service = camera_services[camera_name]
        status = camera_service.get_status()
        
        return BaseResponse(
            success=True,
            message="카메라 상태 조회 성공",
            data={"status": status}
        )
    except Exception as e:
        logger.error(f"카메라 상태 조회 실패: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 연결을 처리하는 엔드포인트"""
    try:
        await websocket.accept()
        
        while True:
            data = await websocket.receive_text()
            # 필요한 WebSocket 로직 처리
            await websocket.send_text(f"Message received: {data}")
            
    except Exception as e:
        logger.error(f"WebSocket 에러: {str(e)}")
    finally:
        try:
            await websocket.close()
        except:
            pass