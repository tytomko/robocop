from fastapi import APIRouter, WebSocket, HTTPException
from ..service.lidar_service import LidarService
from ..models.lidar_models import LidarConfig, LidarStatus
from ....common.models.responses import BaseResponse
from ....common.config.manager import get_settings
import asyncio
import logging

logger = logging.getLogger(__name__)
router = APIRouter()
lidar_service = LidarService()
settings = get_settings()

# 기본 라이다 설정
DEFAULT_LIDAR_CONFIG = LidarConfig(
    ros_bridge_host="172.30.1.78",
    ros_bridge_port=9090,
    topic_name="/ssafy/velodyne_points",
    message_type="sensor_msgs/PointCloud2",
    update_interval=0.1,
    max_points=1000
)

@router.websocket("/ws")
async def lidar_websocket(websocket: WebSocket):
    """라이다 데이터 웹소켓 엔드포인트"""
    if not lidar_service.is_connected:
        await websocket.close(code=1000, reason="라이다가 연결되지 않았습니다")
        return
        
    await lidar_service.register_client(websocket)
    try:
        while True:
            await websocket.receive_text()
    except Exception as e:
        await lidar_service.unregister_client(websocket)

@router.get("/status", response_model=BaseResponse)
async def get_lidar_status():
    """라이다 상태를 조회합니다."""
    status = lidar_service.get_status()
    
    if not lidar_service.is_connected:
        return BaseResponse(
            success=False,
            message="라이다가 연결되지 않았습니다",
            data={"status": "disconnected"}
        )
    
    return BaseResponse(
        success=True,
        message="라이다 상태 조회 성공",
        data={"status": status}
    )

@router.post("/config", response_model=BaseResponse)
async def update_lidar_config(config: LidarConfig):
    """라이다 설정을 업데이트합니다."""
    if not lidar_service.is_connected:
        return BaseResponse(
            success=False,
            message="라이다가 연결되지 않았습니다",
            data={"status": "disconnected"}
        )
        
    try:
        lidar_service.config = config
        return BaseResponse(
            success=True,
            message="라이다 설정 업데이트 성공",
            data={"status": lidar_service.get_status()}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/reconnect", response_model=BaseResponse)
async def reconnect_lidar():
    """라이다 연결을 재시도합니다."""
    try:
        # 기존 연결 종료
        if lidar_service.is_connected:
            await lidar_service.disconnect()
        
        # 연결 재시도
        connected = await lidar_service.connect(
            host=settings.ros.ROS_BRIDGE_HOST,
            port=settings.ros.ROS_BRIDGE_PORT
        )
        
        if not connected:
            return BaseResponse(
                success=False,
                message="라이다 연결 실패",
                data={"status": "disconnected"}
            )
            
        # 구독 시작
        asyncio.create_task(lidar_service.subscribe())
        
        return BaseResponse(
            success=True,
            message="라이다 재연결 성공",
            data={"status": "connected"}
        )
        
    except AttributeError:
        return BaseResponse(
            success=False,
            message="ROS 브릿지 설정을 찾을 수 없습니다",
            data={"status": "error"}
        )
    except Exception as e:
        logger.error(f"라이다 재연결 실패: {str(e)}")
        return BaseResponse(
            success=False,
            message=f"라이다 재연결 실패: {str(e)}",
            data={"status": "error"}
        )

async def start_lidar_subscriber():
    """라이다 구독자를 시작합니다."""
    try:
        # 기본 설정 적용
        lidar_service.config = DEFAULT_LIDAR_CONFIG
        
        # 연결 시도
        try:
            connected = await lidar_service.connect(
                host=lidar_service.config.ros_bridge_host,
                port=lidar_service.config.ros_bridge_port
            )
            
            if not connected:
                logger.warning("라이다 기능이 비활성화되었습니다")
                return
                
            logger.info("라이다 서비스가 시작되었습니다")
            await lidar_service.subscribe()
            
        except AttributeError:
            logger.warning("ROS 브릿지 설정을 찾을 수 없습니다. 라이다 기능이 비활성화됩니다.")
        except Exception as e:
            logger.warning(f"라이다 서비스 오류: {str(e)}")
            
    except asyncio.CancelledError:
        logger.info("라이다 서비스가 중단되었습니다")
    except Exception as e:
        logger.error(f"라이다 서비스 오류: {str(e)}")