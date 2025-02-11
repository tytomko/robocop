from fastapi import APIRouter, WebSocket, HTTPException
from fastapi.responses import StreamingResponse
from ....common.models.responses import BaseResponse
from ..service.camera_service import camera_service  # 싱글톤 인스턴스 import
import logging

logger = logging.getLogger(__name__)
router = APIRouter()

@router.get("/video_feed")
async def video_feed():
    """카메라 영상 스트리밍 엔드포인트"""
    return StreamingResponse(
        camera_service.get_frame(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

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
