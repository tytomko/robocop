from fastapi import APIRouter, Path, Query, Depends, UploadFile, File, Form, Body, status, WebSocket, WebSocketDisconnect
from typing import List, Optional, Dict, Set
from ..service.robot_service import RobotService
from ..models.robot_models import (
    Robot, StatusUpdate, StatusResponse, LogResponse  # RouteRequest, RouteResponse, MapResponse 제거
)
from ....common.models.responses import BaseResponse
from fastapi import HTTPException

router = APIRouter()

class RobotWebSocketManager:
    def __init__(self):
        # robot_id를 키로 하는 연결된 클라이언트 목록
        self.active_connections: Dict[int, Set[WebSocket]] = {}
        
    async def connect(self, websocket: WebSocket, robot_id: int):
        """새로운 WebSocket 연결을 추가합니다."""
        await websocket.accept()
        if robot_id not in self.active_connections:
            self.active_connections[robot_id] = set()
        self.active_connections[robot_id].add(websocket)
    
    async def disconnect(self, websocket: WebSocket, robot_id: int):
        """WebSocket 연결을 제거합니다."""
        self.active_connections[robot_id].remove(websocket)
        if not self.active_connections[robot_id]:
            del self.active_connections[robot_id]
    
    async def broadcast_to_robot(self, robot_id: int, message: dict):
        """특정 로봇의 모든 연결된 클라이언트에게 메시지를 전송합니다."""
        if robot_id in self.active_connections:
            disconnected = set()
            for connection in self.active_connections[robot_id]:
                try:
                    await connection.send_json(message)
                except Exception:
                    disconnected.add(connection)
            
            # 끊어진 연결 제거
            for connection in disconnected:
                await self.disconnect(connection, robot_id)
    
    def get_robot_connections(self, robot_id: int) -> Set[WebSocket]:
        """특정 로봇의 모든 연결된 클라이언트를 반환합니다."""
        return self.active_connections.get(robot_id, set())

robot_service = RobotService()
ws_manager = RobotWebSocketManager()  # 클래스 이름 변경

@router.post("", response_model=BaseResponse[Robot])
async def create_robot(
    name: str = Form(..., description="로봇 별칭 (사용자 지정)"),  # 사용자가 입력하는 이름
    ipAddress: str = Form(...),
    image: Optional[UploadFile] = File(None)
):
    """로봇을 생성합니다. FormData로 이미지와 함께 데이터를 받습니다."""
    try:
        robot = await robot_service.create_robot(
            nickname=name,  # 사용자가 입력한 이름은 nickname으로
            ip_address=ipAddress,
            image=image
        )
        return BaseResponse[Robot](
            status=201,
            success=True,
            message="로봇 등록 성공",
            data=robot
        )
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

@router.get("/", response_model=BaseResponse[List[Robot]])
async def get_robots():
    """모든 로봇 목록을 반환합니다."""
    try:
        robots = await robot_service.get_all_robots()
        return BaseResponse[List[Robot]](
            status=200,
            success=True,
            message="로봇 목록 조회 성공",
            data=robots
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/{identifier}", response_model=BaseResponse[Robot])
async def get_robot(
    identifier: str = Path(..., description="로봇 ID 또는 닉네임")
):
    """로봇 정보를 조회합니다.
    - ID로 조회: 숫자 입력 (예: 1)
    - 닉네임으로 조회: 문자열 입력 (예: "로봇1")
    """
    try:
        robot = await robot_service.get_robot(identifier)
        return BaseResponse[Robot](
            status=200,
            success=True,
            message="로봇 정보 조회 성공",
            data=robot
        )
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"로봇 정보 조회 중 오류가 발생했습니다: {str(e)}"
        )

@router.delete("/{identifier}", response_model=BaseResponse)
async def delete_robot(
    identifier: str = Path(..., description="로봇 ID 또는 닉네임")
):
    """로봇을 삭제(비활성화) 처리합니다."""
    try:
        success = await robot_service.delete_robot(identifier)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="로봇을 찾을 수 없습니다."
            )
        return BaseResponse(
            status=200,
            success=True,
            message="로봇이 성공적으로 삭제되었습니다."
        )
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"로봇 삭제 중 오류가 발생했습니다: {str(e)}"
        )


@router.get("/{id}/logs", response_model=BaseResponse[LogResponse])
async def get_logs(
    id: int = Path(..., description="로봇 ID"),
    type: str = Query(..., description="로그 타입 (route 또는 detection)")
):
    try:
        log_response = await robot_service.get_logs(id, type)
        return BaseResponse[LogResponse](
            status=200,
            success=True,
            message="로그 조회 성공",
            data=log_response
        )
    except Exception as e:
        raise e

@router.patch("/{identifier}/status", response_model=BaseResponse[StatusResponse])
async def update_robot_status(
    identifier: str = Path(..., description="로봇 ID 또는 닉네임"),
    status_update: StatusUpdate = Body(..., description="변경할 상태")
):
    """로봇의 상태를 변경합니다."""
    try:
        status_response = await robot_service.update_status(identifier, status_update.status)
        return BaseResponse[StatusResponse](
            status=200,
            success=True,
            message="로봇 상태가 성공적으로 변경되었습니다.",
            data=status_response
        )
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"로봇 상태 변경 중 오류가 발생했습니다: {str(e)}"
        )

@router.websocket("/ws/{robot_id}")
async def websocket_endpoint(
    websocket: WebSocket, 
    robot_id: int,
    robot_service: RobotService = Depends()
):
    """로봇의 실시간 데이터를 위한 WebSocket 엔드포인트"""
    try:
        # 로봇 존재 여부 확인
        robot = await robot_service.get_robot(robot_id)
        if not robot:
            await websocket.close(code=4004, reason="Robot not found")
            return

        # WebSocket 연결
        await ws_manager.connect(websocket, robot_id)
        
        try:
            while True:
                data = await websocket.receive_json()
                # 데이터 처리 로직은 추후 구현
                await ws_manager.broadcast_to_robot(robot_id, data)
        except WebSocketDisconnect:
            await ws_manager.disconnect(websocket, robot_id)
            
    except Exception as e:
        await websocket.close(code=4000, reason=str(e))