<<<<<<< HEAD
from fastapi import APIRouter, Path, Query, Depends, UploadFile, File, Form, Body, status, WebSocket, WebSocketDisconnect, Request
from typing import List, Optional, Dict, Set
from ..service.robot_service import RobotService
from ..models.robot_models import (
    Robot, StatusUpdate, StatusResponse, LogResponse, NicknameResponse  # RouteRequest, RouteResponse, MapResponse, NicknameResponse 제거
)

=======
from fastapi import APIRouter, Path, Query, Depends, UploadFile, File, Form, Body, status, WebSocket, WebSocketDisconnect
from typing import List, Optional, Dict, Set
from ..service.robot_service import RobotService
from ..models.robot_models import (
    Robot, StatusUpdate, StatusResponse, LogResponse  # RouteRequest, RouteResponse, MapResponse 제거
)
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
from ....common.models.responses import BaseResponse
from fastapi import HTTPException
import logging 
import traceback
import time
import asyncio
from fastapi.responses import StreamingResponse
import json

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

router = APIRouter()
robot_service = RobotService()
<<<<<<< HEAD
logger = logging.getLogger(__name__)
=======
ws_manager = RobotWebSocketManager()  # 클래스 이름 변경
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23

class RobotWebSocketManager:
    def __init__(self):
        # robot_id를 키로 하는 연결된 클라이언트 목록
        self.active_connections: Dict[int, Set[WebSocket]] = {}
        
    async def connect(self, websocket: WebSocket, robot_id: int):
        """새로운 WebSocket 연결을 추가합니다."""
        try:
            # 이미 accept()가 호출되었으므로 여기서는 생략
            if robot_id not in self.active_connections:
                self.active_connections[robot_id] = set()
            self.active_connections[robot_id].add(websocket)
            logger.info(f"Added new WebSocket connection for robot {robot_id}")
        except Exception as e:
            logger.error(f"Error connecting WebSocket: {str(e)}")
            raise

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

@router.post("/", response_model=BaseResponse[Robot])
async def create_robot(
<<<<<<< HEAD
    nickname: str = Form(..., description="로봇 별칭 (사용자 지정)"),  # 사용자가 입력하는 이름
=======
    name: str = Form(..., description="로봇 별칭 (사용자 지정)"),  # 사용자가 입력하는 이름
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
    ipAddress: str = Form(...),
    image: Optional[UploadFile] = File(None)
):
    """로봇을 생성합니다. FormData로 이미지와 함께 데이터를 받습니다."""
    try:
        robot = await robot_service.create_robot(
<<<<<<< HEAD
            nickname=nickname,  # 사용자가 입력한 이름은 nickname으로
=======
            nickname=name,  # 사용자가 입력한 이름은 nickname으로
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
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
        logger.error(f"로봇 목록 조회 중 오류: {str(e)}")
        logger.error(traceback.format_exc())  # 스택 트레이스 추가
        raise HTTPException(
            status_code=500,
            detail=f"로봇 목록 조회 중 오류가 발생했습니다: {str(e)}"
        )

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

<<<<<<< HEAD
@router.patch("/{seq}/nickname", response_model=BaseResponse[NicknameResponse])
async def update_robot_nickname(
    seq: int = Path(..., description="로봇 ID"),
    new_nickname: str = Body(..., description="새로운 닉네임")
):
    """로봇의 닉네임을 변경합니다."""
    try:
        updated_robot = await robot_service.update_nickname(seq, new_nickname)
        return BaseResponse[NicknameResponse](
            status=200,
            success=True,
            message="로봇 닉네임이 성공적으로 변경되었습니다.",
            data=NicknameResponse(
                seq=updated_robot.seq,
                robotID=str(updated_robot.seq),
                nickname=updated_robot.nickname,
                status=updated_robot.status,
                timestamp=updated_robot.updatedAt
            )
        )
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"로봇 닉네임 변경 중 오류가 발생했습니다: {str(e)}"
        )


@router.websocket("/ws/monitoring/{robot_id}")
async def robot_monitoring(
    websocket: WebSocket,
    robot_id: int = Path(...),
):
    """로봇 모니터링을 위한 웹소켓 연결을 처리합니다."""
    try:
        # CORS 헤더 수동 설정
        await websocket.accept()
        response_headers = {
            'Access-Control-Allow-Origin': '*',
            'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
            'Access-Control-Allow-Headers': '*',
            'Access-Control-Allow-Credentials': 'true',
        }
        for key, value in response_headers.items():
            websocket.headers[key] = value

        logger.info(f"새로운 WebSocket 연결 시도: robot_id={robot_id}")
        
        await ws_manager.connect(websocket, robot_id)
        logger.info(f"WebSocket 매니저에 연결됨: robot_id={robot_id}")
        
        # ROS2 상태 구독
        ros_robot_id = "robot_1"
        logger.info(f"ROS2 토픽 구독 시작: {ros_robot_id}")
        await robot_service.subscribe_to_robot_status(ros_robot_id)
=======
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
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
        
        try:
            while True:
                data = await websocket.receive_json()
<<<<<<< HEAD
                logger.debug(f"수신된 데이터: robot_id={robot_id}, data={data}")
                
                if data.get("type") == "ping":
                    await websocket.send_json({"type": "pong"})
                    logger.debug(f"Pong 응답 전송: robot_id={robot_id}")
                    
        except WebSocketDisconnect:
            logger.info(f"클라이언트 연결 해제: robot_id={robot_id}")
        finally:
            await ws_manager.disconnect(websocket, robot_id)
            logger.info(f"WebSocket 매니저에서 연결 해제: robot_id={robot_id}")
            
    except Exception as e:
        logger.error(f"WebSocket 에러 발생: robot_id={robot_id}, error={str(e)}")
        if not websocket.client_state.disconnected:
            await websocket.close()
            
@router.get("/sse/{seq}/down-utm", tags=["robots"])
async def get_robot_down_utm(seq: int, request: Request):
    """
    SSE 형식으로 /robot_{seq}/down_utm 토픽 메시지를 전송합니다.
    매 1초마다 position 필드의 x, y 값만 추출하여 전송합니다.
    메시지 예시:
    {
      "seq": <seq>,
      "position": {"x": <x>, "y": <y>}
    }
    """
    try:
        # RobotService 인스턴스를 가져옵니다.
        robot_service = await RobotService.get_instance(seq)
        # 토픽 구독 설정 (구독 시 콜백에서 last_down_utm 변수가 갱신됩니다)
        await robot_service.subscribe_down_utm(seq)
        
        async def event_generator():
            while True:
                if await request.is_disconnected():
                    break
                message_data = robot_service.last_down_utm
                if message_data:
                    pos = message_data.get("position", {})
                    x = pos.get("x")
                    y = pos.get("y")
                    payload = {"seq": seq, "position": {"x": x, "y": y}}
                else:
                    payload = {"seq": seq, "position": None}
                yield f"data: {json.dumps(payload)}\n\n"
                await asyncio.sleep(0.2)
        
        return StreamingResponse(event_generator(), media_type="text/event-stream")
    except Exception as e:
        logger.error(f"다운 UTm SSE 에러: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
        
        
@router.get("/sse/1/alert", tags=["robots"])
async def get_robot_alert( request: Request):
    """
    SSE 형식으로 /robot_{seq}/ai_info 토픽 메시지를 전송합니다.
    거수자 인식 관련 알림을 실시간으로 전송합니다.
    
    메시지 예시:
    {
        "seq": <seq>,
        "alert": {
            "type": "emergency" | "clear" | "caution",
            "message": <original_message>,  # "거수자인식-정지", "거수자 처리완료-재개", "거수자 사라짐"
            "timestamp": <timestamp>
        }
    }
    """

    
    seq = 1
    robot_service = await RobotService.get_instance(1)
    await robot_service.subscribe_alert(1)
    
    try:
        # logger.info(f"로봇 알림 SSE 스트림 시작 - seq: {seq}")
        # logger.info(f"로봇 {seq}의 알림 토픽 구독 시작")

        async def event_generator():
            # 초기 연결 시 빈 데이터 전송
            yield f"data: {json.dumps({'seq': seq, 'alert': None})}\n\n"
            
            while True:
                if await request.is_disconnected():
                    await robot_service.cleanup(seq)
                    break

                alert_data = robot_service.last_alert
                if alert_data:
                    payload = {
                        "seq": seq,
                        "alert": alert_data
                    }
                    robot_service.last_alert = None
                    yield f"data: {json.dumps(payload)}\n\n"
                
                await asyncio.sleep(0.2)
        
        return StreamingResponse(event_generator(), media_type="text/event-stream")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) 
=======
                # 데이터 처리 로직은 추후 구현
                await ws_manager.broadcast_to_robot(robot_id, data)
        except WebSocketDisconnect:
            await ws_manager.disconnect(websocket, robot_id)
            
    except Exception as e:
        await websocket.close(code=4000, reason=str(e))
>>>>>>> dc86656e24a4d32ae1d229d37b8d461d9390ac23
