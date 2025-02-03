from fastapi import APIRouter, Path, Query, Depends, UploadFile, File, Form, Body
from typing import List, Optional
from ..service.robot_service import RobotService
from ..models.robot_models import Robot, RouteRequest, RouteResponse, MapResponse, StatusUpdate, StatusResponse, LogResponse, RobotCreate
from ....common.models.responses import BaseResponse
from fastapi import HTTPException

router = APIRouter()
robot_service = RobotService()

@router.post("", response_model=BaseResponse[Robot])
async def create_robot(
    robot_data: RobotCreate = Body(..., example={
        "name": "Robot1",
        "ip_address": "192.168.1.100"
    })
):
    try:
        robot = await robot_service.create_robot(
            name=robot_data.name,
            ip_address=robot_data.ip_address,
            image=None
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

@router.get("", response_model=BaseResponse[List[Robot]])
async def get_robots():
    try:
        robots = await robot_service.get_all_robots()
        return BaseResponse[List[Robot]](
            status=200,
            success=True,
            message="로봇 목록 조회 성공",
            data=robots
        )
    except Exception as e:
        raise e

@router.get("/{robot_identifier}", response_model=BaseResponse[Robot])
async def get_robot(
    robot_identifier: str = Path(..., description="로봇 ID 또는 이름")
):
    try:
        # ID로 조회 시도
        try:
            robot_id = int(robot_identifier)
        except ValueError:
            robot_id = robot_identifier

        robot = await robot_service.get_robot(robot_id)
        return BaseResponse[Robot](
            status=200,
            success=True,
            message="로봇 정보 조회 성공",
            data=robot
        )
    except Exception as e:
        raise e

@router.delete("/{robot_identifier}", response_model=BaseResponse)
async def delete_robot(
    robot_identifier: str = Path(..., description="로봇 ID 또는 이름")
):
    try:
        # ID로 삭제 시도
        try:
            robot_id = int(robot_identifier)
        except ValueError:
            robot_id = robot_identifier

        await robot_service.delete_robot(robot_id)
        return BaseResponse(
            status=200,
            success=True,
            message="로봇이 성공적으로 삭제되었습니다."
        )
    except Exception as e:
        raise e

@router.post("/{id}/routes", response_model=BaseResponse[RouteResponse])
async def set_route(
    id: int = Path(..., description="로봇 ID"),
    request: RouteRequest = None
):
    try:
        route_response = await robot_service.set_route(
            id,
            request.sequence,
            {wp.type: wp.position.dict() for wp in request.waypoints}
        )
        return BaseResponse[RouteResponse](
            status=201,
            success=True,
            message="경로 설정 완료",
            data=route_response
        )
    except Exception as e:
        raise e

@router.get("/{id}/map", response_model=BaseResponse[MapResponse])
async def get_map(id: int = Path(..., description="로봇 ID")):
    try:
        map_response = await robot_service.get_map(id)
        return BaseResponse[MapResponse](
            status=200,
            success=True,
            message="로봇 위치 정보 조회 성공",
            data=map_response
        )
    except Exception as e:
        raise e

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

@router.patch("/{robot_id}/status", response_model=BaseResponse[StatusResponse])
async def update_robot_status(
    robot_id: int = Path(..., description="로봇 ID"),
    status_update: StatusUpdate = Body(..., description="변경할 상태")
):
    """로봇의 상태를 변경합니다."""
    try:
        status_response = await robot_service.update_status(robot_id, status_update.status)
        return BaseResponse[StatusResponse](
            status=200,
            success=True,
            message="로봇 상태가 성공적으로 변경되었습니다.",
            data=status_response
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )