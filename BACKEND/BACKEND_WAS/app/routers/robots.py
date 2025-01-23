from fastapi import APIRouter, Path, Query, HTTPException, UploadFile, File, Form
from pydantic import BaseModel
from typing import List, Literal, Optional, Any
from datetime import datetime
from ..schemas.responses import BaseResponse, ErrorDetail
from ..database import db
from ..models.robot import Robot, Position, BatteryStatus, RobotStatus, RobotImage
from ..models.schedule import Schedule, ScheduleCreate, ScheduleUpdate, Location, OperatingTime
import os
import uuid
import re

router = APIRouter(tags=["robots"])

# 이미지 저장 경로 설정
UPLOAD_DIR = "storage/robots"
os.makedirs(UPLOAD_DIR, exist_ok=True)

class Waypoint(BaseModel):
    type: Literal["start", "mid", "end"]
    position: Position

class RouteRequest(BaseModel):
    sequence: int
    waypoints: List[Waypoint]

class RouteResponse(BaseModel):
    routeId: str
    robotId: str
    courseSequence: int
    waypoints: dict
    createdAt: datetime

class MapResponse(BaseModel):
    robotId: str
    mapId: str
    currentLocation: dict
    mapData: dict

class ControlResponse(BaseModel):
    robotId: str
    mode: str
    previousMode: str
    timestamp: datetime
    status: str

class StatusUpdate(BaseModel):
    status: Literal["manual", "auto", "emergency_stop"]

class StatusResponse(BaseModel):
    robotId: str
    status: str
    timestamp: datetime
    location: Position

class LogEntry(BaseModel):
    x: int
    y: int
    timestamp: datetime

class RouteLog(BaseModel):
    routeId: str
    startTime: datetime
    endTime: datetime
    waypoints: List[LogEntry]
    status: str

class DetectionInfo(BaseModel):
    detectionId: str
    timestamp: datetime
    location: Position
    personInfo: dict
    imageUrl: str
    status: str

class LogResponse(BaseModel):
    robotId: str
    routes: Optional[List[RouteLog]] = None
    detections: Optional[List[DetectionInfo]] = None
    pagination: dict

class RobotCreate(BaseModel):
    name: str
    ip_address: str

@router.post("", response_model=BaseResponse[Robot])
async def create_robot(
    name: str = Form(...),
    ip_address: str = Form(...),
    image: Optional[Any] = Form(default=None)
):
    try:
        # 로봇 이름 중복 검사
        existing_robot = await Robot.find_one(Robot.name == name)
        if existing_robot:
            raise HTTPException(
                status_code=400,
                detail="이미 사용 중인 로봇 이름입니다."
            )
        
        # IP 주소 형식 검증
        ip_pattern = r'^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'
        if not re.match(ip_pattern, ip_address):
            raise HTTPException(
                status_code=400,
                detail="유효하지 않은 IP 주소 형식입니다. (예: 192.168.1.100)"
            )

        # 자동 증가 ID 생성
        counter = await db.counters.find_one_and_update(
            {"_id": "robot_id"},
            {"$inc": {"seq": 1}},
            upsert=True,
            return_document=True
        )
        
        robot_id = counter["seq"]
        
        # 이미지 처리
        robot_image = None
        image_path = None
        
        if image and isinstance(image, UploadFile):
            ext = os.path.splitext(image.filename)[1].lower()
            if ext not in ['.jpg', '.jpeg', '.png']:
                raise HTTPException(
                    status_code=400,
                    detail="지원하지 않는 이미지 형식입니다."
                )
            
            image_id = f"robot_{robot_id}_{uuid.uuid4()}{ext}"
            image_path = os.path.join(UPLOAD_DIR, image_id)
            
            try:
                content = await image.read()
                if content:
                    with open(image_path, "wb") as buffer:
                        buffer.write(content)
                    
                    robot_image = RobotImage(
                        imageId=image_id,
                        url=f"/storage/robots/{image_id}",
                        createdAt=datetime.now()
                    )
            except Exception as e:
                if image_path and os.path.exists(image_path):
                    os.remove(image_path)
                print(f"이미지 처리 중 오류 발생: {str(e)}")
                pass
        
        # 로봇 생성
        robot = Robot(
            robotId=robot_id,
            name=name,
            ipAddress=ip_address,
            status=RobotStatus.IDLE,
            position=Position(),
            battery=BatteryStatus(),
            image=robot_image,
            lastActive=datetime.now(),
            createdAt=datetime.now()
        )
        
        # 저장
        await robot.insert()
        
        return BaseResponse[Robot](
            status=201,
            success=True,
            message="로봇 등록 성공",
            data=robot
        )
            
    except Exception as e:
        if image_path and os.path.exists(image_path):
            try:
                os.remove(image_path)
            except:
                pass
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

@router.get("", response_model=BaseResponse[List[Robot]])
async def get_robots():
    try:
        # MongoDB에서 모든 로봇 정보 조회
        robots = await Robot.find_all().to_list()
            
        return BaseResponse[List[Robot]](
            status=200,
            success=True,
            message="로봇 목록 조회 성공",
            data=robots
        )
    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

@router.post("/{id}/routes", response_model=BaseResponse[RouteResponse])
async def set_route(
    id: int = Path(..., description="로봇 ID"),
    request: RouteRequest = None
):
    try:
        return BaseResponse[RouteResponse](
            status=201,
            success=True,
            message="경로 설정 완료",
            data=RouteResponse(
                routeId="route_123",
                robotId=f"robot_{id}",
                courseSequence=request.sequence,
                waypoints={
                    "start": {"x": 100, "y": 200},
                    "mid": {"x": 150, "y": 250},
                    "end": {"x": 200, "y": 300}
                },
                createdAt=datetime.now()
            )
        )
    except Exception:
        return BaseResponse[RouteResponse](
            status=400,
            success=False,
            message="경로 설정 실패",
            errors=[
                ErrorDetail(
                    field="coordinates",
                    message="유효하지 않은 좌표값입니다"
                )
            ]
        )

@router.get("/{id}/map", response_model=BaseResponse[MapResponse])
async def get_map(id: int = Path(..., description="로봇 ID")):
    try:
        return BaseResponse[MapResponse](
            status=200,
            success=True,
            message="로봇 위치 정보 조회 성공",
            data=MapResponse(
                robotId=f"robot_{id}",
                mapId="map_456",
                currentLocation={
                    "x": 150,
                    "y": 200,
                    "timestamp": datetime.now().isoformat()
                },
                mapData={
                    "resolution": 0.05,
                    "width": 1000,
                    "height": 1000,
                    "origin": {"x": 0, "y": 0}
                }
            )
        )
    except Exception:
        return BaseResponse[MapResponse](
            status=404,
            success=False,
            message="맵 정보 조회 실패",
            errors=[
                ErrorDetail(
                    field="robotId",
                    message="해당 로봇을 찾을 수 없습니다"
                )
            ]
        )

@router.post("/{id}/control", response_model=BaseResponse[ControlResponse])
async def control_mode(id: int = Path(..., description="로봇 ID")):
    try:
        return BaseResponse[ControlResponse](
            status=200,
            success=True,
            message="로봇 모드 전환 성공",
            data=ControlResponse(
                robotId=f"robot_{id}",
                mode="manual",
                previousMode="auto",
                timestamp=datetime.now(),
                status="active"
            )
        )
    except Exception:
        return BaseResponse[ControlResponse](
            status=400,
            success=False,
            message="모드 전환 실패",
            errors=[
                ErrorDetail(
                    field="mode",
                    message="현재 상태에서는 모드 전환이 불가능합니다"
                )
            ]
        )

@router.patch("/{id}/status", response_model=BaseResponse[StatusResponse])
async def update_status(
    id: int = Path(..., description="로봇 ID"),
    status: StatusUpdate = None
):
    try:
        return BaseResponse[StatusResponse](
            status=200,
            success=True,
            message="비상 정지 명령 실행 완료",
            data=StatusResponse(
                robotId=f"robot_{id}",
                status="emergency_stopped",
                timestamp=datetime.now(),
                location=Position(x=150, y=200)
            )
        )
    except Exception:
        return BaseResponse[StatusResponse](
            status=500,
            success=False,
            message="비상 정지 실행 실패",
            errors=[
                ErrorDetail(
                    field="system",
                    message="로봇과의 통신이 원활하지 않습니다"
                )
            ]
        )

@router.get("/{id}/logs", response_model=BaseResponse[LogResponse])
async def get_logs(
    id: int = Path(..., description="로봇 ID"),
    type: str = Query(..., description="로그 타입 (route 또는 detection)")
):
    try:
        return BaseResponse[LogResponse](
            status=200,
            success=True,
            message="로그 조회 성공",
            data=LogResponse(
                robotId=f"robot_{id}",
                routes=[],
                detections=[],
                pagination={"total": 0, "page": 1, "size": 10}
            )
        )
    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

@router.get("/{robot_identifier}", response_model=BaseResponse[Robot])
async def get_robot(
    robot_identifier: str = Path(..., description="로봇 ID 또는 이름"),
):
    try:
        # ID인지 이름인지 확인
        if robot_identifier.isdigit():
            robot = await Robot.find_one(Robot.robotId == int(robot_identifier))
        else:
            robot = await Robot.find_one(Robot.name == robot_identifier)

        if not robot:
            raise HTTPException(
                status_code=404,
                detail="해당 로봇을 찾을 수 없습니다."
            )

        return BaseResponse[Robot](
            status=200,
            success=True,
            message="로봇 정보 조회 성공",
            data=robot
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.delete("/{robot_identifier}", response_model=BaseResponse[None])
async def delete_robot(
    robot_identifier: str = Path(..., description="로봇 ID 또는 이름")
):
    try:
        # ID인지 이름인지 확인
        if robot_identifier.isdigit():
            robot = await Robot.find_one(Robot.robotId == int(robot_identifier))
        else:
            robot = await Robot.find_one(Robot.name == robot_identifier)

        if not robot:
            raise HTTPException(
                status_code=404,
                detail="해당 로봇을 찾을 수 없습니다."
            )

        # 로봇 이미지가 있다면 삭제
        if robot.image and robot.image.imageId:
            image_path = os.path.join(UPLOAD_DIR, robot.image.imageId)
            if os.path.exists(image_path):
                os.remove(image_path)

        # 로봇 데이터 삭제
        await robot.delete()

        return BaseResponse[None](
            status=200,
            success=True,
            message="로봇이 성공적으로 삭제되었습니다."
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.post("/{robot_identifier}/schedules", response_model=BaseResponse[Schedule])
async def create_robot_schedule(
    robot_identifier: str = Path(..., description="로봇 ID 또는 이름"),
    schedule: ScheduleCreate = None
):
    try:
        # ID 또는 이름으로 로봇 찾기
        if robot_identifier.isdigit():
            robot = await Robot.find_one(Robot.robotId == int(robot_identifier))
        else:
            robot = await Robot.find_one(Robot.name == robot_identifier)

        if not robot:
            raise HTTPException(status_code=404, detail="로봇을 찾을 수 없습니다")
        
        # ID 생성
        counter = await db.counters.find_one_and_update(
            {"_id": "schedule_id"},
            {"$inc": {"seq": 1}},
            upsert=True,
            return_document=True
        )
        
        # 스케줄 생성
        new_schedule = Schedule(
            scheduleId=counter["seq"],
            robotId=robot.robotId if not schedule.apply_to_all_robots else None,
            title=schedule.title,
            description=schedule.description,
            operatingTime=OperatingTime(
                start_time=schedule.start_time,
                end_time=schedule.end_time,
                active_days=schedule.active_days
            ),
            locations=schedule.locations
        )
        
        await new_schedule.insert()
        
        return BaseResponse[Schedule](
            status=201,
            success=True,
            message="스케줄이 생성되었습니다",
            data=new_schedule
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/{robot_identifier}/schedules", response_model=BaseResponse[List[Schedule]])
async def get_robot_schedules(
    robot_identifier: str = Path(..., description="로봇 ID 또는 이름"),
    is_active: Optional[bool] = Query(None, description="활성 상태로 필터링")
):
    try:
        # ID 또는 이름으로 로봇 찾기
        if robot_identifier.isdigit():
            robot = await Robot.find_one(Robot.robotId == int(robot_identifier))
        else:
            robot = await Robot.find_one(Robot.name == robot_identifier)

        if not robot:
            raise HTTPException(status_code=404, detail="로봇을 찾을 수 없습니다")
        
        # 쿼리 조건 생성 (robotId가 None이거나 현재 로봇의 ID인 경우)
        query = {
            "$or": [
                {"robotId": robot.robotId},
                {"robotId": None}  # 모든 로봇에 적용되는 스케줄
            ]
        }
        if is_active is not None:
            query["status.isActive"] = is_active
        
        # 스케줄 조회
        schedules = await Schedule.find(query).to_list()
        
        return BaseResponse[List[Schedule]](
            status=200,
            success=True,
            message="스케줄 목록 조회 성공",
            data=schedules
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.patch("/{robot_identifier}/schedules/{schedule_id}", response_model=BaseResponse[Schedule])
async def update_robot_schedule(
    robot_identifier: str = Path(..., description="로봇 ID 또는 이름"),
    schedule_id: int = Path(..., description="스케줄 ID"),
    update_data: ScheduleUpdate = None
):
    try:
        # 로봇 확인
        if robot_identifier.isdigit():
            robot = await Robot.find_one(Robot.robotId == int(robot_identifier))
        else:
            robot = await Robot.find_one(Robot.name == robot_identifier)

        if not robot:
            raise HTTPException(status_code=404, detail="로봇을 찾을 수 없습니다")

        # 스케줄 존재 확인
        schedule = await Schedule.find_one({
            "scheduleId": schedule_id,
            "$or": [
                {"robotId": robot.robotId},
                {"robotId": None}
            ]
        })
        
        if not schedule:
            raise HTTPException(status_code=404, detail="스케줄을 찾을 수 없습니다")
        
        # 운영 시간 업데이트
        if update_data.start_time is not None or update_data.end_time is not None or update_data.active_days is not None:
            schedule.operatingTime = OperatingTime(
                start_time=update_data.start_time or schedule.operatingTime.start_time,
                end_time=update_data.end_time or schedule.operatingTime.end_time,
                active_days=update_data.active_days or schedule.operatingTime.active_days
            )
        
        # 기타 필드 업데이트
        if update_data.title is not None:
            schedule.title = update_data.title
        if update_data.description is not None:
            schedule.description = update_data.description
        if update_data.locations is not None:
            schedule.locations = update_data.locations
        if update_data.isActive is not None:
            schedule.status.isActive = update_data.isActive
        
        schedule.updatedAt = datetime.now()
        await schedule.save()
        
        return BaseResponse[Schedule](
            status=200,
            success=True,
            message="스케줄이 수정되었습니다",
            data=schedule
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e)) 