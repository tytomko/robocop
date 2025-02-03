import re
from datetime import datetime
from fastapi import HTTPException, UploadFile
from typing import List, Optional, Union
from ..repository.robot_repository import RobotRepository
from ..models.robot_models import (
    Robot, Position, BatteryStatus, RobotStatus, RobotImage,
    RouteResponse, MapResponse, StatusResponse, LogResponse
)

class RobotService:
    def __init__(self):
        self.repository = RobotRepository()

    async def create_robot(self, name: str, ip_address: str, image: Optional[UploadFile] = None) -> Robot:
        try:
            # IP 주소 형식 검증
            ip_pattern = r'^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'
            if not re.match(ip_pattern, ip_address):
                raise HTTPException(
                    status_code=400,
                    detail="유효하지 않은 IP 주소 형식입니다. (예: 192.168.1.100)"
                )

            # 로봇 이름 중복 검사
            existing_robot = await self.repository.find_robot_by_name(name)
            if existing_robot:
                raise HTTPException(
                    status_code=400,
                    detail="이미 사용 중인 로봇 이름입니다."
                )

            # 새 로봇 ID 생성
            robot_id = await self.repository.get_next_robot_id()

            # 이미지 처리
            robot_image = None
            if image:
                robot_image = await self.repository.save_robot_image(image, robot_id)

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

            return await self.repository.create_robot(robot)
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 생성 중 오류 발생: {str(e)}"
            )

    async def get_all_robots(self) -> List[Robot]:
        return await self.repository.find_all_robots()

    async def get_robot(self, identifier: Union[int, str]) -> Robot:
        if isinstance(identifier, int):
            robot = await self.repository.find_robot_by_id(identifier)
        else:
            robot = await self.repository.find_robot_by_name(identifier)

        if not robot:
            raise HTTPException(
                status_code=404,
                detail="로봇을 찾을 수 없습니다."
            )
        return robot

    async def delete_robot(self, identifier: Union[int, str]) -> bool:
        robot = await self.get_robot(identifier)
        
        # 이미지가 있다면 삭제
        if robot.image:
            await self.repository.delete_robot_image(robot.image.imageId)
        
        return await self.repository.delete_robot(robot.robotId)

    async def set_route(self, robot_id: int, sequence: int, waypoints: dict) -> RouteResponse:
        robot = await self.get_robot(robot_id)
        
        return RouteResponse(
            routeId=f"route_{datetime.now().timestamp()}",
            robotId=str(robot.robotId),
            courseSequence=sequence,
            waypoints=waypoints,
            createdAt=datetime.now()
        )

    async def get_map(self, robot_id: int) -> MapResponse:
        robot = await self.get_robot(robot_id)
        
        return MapResponse(
            robotId=str(robot.robotId),
            mapId=f"map_{robot.robotId}",
            currentLocation={"x": robot.position.x, "y": robot.position.y},
            mapData={}  # 실제 맵 데이터를 여기에 추가
        )

    async def update_status(self, robot_id: int, status: str) -> StatusResponse:
        robot = await self.get_robot(robot_id)
        
        update_data = {
            "status": status,
            "lastActive": datetime.now()
        }
        
        updated_robot = await self.repository.update_robot(robot.robotId, update_data)
        
        return StatusResponse(
            robotId=str(updated_robot.robotId),
            status=updated_robot.status,
            timestamp=updated_robot.lastActive,
            location=updated_robot.position
        )

    async def get_logs(self, robot_id: int, log_type: str) -> LogResponse:
        robot = await self.get_robot(robot_id)
        
        # 실제 로그 데이터를 가져오는 로직 구현 필요
        return LogResponse(
            robotId=str(robot.robotId),
            routes=[],  # 실제 경로 로그 데이터
            detections=[],  # 실제 감지 로그 데이터
            pagination={"total": 0, "page": 1, "size": 10}
        )

    async def update_robot_status(self, robot_id: int, new_status: str) -> None:
        """로봇의 상태를 업데이트합니다."""
        # 유효한 상태값 검증
        valid_statuses = ["IDLE", "RUNNING", "PAUSED", "ERROR", "CHARGING", "active"]  # active 상태 추가
        if new_status not in valid_statuses:
            raise HTTPException(
                status_code=400,
                detail=f"유효하지 않은 상태입니다. 가능한 상태: {', '.join(valid_statuses)}"
            )
        
        # 로봇 존재 여부 확인
        robot = await self.get_robot(robot_id)
        if not robot:
            raise HTTPException(
                status_code=404,
                detail="로봇을 찾을 수 없습니다."
            )
        
        # 상태 업데이트
        await self.repository.update_robot_status(robot_id, new_status)