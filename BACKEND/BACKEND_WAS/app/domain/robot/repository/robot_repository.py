import os
import uuid
from datetime import datetime
from typing import List, Optional, Union
from ....infrastructure.database.connection import DatabaseConnection
from ..models.robot_models import Robot, RobotImage, RobotStatus
from fastapi import HTTPException
import traceback
import logging

UPLOAD_DIR = "storage/robots"
os.makedirs(UPLOAD_DIR, exist_ok=True)

logger = logging.getLogger(__name__)

class RobotRepository:
    def __init__(self):
        self.db = None

    async def initialize(self):
        if self.db is None:
            try:
                self.db = await DatabaseConnection.get_db()
                if self.db is None:
                    raise Exception("데이터베이스 연결을 가져올 수 없습니다.")
            except Exception as e:
                raise HTTPException(
                    status_code=500,
                    detail=f"데이터베이스 초기화 실패: {str(e)}"
                )

    async def create_robot(self, robot: Robot) -> Robot:
        await self.initialize()  # DB 연결 확인
        robot_dict = robot.dict()
        result = await self.db.robots.insert_one(robot_dict)
        robot_dict['_id'] = result.inserted_id
        return Robot(**robot_dict)

    async def find_all_robots(self) -> List[Robot]:
        await self.initialize()
        try:
            # 삭제되지 않은 로봇만 조회
            robots = await self.db.robots.find(
                {"IsDeleted": False}
            ).to_list(length=None)
            
            return [Robot(**robot) for robot in robots]
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 목록 조회 실패: {str(e)}"
            )

    async def find_robot_by_id(self, robot_id: int) -> Optional[Robot]:
        await self.initialize()  # DB 연결 확인
        robot = await self.db.robots.find_one({"robotId": robot_id})
        return Robot(**robot) if robot else None

    async def find_robot_by_name(self, name: str) -> Optional[Robot]:
        await self.initialize()
        robot = await self.db.robots.find_one({"manufactureName": name})
        return Robot(**robot) if robot else None

    async def find_robot_by_nickname(self, nickname: str) -> Optional[Robot]:
        await self.initialize()
        robot = await self.db.robots.find_one({"nickname": nickname})
        return Robot(**robot) if robot else None

    async def update_robot(self, robot_id: int, update_data: dict) -> Optional[Robot]:
        await self.initialize()  # DB 연결 확인
        update_data["updatedAt"] = datetime.now()
        result = await self.db.robots.find_one_and_update(
            {"robotId": robot_id},
            {"$set": update_data},
            return_document=True
        )
        return Robot(**result) if result else None

    async def delete_robot(self, seq: int) -> bool:
        await self.initialize()
        try:
            result = await self.db.robots.update_one(
                {"seq": seq, "IsDeleted": False},
                {
                    "$set": {
                        "IsDeleted": True,
                        "DeletedAt": datetime.now(),
                        "updatedAt": datetime.now()
                    }
                }
            )
            return result.modified_count > 0
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 삭제 실패: {str(e)}"
            )

    async def get_next_robot_id(self) -> int:
        await self.initialize()  # DB 연결 확인
        counter = await self.db.counters.find_one_and_update(
            {"_id": "robot_id"},
            {"$inc": {"seq": 1}},
            upsert=True,
            return_document=True
        )
        return counter["seq"]

    async def save_robot_image(self, image_file, robot_id: int) -> Optional[RobotImage]:
        try:
            ext = os.path.splitext(image_file.filename)[1].lower()
            if ext not in ['.jpg', '.jpeg', '.png']:
                return None

            image_id = f"robot_{robot_id}_{uuid.uuid4()}{ext}"
            image_path = os.path.join(UPLOAD_DIR, image_id)

            content = await image_file.read()
            if not content:
                return None

            with open(image_path, "wb") as buffer:
                buffer.write(content)

            return RobotImage(
                imageId=image_id,
                url=f"/storage/robots/{image_id}",
                createdAt=datetime.now()
            )
        except Exception as e:
            print(f"이미지 저장 중 오류 발생: {str(e)}")
            if 'image_path' in locals() and os.path.exists(image_path):
                os.remove(image_path)
            return None

    async def delete_robot_image(self, image_id: str) -> bool:
        try:
            image_path = os.path.join(UPLOAD_DIR, image_id)
            if os.path.exists(image_path):
                os.remove(image_path)
                return True
            return False
        except Exception as e:
            print(f"이미지 삭제 중 오류 발생: {str(e)}")
            return False

    async def get_robot_id_by_nickname(self, nickname: str) -> Optional[int]:
        await self.initialize()
        robot = await self.db.robots.find_one(
            {"nickname": nickname, "IsDeleted": False},
            projection={"seq": 1}
        )
        return robot["seq"] if robot else None

    async def find_robot_by_identifier(self, identifier: Union[str, int]) -> Optional[Robot]:
        await self.initialize()
        if isinstance(identifier, int):
            query = {"seq": identifier, "IsDeleted": False}
        else:
            query = {"nickname": identifier, "IsDeleted": False}
        
        robot = await self.db.robots.find_one(query)
        return Robot(**robot) if robot else None
    

    async def update_robot_waypoints(self, seq: int, waypoints: List[dict]) -> bool:
        """로봇의 경유점을 업데이트하고 상태를 변경합니다."""
        await self.initialize()
        try:
            # 현재 로봇 상태 확인
            robot = await self.find_robot_by_identifier(seq)
            if not robot:
                raise HTTPException(
                    status_code=404,
                    detail="로봇을 찾을 수 없습니다."
                )

            # 로봇이 업데이트 가능한 상태인지 확인
            if robot.status in [RobotStatus.ERROR, RobotStatus.EMERGENCYSTOPPED]:
                raise HTTPException(
                    status_code=400,
                    detail=f"현재 로봇 상태({robot.status})에서는 경유점을 변경할 수 없습니다."
                )

            # 경유점 개수에 따른 상태 결정
            new_status = RobotStatus.WAITING
            if waypoints:
                new_status = RobotStatus.NAVIGATING if len(waypoints) == 1 else RobotStatus.PATROLLING

            # 업데이트 실행
            result = await self.db.robots.update_one(
                {"seq": seq, "IsDeleted": False},
                {
                    "$set": {
                        "waypoints": waypoints,
                        "status": new_status,
                        "lastActive": datetime.now(),
                        "updatedAt": datetime.now()
                    }
                }
            )
            
            if result.modified_count == 0:
                raise HTTPException(
                    status_code=500,
                    detail="로봇 정보 업데이트에 실패했습니다."
                )
            
            return True

        except HTTPException as e:
            raise e
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 경유점 업데이트 실패: {str(e)}"
            )

    async def update_robot_status(self, seq: int, update_data: dict) -> Optional[Robot]:
        """로봇의 상태를 업데이트합니다."""
        await self.initialize()
        try:
            result = await self.db.robots.find_one_and_update(
                {"seq": seq, "IsDeleted": False},
                {"$set": {
                    "status": update_data["status"],
                    "lastActive": update_data["lastActive"],
                    "updatedAt": datetime.now()
                }},
                return_document=True
            )
            
            if not result:
                raise HTTPException(
                    status_code=404,
                    detail="로봇을 찾을 수 없습니다."
                )
            
            return Robot(**result)
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 상태 업데이트 실패: {str(e)}"
            )

    async def update_robot_nickname(self, seq: int, new_nickname: str) -> Optional[Robot]:
        """로봇의 닉네임을 업데이트합니다."""
        await self.initialize()
        try:
            result = await self.db.robots.find_one_and_update(
                {"seq": seq, "IsDeleted": False},
                {"$set": {
                    "nickname": new_nickname,
                    "updatedAt": datetime.now()
                }},
                return_document=True
            )
            
            if not result:
                raise HTTPException(
                    status_code=404,
                    detail="로봇을 찾을 수 없습니다."
                )
            
            return Robot(**result)
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"로봇 닉네임 업데이트 실패: {str(e)}"
            )

    async def save_robot_logs(self, logs: list) -> bool:
        """로봇 로그를 저장합니다."""
        await self.initialize()
        try:
            if logs:
                result = await self.db.robotlogs.insert_many(logs)
                logger.info(f"{len(result.inserted_ids)}개 로그 저장 완료")
                return True
            return False
        except Exception as e:
            logger.error(f"로봇 로그 저장 중 오류 발생: {str(e)}")
            logger.error(traceback.format_exc())
            return False
