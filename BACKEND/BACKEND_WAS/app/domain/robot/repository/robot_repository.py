import os
import uuid
from datetime import datetime
from typing import List, Optional
from ....infrastructure.database.connection import DatabaseConnection
from ..models.robot_models import Robot, RobotImage

UPLOAD_DIR = "storage/robots"
os.makedirs(UPLOAD_DIR, exist_ok=True)

class RobotRepository:
    def __init__(self):
        self.db = DatabaseConnection.get_db()

    async def create_robot(self, robot: Robot) -> Robot:
        await self.db.robots.insert_one(robot.dict())
        return robot

    async def find_all_robots(self) -> List[Robot]:
        robots = await self.db.robots.find().to_list(length=None)
        return [Robot(**robot) for robot in robots]

    async def find_robot_by_id(self, robot_id: int) -> Optional[Robot]:
        robot = await self.db.robots.find_one({"robotId": robot_id})
        return Robot(**robot) if robot else None

    async def find_robot_by_name(self, name: str) -> Optional[Robot]:
        robot = await self.db.robots.find_one({"name": name})
        return Robot(**robot) if robot else None

    async def update_robot(self, robot_id: int, update_data: dict) -> Optional[Robot]:
        update_data["updatedAt"] = datetime.now()
        result = await self.db.robots.find_one_and_update(
            {"robotId": robot_id},
            {"$set": update_data},
            return_document=True
        )
        return Robot(**result) if result else None
    #소프트딜리트 처리 필요
    async def delete_robot(self, robot_id: int) -> bool:
        result = await self.db.robots.delete_one({"robotId": robot_id})
        return result.deleted_count > 0

    async def get_next_robot_id(self) -> int:
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