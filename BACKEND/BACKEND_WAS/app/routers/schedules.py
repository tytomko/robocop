from fastapi import APIRouter, HTTPException
from typing import List
from ..database import db, schedules
from ..models.schedule import Schedule, ScheduleCreate
from ..schemas.responses import BaseResponse

router = APIRouter(
    prefix="/schedules",
    tags=["schedules"]
)

@router.post("", response_model=BaseResponse[Schedule])
async def create_schedule(schedule_data: ScheduleCreate):
    try:
        # 시퀀스 값 증가
        counter = await db.counters.find_one_and_update(
            {"_id": "schedule_id"},
            {"$inc": {"seq": 1}},
            upsert=True,
            return_document=True
        )
        
        schedule_dict = {
            "schedule_id": counter["seq"],
            "days": [day for day in schedule_data.days],
            "start_time": schedule_data.start_time.isoformat(),
            "end_time": schedule_data.end_time.isoformat(),
            "is_active": True,
            "description": schedule_data.description
        }
        
        await schedules.insert_one(schedule_dict)
        
        return {
            "success": True,
            "message": "스케줄이 성공적으로 생성되었습니다.",
            "data": Schedule(**schedule_dict)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("", response_model=BaseResponse[List[Schedule]])
async def get_schedules():
    try:
        schedules_list = await schedules.find().to_list(length=None)
        return {
            "success": True,
            "message": "스케줄 목록을 성공적으로 조회했습니다.",
            "data": [Schedule(**schedule) for schedule in schedules_list]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.put("/{schedule_id}", response_model=BaseResponse[Schedule])
async def update_schedule(schedule_id: int, schedule_data: ScheduleCreate):
    try:
        schedule_dict = {
            "days": [day for day in schedule_data.days],
            "start_time": schedule_data.start_time.isoformat(),
            "end_time": schedule_data.end_time.isoformat(),
            "description": schedule_data.description
        }
        
        result = await schedules.find_one_and_update(
            {"schedule_id": schedule_id},
            {"$set": schedule_dict},
            return_document=True
        )
        
        if not result:
            raise HTTPException(status_code=404, detail="스케줄을 찾을 수 없습니다.")
        
        return {
            "success": True,
            "message": "스케줄이 성공적으로 수정되었습니다.",
            "data": Schedule(**result)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/{schedule_id}", response_model=BaseResponse)
async def delete_schedule(schedule_id: int):
    try:
        result = await schedules.delete_one({"schedule_id": schedule_id})
        if result.deleted_count == 0:
            raise HTTPException(status_code=404, detail="스케줄을 찾을 수 없습니다.")
        
        return {
            "success": True,
            "message": "스케줄이 성공적으로 삭제되었습니다."
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) 