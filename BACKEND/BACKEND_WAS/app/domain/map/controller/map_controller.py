from fastapi import APIRouter
from ..service.map_service import PointService
from ..models.map_models import PointsData

router = APIRouter()

@router.get("/map", response_model=PointsData)
async def get_points():
    return await PointService.get_points()