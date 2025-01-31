from fastapi import APIRouter, Path, Query, HTTPException, Response
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from ..schemas.responses import BaseResponse, ErrorDetail
from ..database import db
import os

router = APIRouter(tags=["videos"])

class VideoResponse(BaseModel):
    session_id: str
    robot_id: Optional[str]
    start_time: datetime
    end_time: Optional[datetime]
    file_path: str
    frame_count: int
    status: str
    
class VideoListResponse(BaseModel):
    videos: List[VideoResponse]
    pagination: dict

@router.get("", response_model=BaseResponse[VideoListResponse])
async def get_videos(
    robot_id: Optional[str] = Query(None, description="로봇 ID로 필터링"),
    start_date: Optional[datetime] = Query(None, description="시작 날짜로 필터링"),
    end_date: Optional[datetime] = Query(None, description="종료 날짜로 필터링"),
    status: Optional[str] = Query(None, description="상태로 필터링 (recording, completed, error)"),
    page: int = Query(1, ge=1, description="페이지 번호"),
    limit: int = Query(10, ge=1, le=100, description="페이지당 항목 수")
):
    try:
        # 쿼리 조건 생성
        query = {}
        if robot_id:
            query["robot_id"] = robot_id
        if start_date:
            query["start_time"] = {"$gte": start_date}
        if end_date:
            query["end_time"] = {"$lte": end_date}
        if status:
            query["status"] = status

        # 총 개수 계산
        total = await db.videos.count_documents(query)
        
        # 페이지네이션 적용하여 비디오 조회
        videos = await db.videos.find(query)\
            .sort("start_time", -1)\
            .skip((page - 1) * limit)\
            .limit(limit)\
            .to_list(length=limit)

        return BaseResponse[VideoListResponse](
            status=200,
            success=True,
            message="녹화 영상 목록 조회 성공",
            data=VideoListResponse(
                videos=videos,
                pagination={
                    "total": total,
                    "page": page,
                    "size": limit,
                    "pages": (total + limit - 1) // limit
                }
            )
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/{session_id}", response_model=BaseResponse[VideoResponse])
async def get_video(
    session_id: str = Path(..., description="비디오 세션 ID")
):
    try:
        video = await db.videos.find_one({"session_id": session_id})
        if not video:
            raise HTTPException(
                status_code=404,
                detail="해당 영상을 찾을 수 없습니다."
            )

        return BaseResponse[VideoResponse](
            status=200,
            success=True,
            message="녹화 영상 조회 성공",
            data=video
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/{session_id}/stream")
async def stream_video(
    session_id: str = Path(..., description="비디오 세션 ID")
):
    try:
        video = await db.videos.find_one({"session_id": session_id})
        if not video:
            raise HTTPException(
                status_code=404,
                detail="해당 영상을 찾을 수 없습니다."
            )

        if not os.path.exists(video["file_path"]):
            raise HTTPException(
                status_code=404,
                detail="영상 파일을 찾을 수 없습니다."
            )

        # 비디오 스트리밍을 위한 응답 생성
        return Response(
            content=open(video["file_path"], "rb").read(),
            media_type="video/mp4"
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.delete("/{session_id}", response_model=BaseResponse[None])
async def delete_video(
    session_id: str = Path(..., description="비디오 세션 ID")
):
    try:
        video = await db.videos.find_one({"session_id": session_id})
        if not video:
            raise HTTPException(
                status_code=404,
                detail="해당 영상을 찾을 수 없습니다."
            )

        # 파일 시스템에서 영상 파일 삭제
        if os.path.exists(video["file_path"]):
            os.remove(video["file_path"])

        # 데이터베이스에서 레코드 삭제
        await db.videos.delete_one({"session_id": session_id})

        return BaseResponse[None](
            status=200,
            success=True,
            message="녹화 영상이 성공적으로 삭제되었습니다."
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/{session_id}/download")
async def download_video(
    session_id: str = Path(..., description="비디오 세션 ID")
):
    try:
        video = await db.videos.find_one({"session_id": session_id})
        if not video:
            raise HTTPException(
                status_code=404,
                detail="해당 영상을 찾을 수 없습니다."
            )

        if not os.path.exists(video["file_path"]):
            raise HTTPException(
                status_code=404,
                detail="영상 파일을 찾을 수 없습니다."
            )

        # 다운로드를 위한 응답 생성
        return Response(
            content=open(video["file_path"], "rb").read(),
            media_type="video/mp4",
            headers={
                "Content-Disposition": f"attachment; filename={session_id}.mp4"
            }
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e)) 