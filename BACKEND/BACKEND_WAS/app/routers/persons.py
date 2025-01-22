from fastapi import APIRouter, Path, File, UploadFile, Form, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from ..schemas.responses import BaseResponse, ErrorDetail
from ..database import db, persons  # persons 컬렉션 import
from ..models.person import Person, PersonCreate, ImageInfo
import os
import uuid

router = APIRouter(prefix="/api/v1/persons", tags=["persons"])

# 이미지 저장 경로 설정
UPLOAD_DIR = "storage/persons"
os.makedirs(UPLOAD_DIR, exist_ok=True)

class PersonResponse(BaseModel):
    personId: str
    label: str
    images: List[ImageInfo]
    createdAt: Optional[datetime] = None
    updatedAt: Optional[datetime] = None

class PersonUpdate(BaseModel):
    person_id: int
    person_label: str

@router.post("", response_model=BaseResponse[Person])
async def create_person(
    name: str = Form(...),
    department: Optional[str] = Form(None),
    position: Optional[str] = Form(None),
    image: UploadFile = File(...)
):
    try:
        # 이미지 유효성 검사
        ext = os.path.splitext(image.filename)[1].lower()
        if ext not in ['.jpg', '.jpeg', '.png']:
            raise HTTPException(
                status_code=400,
                detail="지원하지 않는 이미지 형식입니다."
            )
        
        # person_id 생성
        counter = await db.counters.find_one_and_update(
            {"_id": "person_id"},
            {"$inc": {"seq": 1}},
            upsert=True,
            return_document=True
        )
        person_id = counter["seq"]
        
        # 이미지 저장
        image_id = f"person_{person_id}_{uuid.uuid4()}{ext}"
        image_path = os.path.join(UPLOAD_DIR, image_id)
        
        with open(image_path, "wb") as buffer:
            content = await image.read()
            buffer.write(content)
        
        # 이미지 정보 생성
        image_info = ImageInfo(
            imageId=image_id,
            url=f"/storage/persons/{image_id}",
            uploadedAt=datetime.now()
        )
        
        # Person 데이터 생성
        person_data = {
            "person_id": person_id,
            "name": name,
            "department": department,
            "position": position,
            "images": [image_info.dict()],
            "created_at": datetime.now()
        }
        
        # 데이터베이스에 저장
        result = await persons.insert_one(person_data)
        
        if not result.inserted_id:
            # 저장 실패 시 이미지 삭제
            if os.path.exists(image_path):
                os.remove(image_path)
            raise HTTPException(
                status_code=500,
                detail="데이터베이스 저장에 실패했습니다."
            )
        
        return BaseResponse[Person](
            status=201,
            success=True,
            message="신원 정보가 성공적으로 등록되었습니다.",
            data=Person(**person_data)
        )
        
    except Exception as e:
        # 에러 발생 시 이미지 삭제
        if 'image_path' in locals() and os.path.exists(image_path):
            try:
                os.remove(image_path)
            except:
                pass
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

@router.get("", response_model=BaseResponse[List[Person]])
async def get_persons(
    department: Optional[str] = None,
    position: Optional[str] = None
):
    try:
        # 필터 조건 설정
        filter_query = {}
        if department:
            filter_query["department"] = department
        if position:
            filter_query["position"] = position
            
        # MongoDB에서 조회
        cursor = persons.find(filter_query)
        person_list = []
        async for doc in cursor:
            person_list.append(Person(**doc))
            
        return BaseResponse[List[Person]](
            status=200,
            success=True,
            message="신원 정보 목록을 성공적으로 조회했습니다.",
            data=person_list
        )
    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

@router.post("/{person_id}/images", response_model=BaseResponse[Person])
async def add_person_image(
    person_id: int,
    image: UploadFile = File(...)
):
    try:
        # 사용자 존재 확인
        person = await persons.find_one({"person_id": person_id})
        if not person:
            raise HTTPException(
                status_code=404,
                detail="해당 사용자를 찾을 수 없습니다."
            )
        
        # 이미지 처리
        ext = os.path.splitext(image.filename)[1].lower()
        if ext not in ['.jpg', '.jpeg', '.png']:
            raise HTTPException(
                status_code=400,
                detail="지원하지 않는 이미지 형식입니다."
            )
        
        # 이미지 저장
        image_id = f"person_{person_id}_{uuid.uuid4()}{ext}"
        image_path = os.path.join(UPLOAD_DIR, image_id)
        
        with open(image_path, "wb") as buffer:
            content = await image.read()
            buffer.write(content)
        
        # 이미지 정보 생성
        image_info = ImageInfo(
            imageId=image_id,
            url=f"/storage/persons/{image_id}",
            uploadedAt=datetime.now()
        )
        
        # 데이터베이스 업데이트
        result = await persons.update_one(
            {"person_id": person_id},
            {
                "$push": {"images": image_info.dict()},
                "$set": {"updated_at": datetime.now()}
            }
        )
        
        if result.modified_count == 0:
            if os.path.exists(image_path):
                os.remove(image_path)
            raise HTTPException(
                status_code=500,
                detail="이미지 정보 업데이트에 실패했습니다."
            )
        
        # 업데이트된 사용자 정보 조회
        updated_person = await persons.find_one({"person_id": person_id})
        
        return BaseResponse[Person](
            status=200,
            success=True,
            message="이미지가 성공적으로 추가되었습니다.",
            data=Person(**updated_person)
        )
        
    except Exception as e:
        # 에러 발생 시 이미지 삭제
        if 'image_path' in locals() and os.path.exists(image_path):
            try:
                os.remove(image_path)
            except:
                pass
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )

@router.put("/{id}", response_model=BaseResponse[PersonResponse])
async def update_person(
    id: int = Path(..., description="Person ID"),
    person: PersonUpdate = None
):
    try:
        return BaseResponse[PersonResponse](
            status=200,
            success=True,
            message="신원 정보 업데이트 성공",
            data=PersonResponse(
                personId=f"person_{id}",
                label="John Doe Updated",
                images=[
                    ImageInfo(
                        imageId="img_002",
                        url="/storage/faces/img_002.jpg",
                        uploadedAt=datetime.now()
                    )
                ],
                updatedAt=datetime.now()
            )
        )
    except Exception:
        return BaseResponse[PersonResponse](
            status=404,
            success=False,
            message="신원 정보 업데이트 실패",
            errors=[
                ErrorDetail(
                    field="personId",
                    message="해당 사용자를 찾을 수 없습니다"
                )
            ]
        )

@router.delete("/{id}", response_model=BaseResponse[dict])
async def delete_person(id: int = Path(..., description="Person ID")):
    try:
        return BaseResponse[dict](
            status=200,
            success=True,
            message="신원 정보 삭제 완료",
            data={
                "personId": f"person_{id}",
                "deletedAt": datetime.now().isoformat()
            }
        )
    except Exception:
        return BaseResponse[dict](
            status=404,
            success=False,
            message="신원 정보 삭제 실패",
            errors=[
                ErrorDetail(
                    field="personId",
                    message="해당 사용자를 찾을 수 없습니다"
                )
            ]
        ) 