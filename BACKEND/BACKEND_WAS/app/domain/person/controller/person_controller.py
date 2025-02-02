from typing import List, Optional
from fastapi import APIRouter, File, Form, UploadFile, Path, HTTPException
from ..models.person_models import Person, PersonCreate, PersonUpdate, ImageInfo
from ..service.person_service import PersonService
from ....common.utils import create_success_response, create_error_response
import uuid

router = APIRouter(prefix="/api/v1/persons", tags=["persons"])
person_service = PersonService()

@router.on_event("startup")
async def startup_event():
    """시작 시 초기화"""
    await person_service.initialize()

@router.post("")
async def create_person(
    name: str = Form(...),
    department: Optional[str] = Form(None),
    position: Optional[str] = Form(None),
    email: Optional[str] = Form(None),
    phone: Optional[str] = Form(None),
    image: UploadFile = File(...)
):
    """새로운 Person을 생성합니다."""
    try:
        person_data = PersonCreate(
            name=name,
            department=department,
            position=position,
            email=email,
            phone=phone
        )
        person = await person_service.create_person(person_data, image)
        return create_success_response(
            data=person,
            message="신원 정보가 성공적으로 등록되었습니다"
        )
    except Exception as e:
        return create_error_response(str(e))

@router.get("")
async def get_persons(
    department: Optional[str] = None,
    position: Optional[str] = None
):
    """Person 목록을 조회합니다."""
    try:
        persons = await person_service.get_persons(department, position)
        return create_success_response(
            data=persons,
            message="신원 정보 목록을 성공적으로 조회했습니다"
        )
    except Exception as e:
        return create_error_response(str(e))

@router.post("/{person_id}/images")
async def add_person_image(
    person_id: int = Path(..., description="Person ID"),
    image: UploadFile = File(...)
):
    """Person에 이미지를 추가합니다."""
    try:
        image_info = ImageInfo(
            image_id=str(uuid.uuid4()),
            file_name=image.filename,
            file_path=f"storage/images/{person_id}/{image.filename}",
            file_size=0,  # 파일 크기는 저장 후 업데이트
            content_type=image.content_type
        )
        
        person = await person_service.add_person_image(person_id, image_info)
        if not person:
            raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")
        return create_success_response(
            data=person,
            message="이미지가 성공적으로 추가되었습니다"
        )
    except Exception as e:
        return create_error_response(str(e))

@router.put("/{person_id}")
async def update_person(
    person_id: int = Path(..., description="Person ID"),
    name: str = Form(...),
    department: Optional[str] = Form(None),
    position: Optional[str] = Form(None),
    email: Optional[str] = Form(None),
    phone: Optional[str] = Form(None)
):
    """Person 정보를 업데이트합니다."""
    try:
        person_data = PersonUpdate(
            name=name,
            department=department,
            position=position,
            email=email,
            phone=phone
        )
        person = await person_service.update_person(person_id, person_data)
        if not person:
            raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")
        return create_success_response(
            data=person,
            message="신원 정보가 성공적으로 업데이트되었습니다"
        )
    except Exception as e:
        return create_error_response(str(e))

@router.delete("/{person_id}")
async def delete_person(
    person_id: int = Path(..., description="Person ID")
):
    """Person을 삭제합니다."""
    try:
        success = await person_service.delete_person(person_id)
        if not success:
            raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")
        return create_success_response(
            message="신원 정보가 성공적으로 삭제되었습니다"
        )
    except Exception as e:
        return create_error_response(str(e))