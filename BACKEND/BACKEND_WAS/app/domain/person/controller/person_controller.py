from typing import List, Optional
from fastapi import APIRouter, File, Form, UploadFile, Path, HTTPException
from ..models.person_models import Person, PersonCreate, PersonUpdate, ImageInfo
from ..service.person_service import PersonService
from ....common.utils import create_success_response, create_error_response
import uuid

router = APIRouter()
person_service = PersonService()

@router.on_event("startup")
async def startup_event():
    """시작 시 초기화"""
    await person_service.initialize()

@router.post("")
async def create_person(
    personName: str = Form(...),
    personPosition: str = Form(...), 
    personDepartment: Optional[str] = Form(None),
    personPhone: Optional[str] = Form(None),
    image: UploadFile = File(...)
):
    """새로운 Person을 생성합니다."""
    try:
        person_data = PersonCreate(
            name=personName,
            department=personDepartment,
            position=personPosition,
            phone=personPhone
        )

        person = await person_service.create_person(person_data, image)
        return create_success_response(
            data=person,
            message="신원 정보와 이미지가 성공적으로 등록되었습니다"
        )
    except Exception as e:
        return create_error_response(str(e))
    
    
@router.get("")
async def get_persons():
    """Person 목록을 조회합니다."""
    try:
        # 서비스 레이어를 통해 데이터를 조회하는 것이 더 좋습니다.
        # 1. 비즈니스 로직을 서비스에서 처리할 수 있음
        # 2. 레포지토리 의존성을 서비스에서 관리
        # 3. 컨트롤러는 HTTP 요청/응답만 처리하도록 책임 분리
        persons = await person_service.get_all_persons()
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
        person = await person_service.add_person_image(person_id, image)
        return create_success_response(
            data=person,
            message="이미지가 성공적으로 추가되었습니다"
        )
    except HTTPException as e:
        return create_error_response(str(e))
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

@router.delete("/{identifier}")
async def delete_person(
    identifier: str = Path(..., description="Person ID 또는 이름")
):
    """Person을 삭제합니다."""
    try:
        # identifier가 숫자면 ID로, 아니면 이름으로 처리
        if identifier.isdigit():
            await person_service.delete_person(int(identifier))
        else:
            await person_service.delete_person_by_name(identifier)
            
        return create_success_response(
            message="신원 정보가 성공적으로 삭제되었습니다"
        )
    except HTTPException as e:
        return create_error_response(str(e))
    except Exception as e:
        return create_error_response(str(e))