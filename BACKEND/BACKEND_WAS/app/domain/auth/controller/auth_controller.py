from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from ..service.auth_service import AuthService
from ..models.auth_models import Token, User, PasswordChange, UserCreate, UserLogin
from ....common.models.responses import BaseResponse
from ....common.utils import create_success_response, create_error_response
from ..repository.auth_repository import AuthRepository
import logging

router = APIRouter()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/v1/auth/login")
auth_service = AuthService()
auth_repository = AuthRepository()
logger = logging.getLogger(__name__)

async def create_admin_user():
    """관리자 계정을 생성합니다."""
    try:
        # 먼저 admin 계정이 있는지 확인
        existing_admin = await auth_repository.find_user_by_username(username="admin")
        if existing_admin:
            logger.info("Admin 계정이 이미 존재합니다.")
            return


        logger.info("Admin 계정 생성 시도...")
        admin_data = UserCreate(
            username="admin",
            password="admin1234",
            role="admin"
        )
        await auth_service.create_user(admin_data)
        logger.info("Admin 계정이 성공적으로 생성되었습니다.")
    except Exception as e:
        logger.error(f"Admin 계정 생성 중 오류 발생: {str(e)}")
        if "duplicate key error" not in str(e).lower():
            logger.error(f"예상치 못한 오류: {str(e)}")
            raise e
        else:
            logger.info("중복된 Admin 계정이 감지되었습니다.")

async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    return await auth_service.verify_token(token)

@router.post("/login", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    user = await auth_service.authenticate_user(form_data.username, form_data.password)
    tokens = await auth_service.create_tokens(username=form_data.username)
    logger.info(tokens)

    return {
        "accessToken": tokens.access_token,
        "refreshToken": tokens.refresh_token,
        "tokenType": tokens.token_type
    }
#나중에 반드시 확인 필요. 리프레시 토큰이 들어가는지 아닌지 알아야함.
@router.post("/refresh", response_model=Token)
async def refresh_token(refresh_token: str):    
    tokens = await auth_service.refresh_tokens(refresh_token=refresh_token)

    return {
        "accessToken": tokens.access_token,
        "refreshToken": tokens.refresh_token,
        "tokenType": tokens.token_type
    }

@router.post("/change-password", response_model=BaseResponse,
    responses={401: {"description": "Not authenticated"}},
    summary="비밀번호 변경",
    description="사용자 비밀번호를 변경합니다.",
    tags=["auth"])
async def change_password(
    password_data: PasswordChange,
    current_user: User = Depends(get_current_user),
    token: str = Depends(oauth2_scheme)
):
    # 새 비밀번호와 확인 비밀번호가 일치하는지 확인
    if password_data.newPassword != password_data.confirmPassword:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="새 비밀번호와 확인 비밀번호가 일치하지 않습니다."
        )
        
    # 토큰 검증
    try:
        await auth_service.verify_token(token)
    except:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="유효하지 않은 토큰입니다.",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return await auth_service.change_password(
        current_user.username,
        password_data.currentPassword,
        password_data.newPassword,
        password_data.confirmPassword
    )

@router.get("/check-password-status", response_model=BaseResponse)
async def check_password_status(current_user: User = Depends(get_current_user)):
    return {
        "success": True,
        "data": {
            "isDefaultPassword": current_user.is_default_password
        }
    }

@router.post("/logout", response_model=BaseResponse,
    responses={401: {"description": "Not authenticated"}},
    summary="로그아웃",
    description="사용자 로그아웃을 처리합니다.",
    tags=["auth"])
async def logout(
    current_user: User = Depends(get_current_user),
    token: str = Depends(oauth2_scheme)
):
    # 토큰 검증
    try:
        await auth_service.verify_token(token)
    except:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="유효하지 않은 토큰입니다.",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return await auth_service.logout(current_user.username)