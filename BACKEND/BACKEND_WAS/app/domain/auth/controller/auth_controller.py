from fastapi import APIRouter, Depends, HTTPException
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from ..service.auth_service import AuthService
from ..models.auth_models import Token, User, PasswordChange, UserCreate, UserLogin
from ....common.models.responses import BaseResponse
from ....common.utils import create_success_response, create_error_response

router = APIRouter()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/v1/auth/login")
auth_service = AuthService()

async def create_admin_user():
    """관리자 계정을 생성합니다."""
    try:
        admin_data = UserCreate(
            username="admin",
            password="admin1234",
            role="admin"
        )
        await auth_service.create_user(admin_data)
    except Exception as e:
        if "duplicate key error" not in str(e).lower():
            raise e

async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    return await auth_service.verify_token(token)

@router.post("/login", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    user = await auth_service.authenticate_user(form_data.username, form_data.password)
    tokens = await auth_service.create_tokens(form_data.username)
    return {
        "accessToken": tokens.access_token,
        "refreshToken": tokens.refresh_token,
        "tokenType": tokens.token_type
    }

@router.post("/refresh", response_model=Token)
async def refresh_token(current_token: str):
    tokens = await auth_service.refresh_tokens(current_token)
    return {
        "accessToken": tokens.access_token,
        "refreshToken": tokens.refresh_token,
        "tokenType": tokens.token_type
    }

@router.post("/change-password", response_model=BaseResponse)
async def change_password(
    password_data: PasswordChange,
    current_user: User = Depends(get_current_user)
):
    return await auth_service.change_password(
        current_user["username"],
        password_data.currentPassword,
        password_data.newPassword,
        password_data.confirmPassword
    )

@router.get("/check-password-status", response_model=BaseResponse)
async def check_password_status(current_user: User = Depends(get_current_user)):
    return {
        "success": True,
        "data": {
            "isDefaultPassword": current_user["isDefaultPassword"]
        }
    }

@router.post("/logout", response_model=BaseResponse)
async def logout(current_user: User = Depends(get_current_user)):
    return await auth_service.logout(current_user["username"])