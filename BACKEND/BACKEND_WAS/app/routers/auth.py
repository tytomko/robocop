# backend/app/routers/auth.py
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from jose import JWTError, jwt
from ..core.security import (
    SECRET_KEY,
    ALGORITHM,
    create_access_token,
    create_refresh_token,
    verify_password,
    get_password_hash
)
from ..database import db
from ..models.user import Token, User, PasswordChange, TokenData
from ..schemas.responses import BaseResponse
from datetime import datetime

router = APIRouter()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/v1/auth/login")

# 초기 관리자 계정 생성 함수
async def create_admin_user():
    # 관리자 계정이 없는 경우에만 생성
    if not await db.users.find_one({"username": "admin"}):
        admin_user = {
            "username": "admin",
            "hashedPassword": get_password_hash("admin"),
            "role": "admin",
            "isActive": True,
            "createdAt": datetime.now()
        }
        await db.users.insert_one(admin_user)

async def get_current_user(token: str = Depends(oauth2_scheme)):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
        token_data = TokenData(username=username)
    except JWTError:
        raise credentials_exception
    
    user = await db.users.find_one({"username": token_data.username})
    if user is None:
        raise credentials_exception
    return user

@router.post("/login", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    # 사용자 확인
    user = await db.users.find_one({"username": form_data.username})
    
    if not user or not verify_password(form_data.password, user["hashedPassword"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # 토큰 생성
    access_token = create_access_token(
        data={"sub": user["username"]}
    )
    refresh_token = create_refresh_token(
        data={"sub": user["username"]}
    )

    # Refresh 토큰을 DB에 저장하고 lastLogin 업데이트
    await db.users.update_one(
        {"username": user["username"]},
        {
            "$set": {
                "refreshToken": refresh_token,
                "lastLogin": datetime.now(),
                "updatedAt": datetime.now()
            }
        }
    )

    return {
        "accessToken": access_token,
        "refreshToken": refresh_token,
        "tokenType": "bearer"
    }

@router.post("/refresh", response_model=Token)
async def refresh_token(current_token: str):
    try:
        # Refresh 토큰 검증
        payload = jwt.decode(current_token, SECRET_KEY, algorithms=[ALGORITHM])
        username = payload.get("sub")
        
        if username is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid refresh token"
            )

        # DB에서 사용자의 refresh 토큰 확인
        user = await db.users.find_one({"username": username})
        if not user or user.get("refreshToken") != current_token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid refresh token"
            )

        # 기존 refresh token 삭제
        await db.users.update_one(
            {"username": username},
            {"$set": {"refreshToken": None}}
        )

        # 새로운 토큰 생성
        new_access_token = create_access_token(data={"sub": username})
        new_refresh_token = create_refresh_token(data={"sub": username})

        # 새로운 refresh 토큰을 DB에 저장
        await db.users.update_one(
            {"username": username},
            {
                "$set": {
                    "refreshToken": new_refresh_token,
                    "updatedAt": datetime.now()
                }
            }
        )

        return {
            "accessToken": new_access_token,
            "refreshToken": new_refresh_token,
            "tokenType": "bearer"
        }

    except JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid refresh token"
        )

@router.post("/change-password", response_model=BaseResponse)
async def change_password(
    password_data: PasswordChange,
    current_user: User = Depends(get_current_user)
):
    try:
        # 현재 비밀번호 확인 (current_user["hashedPassword"] 유지 - DB 필드명)
        if not verify_password(password_data.currentPassword, current_user["hashedPassword"]):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="현재 비밀번호가 일치하지 않습니다."
            )
        
        # 새 비밀번호 확인
        if password_data.newPassword != password_data.confirmPassword:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="새 비밀번호가 일치하지 않습니다."
            )
        
        # 비밀번호 업데이트
        hashed_password = get_password_hash(password_data.newPassword)
        await db.users.update_one(
            {"username": current_user["username"]},  # username은 그대로 유지
            {
                "$set": {
                    "hashedPassword": hashed_password,     # password는 DB 필드명이므로 유지
                    "isDefaultPassword": False,
                    "updatedAt": datetime.now()
                }
            }
        )
        
        return {
            "success": True,
            "message": "비밀번호가 성공적으로 변경되었습니다."
        }
        
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=str(e)
        )

@router.get("/check-password-status", response_model=BaseResponse)
async def check_password_status(current_user: User = Depends(get_current_user)):
    return {
        "success": True,
        "data": {
            "isDefaultPassword": current_user["isDefaultPassword"]  # is_default_password -> isDefaultPassword
        }
    }

@router.post("/logout", response_model=BaseResponse)
async def logout(current_user = Depends(get_current_user)):
    try:
        # 리프레시 토큰 무효화
        await db.users.update_one(
            {"username": current_user["username"]},  # username은 그대로 유지
            {
                "$set": {
                    "refreshToken": None,
                    "lastLogout": datetime.now()
                }
            }
        )
        
        return BaseResponse(
            status=200,
            success=True,
            message="로그아웃 되었습니다."
        )
    except Exception as e:
        raise HTTPException(
            status_code=400,
            detail=str(e)
        )