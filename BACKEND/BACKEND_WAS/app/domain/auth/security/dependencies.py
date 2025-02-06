from typing import Optional
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from .service import SecurityService
from .models import TokenData

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/v1/auth/login")
security_service = SecurityService()

async def get_current_user(token: str = Depends(oauth2_scheme)) -> Optional[str]:
    """현재 인증된 사용자의 username을 반환합니다."""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="인증 정보가 유효하지 않습니다",
        headers={"WWW-Authenticate": "Bearer"},
    )
    
    token_data = security_service.verify_token(token)
    if token_data is None:
        raise credentials_exception
        
    return token_data.username