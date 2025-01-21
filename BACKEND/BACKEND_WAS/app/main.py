from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .database import test_connection
from .routers.auth import router as auth_router, create_admin_user
from .routers.schedules import router as schedules_router
from .routers.robots import router as robots_router
from .routers.persons import router as persons_router
from .config import get_settings

app = FastAPI()
settings = get_settings()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 실제 운영 환경에서는 구체적인 origin을 지정해야 합니다
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.on_event("startup")
async def startup_event():
    await test_connection()
    await create_admin_user()

# 라우터 등록
app.include_router(auth_router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(schedules_router, prefix="/api/v1", tags=["schedules"])
app.include_router(robots_router, prefix="/api/v1", tags=["robots"])
app.include_router(persons_router, prefix="/api/v1", tags=["persons"])

@app.get("/")
async def root():
    return {"message": "Robot Management API"}

@app.on_event("startup")
async def startup_event():
    # MongoDB 연결 테스트
    if not await test_connection():
        import sys
        sys.exit(1)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host=settings.HOST, port=settings.PORT, reload=True) 