from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routers import auth, robots, persons
from .database import init_db
import uvicorn
import os

app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 실제 운영 환경에서는 특정 도메인만 허용하도록 수정
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 라우터 등록
app.include_router(auth.router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(robots.router, prefix="/api/v1/robots", tags=["robots"])
app.include_router(persons.router, tags=["persons"])

@app.on_event("startup")
async def startup_event():
    await init_db()
    await auth.create_admin_user()

@app.get("/")
async def root():
    return {"message": "Robot Management API"}

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port, reload=False) 