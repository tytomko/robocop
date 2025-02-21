import pytest
from fastapi import FastAPI
from httpx import AsyncClient
from ..main import app
from ..common.config.manager import get_settings
from ..domain.auth.security.service import SecurityService
from ..domain.person.service import PersonService

settings = get_settings()
security_service = SecurityService()
person_service = PersonService()

@pytest.mark.asyncio
async def test_auth_flow():
    """인증 플로우 통합 테스트"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # 회원가입
        register_data = {
            "email": "test@example.com",
            "password": "Test1234!",
            "name": "Test User"
        }
        response = await client.post("/api/v1/auth/register", json=register_data)
        assert response.status_code in [201, 409]  # 409는 이미 존재하는 경우
        
        # 로그인
        login_data = {
            "username": register_data["email"],
            "password": register_data["password"]
        }
        response = await client.post("/api/v1/auth/login", json=login_data)
        assert response.status_code == 200
        token = response.json()["access_token"]
        
        # 프로필 조회
        headers = {"Authorization": f"Bearer {token}"}
        response = await client.get("/api/v1/auth/profile", headers=headers)
        assert response.status_code == 200
        assert response.json()["email"] == register_data["email"]

@pytest.mark.asyncio
async def test_robot_camera_integration():
    """로봇-카메라 통합 테스트"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # 카메라 초기화
        camera_config = {
            "camera_topic": "/test/camera",
            "camera_name": "test_camera"
        }
        response = await client.post("/api/v1/cameras/initialize", json=camera_config)
        assert response.status_code in [200, 503]
        
        if response.status_code == 200:
            # 로봇에 카메라 스트림 연결
            response = await client.post("/api/v1/robots/connect_camera", json=camera_config)
            assert response.status_code in [200, 503]

@pytest.mark.asyncio
async def test_lidar_robot_integration():
    """라이다-로봇 통합 테스트"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # 라이다 데이터 수집
        response = await client.get("/api/v1/lidar/data")
        assert response.status_code in [200, 503]
        
        if response.status_code == 200:
            lidar_data = response.json()
            # 로봇에 라이다 데이터 전송
            response = await client.post("/api/v1/robots/process_lidar", json=lidar_data)
            assert response.status_code in [200, 503]

@pytest.mark.asyncio
async def test_person_detection_flow():
    """사용자 감지 플로우 통합 테스트"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # 카메라에서 사용자 감지
        response = await client.post("/api/v1/cameras/detect_person")
        assert response.status_code in [200, 503]
        
        if response.status_code == 200:
            person_data = response.json()
            # 감지된 사용자 정보 저장
            response = await client.post("/api/v1/persons", json=person_data)
            assert response.status_code in [201, 409]

@pytest.mark.asyncio
async def test_error_handling():
    """에러 처리 통합 테스트"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # 잘못된 엔드포인트
        response = await client.get("/api/v1/not_exists")
        assert response.status_code == 404
        
        # 잘못된 인증
        response = await client.get("/api/v1/auth/profile")
        assert response.status_code == 401
        
        # 잘못된 요청 데이터
        response = await client.post("/api/v1/auth/register", json={})
        assert response.status_code == 400

@pytest.mark.asyncio
async def test_middleware_integration():
    """미들웨어 통합 테스트"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # CORS 테스트
        headers = {
            "Origin": "http://localhost:3000",
            "Access-Control-Request-Method": "POST"
        }
        response = await client.options("/api/v1/auth/login", headers=headers)
        assert response.status_code == 200
        assert "Access-Control-Allow-Origin" in response.headers
        
        # 로깅 미들웨어 테스트
        response = await client.get("/")
        assert response.status_code == 200
        # 로그 파일 확인 로직 추가 필요