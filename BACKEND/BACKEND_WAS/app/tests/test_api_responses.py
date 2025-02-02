import pytest
from .conftest import save_example_response
from ..domain.auth.security.service import SecurityService
from ..domain.camera.models.camera_models import CameraConfig
from datetime import datetime

security_service = SecurityService()

def test_root_endpoint(test_client, example_responses_dir):
    """루트 엔드포인트 테스트"""
    response = test_client.get("/")
    assert response.status_code == 200
    save_example_response(response.json(), "root", example_responses_dir)

def test_auth_login(test_client, example_responses_dir):
    """로그인 엔드포인트 테스트"""
    login_data = {
        "username": "admin",
        "password": "admin1234"
    }
    response = test_client.post("/api/v1/auth/login", json=login_data)
    assert response.status_code in [200, 401]  # 401은 초기 설정이 안된 경우
    save_example_response(response.json(), "auth_login", example_responses_dir)

def test_robot_status(test_client, example_responses_dir):
    """로봇 상태 조회 테스트"""
    response = test_client.get("/api/v1/robots/status")
    assert response.status_code in [200, 404]
    save_example_response(response.json(), "robot_status", example_responses_dir)

def test_camera_status(test_client, example_responses_dir):
    """카메라 상태 조회 테스트"""
    config = CameraConfig(
        camera_topic="/test/camera",
        camera_name="test_camera"
    )
    response = test_client.post("/api/v1/cameras/initialize", json=config.dict())
    assert response.status_code in [200, 503]
    save_example_response(response.json(), "camera_status", example_responses_dir)

def test_lidar_data(test_client, example_responses_dir):
    """라이다 데이터 조회 테스트"""
    response = test_client.get("/api/v1/lidar/data")
    assert response.status_code in [200, 503]
    save_example_response(response.json(), "lidar_data", example_responses_dir)

def test_person_list(test_client, example_responses_dir):
    """사용자 목록 조회 테스트"""
    response = test_client.get("/api/v1/persons")
    assert response.status_code == 200
    save_example_response(response.json(), "person_list", example_responses_dir)

def test_error_responses(test_client, example_responses_dir):
    """에러 응답 테스트"""
    # 404 에러
    response = test_client.get("/api/v1/not_found")
    assert response.status_code == 404
    save_example_response(response.json(), "error_404", example_responses_dir)
    
    # 401 에러
    response = test_client.get("/api/v1/auth/profile")
    assert response.status_code == 401
    save_example_response(response.json(), "error_401", example_responses_dir)
    
    # 400 에러
    response = test_client.post("/api/v1/auth/login", json={})
    assert response.status_code == 400
    save_example_response(response.json(), "error_400", example_responses_dir)