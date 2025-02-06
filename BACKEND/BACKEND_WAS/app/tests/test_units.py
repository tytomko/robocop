import pytest
from ..domain.auth.security.service import SecurityService
from ..domain.camera.service import CameraService
from ..domain.robot.service import RobotService
from ..domain.lidar.service import LidarService
from ..domain.person.service import PersonService
from ..common.exceptions.base import (
    ValidationError,
    AuthenticationError,
    NotFoundError,
    DatabaseError
)

def test_security_service():
    """보안 서비스 단위 테스트"""
    security_service = SecurityService()
    
    # 비밀번호 해시 테스트
    password = "test1234"
    hashed = security_service.get_password_hash(password)
    assert security_service.verify_password(password, hashed)
    
    # 토큰 생성 및 검증 테스트
    token_data = {"sub": "test@example.com"}
    token = security_service.create_access_token(token_data)
    decoded = security_service.verify_token(token)
    assert decoded["sub"] == token_data["sub"]

def test_camera_service():
    """카메라 서비스 단위 테스트"""
    camera_service = CameraService()
    
    # 카메라 초기화 테스트
    with pytest.raises(ValidationError):
        camera_service.initialize(None)
    
    # 카메라 상태 확인 테스트
    status = camera_service.get_status()
    assert isinstance(status, dict)

def test_robot_service():
    """로봇 서비스 단위 테스트"""
    robot_service = RobotService()
    
    # 로봇 상태 확인 테스트
    status = robot_service.get_status()
    assert isinstance(status, dict)
    
    # 로봇 명령 테스트
    with pytest.raises(ValidationError):
        robot_service.send_command(None)

def test_lidar_service():
    """라이다 서비스 단위 테스트"""
    lidar_service = LidarService()
    
    # 라이다 데이터 조회 테스트
    data = lidar_service.get_data()
    assert isinstance(data, dict)
    
    # 라이다 설정 테스트
    with pytest.raises(ValidationError):
        lidar_service.configure(None)

def test_person_service():
    """사용자 서비스 단위 테스트"""
    person_service = PersonService()
    
    # 사용자 생성 테스트
    with pytest.raises(ValidationError):
        person_service.create_person(None)
    
    # 사용자 조회 테스트
    with pytest.raises(NotFoundError):
        person_service.get_person_by_id("non_existent_id")

def test_database_operations():
    """데이터베이스 작업 단위 테스트"""
    person_service = PersonService()
    
    # 데이터베이스 연결 테스트
    try:
        person_service.get_all_persons()
    except DatabaseError as e:
        assert str(e)

def test_authentication():
    """인증 단위 테스트"""
    security_service = SecurityService()
    
    # 잘못된 인증 정보 테스트
    with pytest.raises(AuthenticationError):
        security_service.authenticate_user("wrong@example.com", "wrongpass")