import os
import json
import pytest
from fastapi.testclient import TestClient
from ..main import app
from ..core.config import get_settings

@pytest.fixture
def test_client():
    """테스트 클라이언트 생성"""
    return TestClient(app)

@pytest.fixture
def example_responses_dir():
    """예제 응답을 저장할 디렉토리 생성"""
    dir_path = os.path.join(os.path.dirname(__file__), "example_responses")
    os.makedirs(dir_path, exist_ok=True)
    return dir_path

def save_example_response(response_data: dict, endpoint_name: str, dir_path: str):
    """API 응답 예제를 JSON 파일로 저장"""
    file_path = os.path.join(dir_path, f"{endpoint_name}.json")
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(response_data, f, ensure_ascii=False, indent=2)

@pytest.fixture
def settings():
    """테스트용 설정 가져오기"""
    return get_settings()

@pytest.fixture
def test_db():
    """테스트용 데이터베이스 설정"""
    settings = get_settings()
    # 여기에 테스트 DB 설정 로직 추가
    yield
    # 테스트 후 정리 작업

@pytest.fixture
def test_storage():
    """테스트용 스토리지 설정"""
    settings = get_settings()
    test_storage_path = os.path.join(os.path.dirname(__file__), "test_storage")
    os.makedirs(test_storage_path, exist_ok=True)
    
    # 테스트용 스토리지 경로 설정
    original_storage_path = settings.storage.BASE_STORAGE_PATH
    settings.storage.BASE_STORAGE_PATH = test_storage_path
    
    yield test_storage_path
    
    # 테스트 후 원래 설정으로 복구
    settings.storage.BASE_STORAGE_PATH = original_storage_path