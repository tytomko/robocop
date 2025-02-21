import os
import uuid
from datetime import datetime
from typing import Optional, Dict, Any
from .constants import ALLOWED_IMAGE_EXTENSIONS, MAX_IMAGE_SIZE

def generate_unique_id(prefix: str = "") -> str:
    """고유 ID 생성"""
    unique_id = str(uuid.uuid4())
    return f"{prefix}_{unique_id}" if prefix else unique_id

def validate_image_file(file_name: str, file_size: int) -> bool:
    """이미지 파일 유효성 검사"""
    ext = os.path.splitext(file_name)[1].lower()
    return ext in ALLOWED_IMAGE_EXTENSIONS and file_size <= MAX_IMAGE_SIZE

def format_datetime(dt: datetime) -> str:
    """날짜/시간 포맷팅"""
    return dt.strftime("%Y%m%d_%H%M%S")

def parse_datetime(dt_str: str) -> Optional[datetime]:
    """날짜/시간 문자열 파싱"""
    try:
        return datetime.strptime(dt_str, "%Y%m%d_%H%M%S")
    except ValueError:
        return None

def create_error_response(message: str, code: int = 400) -> Dict[str, Any]:
    """에러 응답 생성"""
    return {
        "success": False,
        "status": code,
        "message": message,
        "timestamp": datetime.now().isoformat()
    }

def create_success_response(data: Any = None, message: str = "성공") -> Dict[str, Any]:
    """성공 응답 생성"""
    response = {
        "success": True,
        "status": 200,
        "message": message,
        "timestamp": datetime.now().isoformat()
    }
    if data is not None:
        response["data"] = data
    return response

def sanitize_filename(filename: str) -> str:
    """파일명 정리"""
    # 허용되지 않는 문자 제거
    invalid_chars = '<>:"/\\|?*'
    for char in invalid_chars:
        filename = filename.replace(char, '')
    return filename.strip()

def get_file_extension(filename: str) -> str:
    """파일 확장자 추출"""
    return os.path.splitext(filename)[1].lower()

def format_file_size(size_in_bytes: int) -> str:
    """파일 크기 포맷팅"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size_in_bytes < 1024:
            return f"{size_in_bytes:.1f}{unit}"
        size_in_bytes /= 1024
    return f"{size_in_bytes:.1f}TB"

def ensure_directory(directory: str):
    """디렉토리 존재 확인 및 생성"""
    if not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)

def cleanup_old_files(directory: str, max_age_days: int = 7):
    """오래된 파일 정리"""
    now = datetime.now()
    for filename in os.listdir(directory):
        filepath = os.path.join(directory, filename)
        if os.path.isfile(filepath):
            file_modified = datetime.fromtimestamp(os.path.getmtime(filepath))
            if (now - file_modified).days > max_age_days:
                try:
                    os.remove(filepath)
                except OSError:
                    pass