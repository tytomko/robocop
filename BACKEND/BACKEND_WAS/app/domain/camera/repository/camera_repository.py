from datetime import datetime
from pathlib import Path
from typing import Optional
from ....infrastructure.database.connection import DatabaseConnection
from ..models.camera_models import VideoSession
from ....common.config.manager import get_settings

settings = get_settings()

class CameraRepository:
    """카메라 저장소"""
    def __init__(self):
        """초기화"""
        self.db = None
        self.video_storage_path = Path(settings.storage.VIDEO_STORAGE_PATH)
        self.frame_storage_path = Path(settings.storage.FRAME_STORAGE_PATH)
        self.video_storage_path.mkdir(parents=True, exist_ok=True)
        self.frame_storage_path.mkdir(parents=True, exist_ok=True)

    async def create_video_session(self, camera_name: str) -> VideoSession:
        """새로운 비디오 세션을 생성합니다."""
        session_id = datetime.now().strftime(f"{camera_name}_%Y%m%d_%H%M%S")
        file_path = str(self.video_storage_path / f"{session_id}.mp4")
        
        video_session = VideoSession(
            session_id=session_id,
            file_path=file_path,
            camera_type=camera_name
        )

        with DatabaseConnection() as db:
            db_video = {
                "session_id": video_session.session_id,
                "file_path": video_session.file_path,
                "camera_type": video_session.camera_type,
                "created_at": video_session.created_at,
                "status": video_session.status
            }
            await db.videos.insert_one(db_video)

        return video_session

    async def update_video_session_status(self, session_id: str, status: str, metadata: Optional[dict] = None):
        """비디오 세션의 상태를 업데이트합니다."""
        update_data = {
            "status": status,
            "updated_at": datetime.now()
        }
        if metadata:
            update_data["metadata"] = metadata

        with DatabaseConnection() as db:
            await db.videos.update_one(
                {"session_id": session_id},
                {"$set": update_data}
            )

    def create_frame_directory(self, session_id: str) -> Path:
        """프레임을 저장할 디렉토리를 생성합니다."""
        frame_dir = self.frame_storage_path / session_id
        frame_dir.mkdir(parents=True, exist_ok=True)
        return frame_dir

    def get_video_path(self, session_id: str) -> Path:
        """비디오 파일 경로를 반환합니다."""
        return self.video_storage_path / f"{session_id}.mp4"

    def get_frame_path(self, session_id: str) -> Path:
        """프레임 디렉토리 경로를 반환합니다."""
        return self.frame_storage_path / session_id