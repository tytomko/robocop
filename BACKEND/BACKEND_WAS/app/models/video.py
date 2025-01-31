from sqlalchemy import Column, Integer, String, DateTime
from datetime import datetime
from ..database import Base

class Video(Base):
    __tablename__ = "videos"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, unique=True, index=True)
    start_time = Column(DateTime, default=datetime.utcnow)
    end_time = Column(DateTime, nullable=True)
    file_path = Column(String)
    frame_count = Column(Integer, default=0)
    status = Column(String, default="recording")  # recording, completed, error
    camera_type = Column(String)  # front, rear 