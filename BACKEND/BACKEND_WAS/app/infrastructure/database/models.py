from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field

class BaseDBModel(BaseModel):
    """기본 데이터베이스 모델"""
    createdAt: datetime = Field(default_factory=datetime.now)
    updatedAt: Optional[datetime] = None

    def to_db_dict(self) -> dict:
        """모델을 데이터베이스 문서로 변환"""
        data = self.dict(by_alias=True)
        if not data.get("updatedAt"):
            data.pop("updatedAt", None)
        return data

    @classmethod
    def from_db_dict(cls, data: Dict[str, Any]):
        """데이터베이스 문서를 모델로 변환"""
        return cls(**data)

class Counter(BaseModel):
    """시퀀스 카운터 모델"""
    id: str
    seq: int = 0

class DatabaseInfo(BaseModel):
    """데이터베이스 정보 모델"""
    version: str
    lastMigration: Optional[datetime] = None
    status: str = "active"