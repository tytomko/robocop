from pydantic import BaseModel

class PublishRequest(BaseModel):
    message: str
