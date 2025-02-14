import roslibpy
import logging
import time
from typing import Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RosBridgeConnection:
    _instance = None
    _client = None
    
    HOST = "127.0.0.1"
    # HOST = "localhost"
    PORT = 9090
    # PORT = 10000
    MAX_RETRIES = 3
    RETRY_DELAY = 2  # 초
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RosBridgeConnection, cls).__new__(cls)
            cls._instance._connect()
        return cls._instance
    
    def _connect(self) -> Optional[roslibpy.Ros]:
        """ROS Bridge 서버에 연결 시도"""
        retries = 0
        while retries < self.MAX_RETRIES:
            if not self._client or not self._client.is_connected:
                try:
                    logger.info(f"ROS Bridge 연결 시도 중... (시도 {retries + 1}/{self.MAX_RETRIES})")
                    self._client = roslibpy.Ros(host=self.HOST, port=self.PORT)
                    self._client.run()
                    logger.info("ROS Bridge 서버에 연결되었습니다.")
                    return self._client
                except Exception as e:
                    logger.error(f"ROS Bridge 연결 실패: {e}")
                    self._client = None
                    retries += 1
                    if retries < self.MAX_RETRIES:
                        logger.info(f"{self.RETRY_DELAY}초 후 재시도합니다...")
                        time.sleep(self.RETRY_DELAY)
        
        raise Exception("최대 재시도 횟수를 초과했습니다.")
    
    @property
    def client(self):
        """ROS Bridge 클라이언트 반환"""
        if not self._client or not self._client.is_connected:
            self._connect()
        return self._client
    
    def disconnect(self):
        """ROS Bridge 연결 종료"""
        if self._client and self._client.is_connected:
            self._client.terminate()
            logger.info("ROS Bridge 연결이 종료되었습니다.") 