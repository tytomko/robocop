import roslibpy
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RosBridgeConnection:
    _instance = None
    _client = None
    
    HOST = "192.168.100.34"
    # HOST = "localhost"
    PORT = 9090
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RosBridgeConnection, cls).__new__(cls)
            cls._instance._connect()
        return cls._instance
    
    def _connect(self):
        """ROS Bridge 서버에 연결"""
        if not self._client or not self._client.is_connected:
            try:
                self._client = roslibpy.Ros(host=self.HOST, port=self.PORT)
                self._client.run()
                logger.info("ROS Bridge 서버에 연결되었습니다.")
            except Exception as e:
                logger.error(f"ROS Bridge 연결 실패: {e}")
                raise e
    
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