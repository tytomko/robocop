import roslibpy
import asyncio
import logging
import time
from typing import Optional


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from redis import Redis
from ....common.config.manager import get_settings

settings = get_settings()

class RosBridgeConnection:
    
    # _instance = None
    # client = None
    
    # HOST = "127.0.0.1"
    # PORT = 10000
    # MAX_RETRIES = 3
    # RETRY_DELAY = 2

    # def __new__(cls):
    #     if cls._instance is None:
    #         cls._instance = super(RosBridgeConnection, cls).__new__(cls)
    #         cls._instance.connect()  # 동기적으로 연결
    #     return cls._instance
    
    _instances = {}
    
    def __new__(cls, port=10000):
        if port not in cls._instances:
            instance = super(RosBridgeConnection, cls).__new__(cls)
            instance.HOST = "127.0.0.1"
            instance.PORT = port
            instance.MAX_RETRIES = 3
            instance.RETRY_DELAY = 2
            instance.client = None
            instance.connected_clients = set()
            instance.redis_client = Redis(
                host=settings.REDIS_HOST,
                port=settings.REDIS_PORT,
                decode_responses=True
            )
            cls._instances[port] = instance
            instance.connect()
        return cls._instances[port]
    
    def register_client(self, client_id: str):
        """새로운 클라이언트를 등록합니다."""
        try:
            self.connected_clients.add(client_id)
            self.redis_client.sadd(f'rosbridge_clients:{self.PORT}', client_id)
            logger.info(f"ROS Bridge 클라이언트 등록 (Port {self.PORT}): {client_id}")
        except Exception as e:
            logger.error(f"클라이언트 등록 실패: {str(e)}")
            raise
        
    def unregister_client(self, client_id: str):
        """클라이언트를 제거합니다."""
        try:
            # discard()는 요소가 없어도 에러를 발생시키지 않음
            self.connected_clients.discard(client_id)
            self.redis_client.srem(f'rosbridge_clients:{self.PORT}', client_id)
            
            # 연결된 클라이언트가 없을 때 연결 종료
            if not self.connected_clients:
                if self.client and self.client.is_connected:
                    try:
                        self.client.close()  # terminate() 대신 close() 사용
                    except:
                        pass
                    finally:
                        self.client = None
                
                if self.PORT in self._instances:
                    del self._instances[self.PORT]
                
            logger.info(f"ROS Bridge 클라이언트 제거 (Port {self.PORT}): {client_id}")
        except Exception as e:
            logger.error(f"클라이언트 제거 실패: {str(e)}")

    def ensure_connected(self):
        """연결이 되어 있는지 확인하고, 필요 시 재연결합니다."""
        if not self.client or not self.client.is_connected:
            logger.warning("ROS Bridge 연결이 끊어졌습니다. 재연결을 시도합니다.")
            self.connect()

    def publish(self, topic_name: str, msg_type: str, message: dict):
        """토픽에 메시지를 발행합니다."""
        try:
            if not self.client or not self.client.is_connected:
                self.ensure_connected()
            
            topic = roslibpy.Topic(self.client, topic_name, msg_type)
            topic.publish(message)
            logger.info(f"Published to {topic_name}: {message}")
            return True
        except Exception as e:
            logger.error(f"Failed to publish to {topic_name}: {str(e)}")
            return False

    def connect(self):
        try:
            if self.client and self.client.is_connected:
                return
                
            logger.info(f"ROS Bridge 연결 시도 중... (Port: {self.PORT})")
            
            # reactor 상태 확인 및 초기화
            try:
                from twisted.internet import reactor
                if reactor.running:
                    reactor.stop()
            except:
                pass
                
            self.client = roslibpy.Ros(host=self.HOST, port=self.PORT)
            connection_timeout = 15  # 타임아웃 증가
            
            def on_connection():
                logger.info("ROS Bridge 서버에 연결되었습니다.")
                
            self.client.on_ready(on_connection)
            self.client.run()
            
            start_time = time.time()
            while not self.client.is_connected and (time.time() - start_time) < connection_timeout:
                time.sleep(0.1)
                
            if not self.client.is_connected:
                raise Exception("연결 시간 초과")
                
        except Exception as e:
            logger.error(f"ROS Bridge 연결 실패: {str(e)}")
            self.client = None


