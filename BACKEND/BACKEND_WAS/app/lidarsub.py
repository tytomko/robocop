import asyncio  # 비동기 작업을 위한 라이브러리
import websockets  # 웹소켓 통신을 위한 라이브러리
import json  # JSON 데이터 처리
import logging  # 로깅 기능
import math  # 수학 연산
from fastapi import WebSocket  # FastAPI 웹소켓
import base64  # base64 인코딩/디코딩
import struct  # 바이너리 데이터 구조 처리
import time  # 시간 관련 기능

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LidarSubscriber:
    def __init__(self, websocket_url="ws://192.168.100.104:9090"):
        self.websocket_url = websocket_url  # ROS2 웹소켓 브릿지 URL
        self.websocket = None  # ROS2 웹소켓 연결 객체
        self.clients = set()  # 연결된 클라이언트들을 저장하는 집합
        self.last_process_time = 0  # 마지막 데이터 처리 시간
        
    async def connect(self):
        """ROS2 웹소켓 브릿지에 연결하고 데이터를 구독하는 메인 함수"""
        try:
            # ROS2 웹소켓 브릿지에 연결
            self.websocket = await websockets.connect(self.websocket_url)
            logger.info("ROS 웹소켓 서버에 연결되었습니다.")
            
            # 벨로다인 라이다 토픽 구독 메시지 생성
            subscribe_message = {
                "op": "subscribe",
                "topic": "/ssafy/velodyne_points",  # 구독할 토픽 이름
                "type": "sensor_msgs/PointCloud2"   # 메시지 타입
            }
            
            # 구독 요청 전송
            await self.websocket.send(json.dumps(subscribe_message))
            logger.info("LaserScan 토픽 구독을 시작합니다.")
            
            # 메시지 수신 루프
            while True:
                try:
                    message = await self.websocket.recv()  # 메시지 수신 대기
                    current_time = time.time()
                    
                    # 1초에 한 번만 데이터 처리
                    if current_time - self.last_process_time >= 1.0:
                        data = json.loads(message)
                        
                        if data["op"] == "publish":
                            scan_data = data["msg"]  # 포인트 클라우드 데이터 추출
                            points = self.convert_scan_to_points(scan_data)  # 데이터 변환
                            await self.broadcast(points)  # 클라이언트들에게 전송
                            self.last_process_time = current_time  # 처리 시간 갱신
                    else:
                        # 1초가 지나지 않았으면 잠시 대기
                        await asyncio.sleep(0.1)
                        
                except websockets.exceptions.ConnectionClosed:
                    logger.error("ROS 웹소켓 연결이 끊어졌습니다.")
                    break
                    
        except Exception as e:
            logger.error(f"연결 중 오류 발생: {str(e)}")
    
    def convert_scan_to_points(self, point_cloud_data):
        """PointCloud2 데이터를 Three.js에서 사용할 수 있는 포인트 배열로 변환"""
        try:
            # base64로 인코딩된 포인트 클라우드 데이터를 디코딩
            binary_data = base64.b64decode(point_cloud_data.get('data', ''))
            point_step = point_cloud_data.get('point_step', 22)  # 각 포인트의 바이트 크기
            
            # 디버깅을 위한 데이터 크기 로깅
            logger.info(f"Binary data length: {len(binary_data)}")
            logger.info(f"Point step: {point_step}")
            
            points = []
            # 포인트 수 제한 (최대 10000개)
            max_points = 10000
            step = max(1, len(binary_data) // point_step // max_points)
            
            # 바이너리 데이터를 포인트로 변환
            for i in range(0, len(binary_data), point_step * step):
                if i + 22 <= len(binary_data):
                    try:
                        # 4개의 float32 값(x, y, z, intensity)을 읽음
                        x, y, z, intensity = struct.unpack_from('ffff', binary_data, i)
                        
                        # 유효한 포인트만 추가 (0에 가까운 값 제외)
                        if not (abs(x) < 0.001 and abs(y) < 0.001 and abs(z) < 0.001):
                            # ROS2 좌표계를 Three.js 좌표계로 변환
                            three_x = -y  # ROS의 y축을 Three.js의 -x축으로
                            three_y = z   # ROS의 z축을 Three.js의 y축으로
                            three_z = x   # ROS의 x축을 Three.js의 z축으로
                            
                            points.append({
                                "x": three_x,
                                "y": three_y,
                                "z": three_z,
                                "intensity": intensity
                            })
                    except struct.error as e:
                        logger.error(f"Error unpacking point at index {i}: {str(e)}")
                        continue
                    
                    # 처리 진행 상황 로깅 (1000개마다)
                    if len(points) % 1000 == 0:
                        logger.info(f"Processed {len(points)} points")
            
            # 최종 처리된 포인트 수와 샘플 데이터 로깅
            logger.info(f"Final points count: {len(points)}")
            if points:
                logger.info(f"Sample point: {points[0]}")
            
            return points
            
        except Exception as e:
            logger.error(f"Error parsing point cloud data: {str(e)}")
            logger.exception(e)
            return []
    
    async def register_client(self, websocket: WebSocket):
        """새로운 웹 클라이언트 연결을 등록"""
        await websocket.accept()  # 웹소켓 연결 수락
        self.clients.add(websocket)  # 클라이언트 목록에 추가
        logger.info(f"새로운 클라이언트가 연결되었습니다. 현재 클라이언트 수: {len(self.clients)}")
    
    async def unregister_client(self, websocket: WebSocket):
        """웹 클라이언트 연결 해제"""
        self.clients.remove(websocket)  # 클라이언트 목록에서 제거
        logger.info(f"클라이언트가 연결 해제되었습니다. 현재 클라이언트 수: {len(self.clients)}")
    
    async def broadcast(self, points):
        """모든 연결된 클라이언트에게 포인트 클라우드 데이터 전송"""
        if not self.clients:
            return
            
        # 전송할 메시지 생성
        message = {
            "type": "lidar_points",
            "points": points,
            "timestamp": time.time()
        }
        
        logger.info(f"Broadcasting {len(points)} points")
        
        # 모든 클라이언트에게 데이터 전송
        for client in self.clients.copy():
            try:
                await client.send_json(message)
            except Exception as e:
                logger.error(f"클라이언트 전송 실패: {str(e)}")
                await self.unregister_client(client)

# 전역 LidarSubscriber 인스턴스 생성
lidar_subscriber = LidarSubscriber()

async def start_lidar_subscriber():
    """LidarSubscriber 시작 함수"""
    await lidar_subscriber.connect()

async def websocket_endpoint(websocket: WebSocket):
    """FastAPI 웹소켓 엔드포인트 핸들러"""
    await lidar_subscriber.register_client(websocket)
    try:
        while True:
            # 클라이언트 연결 유지 확인
            await websocket.receive_text()
    except Exception as e:
        await lidar_subscriber.unregister_client(websocket)
