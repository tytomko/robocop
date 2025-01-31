import asyncio  # 비동기 작업을 위한 라이브러리
import websockets  # 웹소켓 통신을 위한 라이브러리
import json  # JSON 데이터 처리
import logging  # 로깅 기능
import math  # 수학 연산
import base64  # base64 인코딩/디코딩
import struct  # 바이너리 데이터 구조 처리
import time  # 시간 관련 기능
from http import HTTPStatus
import pathlib
from aiohttp import web
import aiohttp
import roslibpy
import threading
import queue

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

HOST = "127.0.0.1"
PORT = 3000

class LidarSubscriber:
    def __init__(self, ros_host="localhost", ros_port=9090):
        self.ros_host = ros_host
        self.ros_port = ros_port
        self.clients = set()
        self.last_process_time = 0
        self.ros_client = None
        self.ros_thread = None
        self.point_queue = queue.Queue()
        
    def start_ros_thread(self):
        """ROS 클라이언트를 별도의 스레드에서 실행"""
        self.ros_thread = threading.Thread(target=self._run_ros_client)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
    def _run_ros_client(self):
        """ROS 클라이언트 실행 및 토픽 구독"""
        try:
            self.ros_client = roslibpy.Ros(host=self.ros_host, port=self.ros_port)
            self.ros_client.run()
            logger.info("ROS 클라이언트가 연결되었습니다.")
            
            # 라이다 토픽 구독
            listener = roslibpy.Topic(self.ros_client, 
                                    '/ssafy/velodyne_points', 
                                    'sensor_msgs/PointCloud2')
            
            def callback(message):
                current_time = time.time()
                if current_time - self.last_process_time >= 1.0:
                    points = self.convert_scan_to_points(message)
                    self.point_queue.put(points)
                    self.last_process_time = current_time
            
            listener.subscribe(callback)
            logger.info("velodyne_points 토픽 구독을 시작합니다.")
            
        except Exception as e:
            logger.error(f"ROS 연결 중 오류 발생: {str(e)}")
    
    async def connect(self):
        """ROS 연결 시작"""
        self.start_ros_thread()
        
        while True:
            try:
                # 큐에서 포인트 데이터를 확인
                if not self.point_queue.empty():
                    points = self.point_queue.get_nowait()
                    await self.broadcast(points)
            except queue.Empty:
                pass
            except Exception as e:
                logger.error(f"데이터 처리 중 오류 발생: {str(e)}")
            
            await asyncio.sleep(0.1)  # 짧은 대기 시간으로 CPU 사용량 조절
    
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
            max_points = 10000  # 포인트 수 제한 (최대 10000개)

            # 데이터 처리 시 샘플링 간격 지정 (적어도 1의 간격으로 샘플링)
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
    
    async def register_client(self, websocket):
        """새로운 웹 클라이언트 연결을 등록"""
        self.clients.add(websocket)  # 웹소켓 연결 수락 제거
        logger.info(f"새로운 클라이언트가 연결되었습니다. 현재 클라이언트 수: {len(self.clients)}")
    
    async def unregister_client(self, websocket):
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

async def handle_index(request):
    """HTML 파일 제공"""
    try:
        with open('public/html/lidar_page.html', 'r', encoding='utf-8') as f:
            return web.Response(text=f.read(), content_type='text/html')
    except Exception as e:
        logger.error(f"HTML 파일 제공 중 오류: {str(e)}")
        return web.Response(status=404)

async def websocket_handler(request):
    """웹소켓 연결 처리"""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    try:
        await lidar_subscriber.register_client(ws)
        async for msg in ws:
            if msg.type == web.WSMsgType.ERROR:
                logger.error(f'웹소켓 에러: {ws.exception()}')
    finally:
        await lidar_subscriber.unregister_client(ws)
    
    return ws

async def main():
    # 라이다 구독자 시작
    lidar_task = asyncio.create_task(lidar_subscriber.connect())
    
    # aiohttp 앱 설정
    app = web.Application()
    app.router.add_get('/', handle_index)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_static('/public', 'public')
    
    # 서버 실행
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, HOST, PORT)
    await site.start()
    
    logger.info(f"서버가 시작되었습니다. http://{HOST}:{PORT}")
    
    # 서버 실행 유지
    await asyncio.gather(
        lidar_task,
        asyncio.Event().wait()  # 서버를 계속 실행 상태로 유지
    )

# 전역 LidarSubscriber 인스턴스 생성
lidar_subscriber = LidarSubscriber()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("프로그램을 종료합니다.")
    except Exception as e:
        logger.error(f"예기치 않은 오류 발생: {str(e)}")
        logger.exception(e)
