import asyncio
import json
import base64
import os
from aiohttp import web
import roslibpy
import numpy as np
import cv2
import av
import glob
import websockets
import fractions
import socket
from datetime import datetime

# ROS 클라이언트 설정
# HOST = '192.168.100.104'
LOCAL_HOST = '127.0.0.1'
ros_client = roslibpy.Ros(host=LOCAL_HOST, port=9090)
connected_clients = set()

# 전역 변수로 이벤트 루프 저장
main_loop = None

# static 폴더가 없다면 생성
if not os.path.exists('static'):
    os.makedirs('static')

async def handle_index(request):
    """html 파일 루트 경로 처리"""
    print("Lidar 페이지 요청")
    return web.FileResponse('public/html/lidar_page.html')

async def broadcast_to_clients(data):
    if connected_clients:
        message = json.dumps(data)
        await asyncio.gather(
            *[client.send_str(message) for client in connected_clients]
        )

async def handle_websocket(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    print('WebSocket 클라이언트 연결됨')
    connected_clients.add(ws)

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.ERROR:
                print(f'WebSocket 에러: {ws.exception()}')
    finally:
        connected_clients.remove(ws)
        print('WebSocket 클라이언트 연결 해제')
    
    return ws

async def init_ros():
    global ros_client
    try:
        print("\nROS Bridge 연결 시도 중...")
        
        # ROS Bridge 서버 정보
        ROS_BRIDGE_HOST = LOCAL_HOST  # 실제 ROS Bridge가 실행 중인 IP
        ROS_BRIDGE_PORT = 9090
        
        print(f"ROS Bridge 서버: {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
        
        # 호스트 연결 가능 여부 확인
        try:
            socket.gethostbyname(ROS_BRIDGE_HOST)
            print("호스트 확인 성공")
        except socket.gaierror as e:
            print(f"호스트 확인 실패: {e}")
            raise
        
        # ROS Bridge 연결
        ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
        
        # 연결 시도
        try:
            ros_client.run()
            print("ROS Bridge 연결 시도 중...")
            
            # 연결 상태 확인
            retry_count = 0
            max_retries = 5
            while not ros_client.is_connected and retry_count < max_retries:
                print(f'연결 대기 중... ({retry_count + 1}/{max_retries})')
                await asyncio.sleep(1)
                retry_count += 1
                
            if not ros_client.is_connected:
                raise Exception("ROS Bridge 연결 시간 초과")
                
            print('ROS Bridge 연결 성공')
            
        except Exception as e:
            print(f"ROS Bridge 연결 실패: {e}")
            if ros_client:
                ros_client.terminate()
            raise
        
        # 토픽 구독 설정
        lidar_topic = roslibpy.Topic(
            ros_client,
            '/ssafy/velodyne_points',
            'sensor_msgs/PointCloud2'
        )
        
        def topic_callback(msg):
            try:
                print("\n=== 라이다 메시지 수신 ===")
                print(f"토픽: {lidar_topic.name}")
                handle_lidar_message(msg)
            except Exception as e:
                print(f"콜백 에러: {e}")
                import traceback
                traceback.print_exc()
        
        # 구독 시작
        lidar_topic.subscribe(topic_callback)
        print(f"\n토픽 구독 시작: {lidar_topic.name}")
        
    except Exception as e:
        print(f'\n=== ROS 연결 에러 ===')
        print(f'에러 타입: {type(e).__name__}')
        print(f'에러 메시지: {str(e)}')
        if ros_client and ros_client.is_connected:
            print("연결 종료...")
            ros_client.terminate()
        raise

def handle_lidar_message(message):
    try:
        print("\n=== 라이다 메시지 처리 ===")
        
        # 데이터 구조 확인을 위한 로깅
        print("Fields:", message.get('fields', []))
        print("Point Step:", message.get('point_step', 0))
        print("Data type:", type(message.get('data', [])))
        print("Data length:", len(message.get('data', [])))
        
        # 문자열을 바이트로 변환
        if isinstance(message['data'], str):
            data_bytes = message['data'].encode('latin1')
        else:
            data_bytes = message['data']
            
        # 바이트 데이터를 numpy array로 변환
        data_buffer = np.frombuffer(data_bytes, dtype=np.uint8)
        
        # point_step에 따라 데이터 재구성
        point_step = message.get('point_step', 0)
        if point_step > 0:
            # 전체 데이터 크기가 point_step으로 나누어 떨어지는지 확인
            num_points = len(data_buffer) // point_step
            total_size = num_points * point_step
            
            # 실제 사용할 데이터만 선택
            data_buffer = data_buffer[:total_size]
            points_data = data_buffer.reshape(num_points, point_step)
            
            # fields 정보를 이용하여 x, y, z 오프셋 찾기
            fields = message.get('fields', [])
            offsets = {field['name']: field['offset'] for field in fields}
            
            # float32 타입으로 변환 (x, y, z 좌표)
            x_data = points_data[:, offsets['x']:offsets['x']+4].view(np.float32).reshape(-1)
            y_data = points_data[:, offsets['y']:offsets['y']+4].view(np.float32).reshape(-1)
            z_data = points_data[:, offsets['z']:offsets['z']+4].view(np.float32).reshape(-1)
            
            # 디버깅을 위한 출력
            print(f"Number of points: {num_points}")
            print(f"Point data shape: {points_data.shape}")
            print(f"X data shape: {x_data.shape}")
            print(f"Y data shape: {y_data.shape}")
            print(f"Z data shape: {z_data.shape}")
            
            # x, y, z 좌표를 하나의 배열로 결합
            points_xyz = np.column_stack((x_data, y_data, z_data))
            
            # 웹소켓으로 전송할 데이터 준비
            lidar_data = {
                'header': {
                    'seq': message.get('header', {}).get('seq', 0),
                    'stamp': message.get('header', {}).get('stamp', 0),
                    'frame_id': message.get('header', {}).get('frame_id', '')
                },
                'height': message.get('height', 0),
                'width': message.get('width', 0),
                'fields': message.get('fields', []),
                'point_step': point_step,
                'row_step': message.get('row_step', 0),
                'is_dense': message.get('is_dense', False),
                'data_size': len(points_xyz),
                'points': points_xyz.tolist(),  # numpy array를 list로 변환
                'timestamp': datetime.now().isoformat()
            }
            
            # 메인 이벤트 루프를 통해 비동기 작업 실행
            if main_loop is not None:
                main_loop.call_soon_threadsafe(
                    lambda: asyncio.create_task(broadcast_to_clients(lidar_data))
                )
            
    except Exception as e:
        print(f"라이다 처리 에러: {e}")
        import traceback
        traceback.print_exc()


async def main():
    try:
        print("\n=== 서버 시작 ===")
        global main_loop
        main_loop = asyncio.get_running_loop()
        
        # ROS 초기화 (더 긴 타임아웃 설정)
        try:
            await asyncio.wait_for(init_ros(), timeout=10.0)
        except asyncio.TimeoutError:
            print("ROS 초기화 시간 초과")
            raise
        
        # 웹 앱 설정
        app = web.Application()
        
        # 라우트 설정
        app.router.add_get('/', handle_index)
        app.router.add_get('/ws', handle_websocket)
        
        # 정적 파일 설정
        app.router.add_static('/static', 'static')
        app.router.add_static('/public', 'public')
        
        # 서버 디렉토리 구조 확인
        print("\n=== 서버 디렉토리 구조 ===")
        print("현재 작업 디렉토리:", os.getcwd())
        print("public/html/lidar_page.html 존재 여부:", os.path.exists('public/html/index.html'))
        
        # 서버 실행
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, 'localhost', 3000)
        print("\n서버 시작...")
        await site.start()
        print("서버가 http://localhost:3000 에서 실행 중")
        
        await asyncio.Event().wait()
        
    except Exception as e:
        print(f"\n=== 서버 시작 중 에러 발생 ===")
        print(f"에러 타입: {type(e).__name__}")
        print(f"에러 메시지: {str(e)}")
        if ros_client and ros_client.is_connected:
            ros_client.terminate()
        raise

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n서버 종료")
        if ros_client and ros_client.is_connected:
            ros_client.terminate() 