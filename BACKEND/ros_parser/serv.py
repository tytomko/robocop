import asyncio
import json
import base64
import os
from aiohttp import web
import roslibpy
# from PIL import Image
import io
import numpy as np

# ROS 클라이언트 설정
ros_client = roslibpy.Ros(host='192.168.100.104', port=9090)
connected_clients = set()

# 전역 변수로 이벤트 루프 저장
main_loop = None

async def handle_index(request):
    return web.FileResponse(os.path.join('public', 'html', 'index.html'))

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
            if msg.type == web.WSMsgType.TEXT:
                print(f'클라이언트로부터 메시지 수신: {msg.data}')
            elif msg.type == web.WSMsgType.ERROR:
                print(f'WebSocket 에러: {ws.exception()}')
    finally:
        connected_clients.remove(ws)
        print('WebSocket 클라이언트 연결 해제')
    
    return ws

def handle_image_message(message):
    print(type(message))
    print("데이터 타입:", type(message['data']))
    
    raw_data = message['data']
    
    if isinstance(raw_data, str):
        raw_data = raw_data.encode('latin1')
    
    base64_image = base64.b64encode(raw_data).decode('utf-8')
    
    json_data = {
        'width': message.get('width', 0),
        'height': message.get('height', 0),
        'encoding': 'rgb8',
        'image': base64_image
    }
    
    print('이미지 데이터 크기:', len(raw_data))
    
    # 메인 이벤트 루프에서 비동기 작업 실행
    asyncio.run_coroutine_threadsafe(broadcast_to_clients(json_data), main_loop)

async def init_ros():
    ros_client.run()
    print('ROS 브리지에 연결됨')

    # 이미지 토픽 구독
    image_topic = roslibpy.Topic(
        ros_client,
        '/ssafy/tb3_camera/image_raw',
        'sensor_msgs/Image'
    )
    image_topic.subscribe(handle_image_message)

async def main():
    global main_loop
    main_loop = asyncio.get_running_loop()
    
    # 웹 앱 설정
    app = web.Application()
    app.router.add_get('/', handle_index)
    app.router.add_static('/static', path='public')
    app.router.add_get('/ws', handle_websocket)

    # ROS 초기화
    await init_ros()

    # 서버 시작
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, 'localhost', 3000)
    await site.start()
    print(f'서버가 http://localhost:3000 에서 실행 중입니다')

    # 서버 실행 유지
    try:
        await asyncio.Future()
    finally:
        ros_client.terminate()

if __name__ == '__main__':
    asyncio.run(main()) 