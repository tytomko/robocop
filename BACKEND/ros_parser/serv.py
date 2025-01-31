import asyncio
import json
import base64
import os
from aiohttp import web
import roslibpy
import numpy as np
import cv2
import av
from aiortc import VideoStreamTrack, RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
from datetime import datetime
import glob
import websockets
import fractions
import socket

# ROS 클라이언트 설정
# HOST = '192.168.100.104'
LOCAL_HOST = '127.0.0.1'
ros_client = roslibpy.Ros(host=LOCAL_HOST, port=9090)
connected_clients = set()

# 전역 변수로 이벤트 루프 저장
main_loop = None

# 전역 변수 선언
video_tracks = set()  # 활성화된 모든 video track을 저장

# static 폴더가 없다면 생성
if not os.path.exists('static'):
    os.makedirs('static')

async def handle_index(request):
    """루트 경로 처리"""
    print("Index 페이지 요청")
    return web.FileResponse('public/html/index.html')

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
        image_topic = roslibpy.Topic(
            ros_client,
            '/ssafy/tb3_front_camera/image_raw/compressed',
            'sensor_msgs/CompressedImage'
        )
        
        def topic_callback(msg):
            try:
                print("\n=== 이미지 메시지 수신 ===")
                print(f"토픽: {image_topic.name}")
                handle_image_message(msg)
            except Exception as e:
                print(f"콜백 에러: {e}")
                import traceback
                traceback.print_exc()
        
        # 구독 시작
        image_topic.subscribe(topic_callback)
        print(f"\n토픽 구독 시작: {image_topic.name}")
        
    except Exception as e:
        print(f'\n=== ROS 연결 에러 ===')
        print(f'에러 타입: {type(e).__name__}')
        print(f'에러 메시지: {str(e)}')
        if ros_client and ros_client.is_connected:
            print("연결 종료...")
            ros_client.terminate()
        raise

def handle_image_message(message):
    try:
        print("\n=== 이미지 메시지 처리 ===")
        print("메시지 키:", list(message.keys()))
        
        if 'format' in message:
            print(f"이미지 포맷: {message['format']}")
        
        if 'data' not in message:
            print("에러: 메시지에 'data' 필드가 없음")
            return
            
        raw_data = message['data']
        print(f"데이터 타입: {type(raw_data)}")
        print(f"데이터 크기: {len(raw_data) if isinstance(raw_data, (str, bytes)) else 'unknown'}")
        
        # 활성 트랙 확인 및 업데이트
        active_tracks = [track for track in video_tracks if not track.stopped]
        print(f"\n=== 트랙 상태 ===")
        print(f"전체 트랙 수: {len(video_tracks)}")
        print(f"활성 트랙 수: {len(active_tracks)}")
        
        # 각 트랙의 상태 자세히 출력
        for i, track in enumerate(video_tracks):
            print(f"\n트랙 {i}:")
            print(f"  - 중지 상태: {track.stopped}")
            print(f"  - 프레임 수: {track.frame_count}")
            print(f"  - 최근 프레임 존재: {'예' if track.latest_frame is not None else '아니오'}")
        
        if not active_tracks:
            print("경고: 활성화된 비디오 트랙 없음")
            return
            
        # 모든 활성 트랙에 프레임 업데이트 시도
        for track in active_tracks:
            print(f"프레임 업데이트 시도")
            track.update_frame(raw_data)  # 무조건 업데이트 시도
            
    except Exception as e:
        print(f"이미지 처리 에러: {e}")
        import traceback
        traceback.print_exc()

# WebRTC 비디오 스트림을 위한 클래스
class ROSVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.latest_frame = None  # 최신 프레임 저장
        self.frame_count = 0      # 처리된 총 프레임 수
        self.stopped = False      # 트랙 중지 상태
        self.pts_time = 0         # 프레임 타임스탬프
        print("새로운 VideoTrack 생성됨")
    
    def stop(self):
        self.stopped = True
        super().stop()
    
    def update_frame(self, image_data):
        if self.stopped:
            print("트랙이 중지됨")
            return
            
        try:
            print("\n=== 프레임 업데이트 시작 ===")
            
            # Base64로 인코딩된 이미지 데이터를 디코딩
            if isinstance(image_data, str):
                try:
                    image_data = base64.b64decode(image_data)
                    print("Base64 디코딩 완료")
                except Exception as e:
                    print(f"Base64 디코딩 실패, latin1 인코딩 시도: {e}")
                    image_data = image_data.encode('latin1')
                    print("Latin1 인코딩 완료")
            
            # JPEG 압축 해제
            np_arr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                print("이미지 디코딩 실패")
                return
                
            print(f"이미지 디코딩 완료: shape={frame.shape}")
            
            # VideoFrame 생성
            video_frame = av.VideoFrame.from_ndarray(frame, format='bgr24')
            video_frame.time_base = fractions.Fraction(1, 30)
            self.pts_time += 1
            video_frame.pts = self.pts_time
            
            self.latest_frame = video_frame
            self.frame_count += 1
            print(f"프레임 처리 완료 #{self.frame_count}")
            
        except Exception as e:
            print(f"프레임 처리 에러: {e}")
            import traceback
            traceback.print_exc()
    
    async def recv(self):
        print("recv 호출됨")
        if self.stopped:
            print("트랙 중지됨 - recv 종료")
            return None
            
        if self.latest_frame is None:
            print("프레임 대기 중...")
            await asyncio.sleep(0.1)
            return await self.recv()
            
        frame = self.latest_frame
        self.latest_frame = None  # 프레임 사용 후 초기화
        print(f"프레임 전송: pts={frame.pts if frame else 'None'}")
        return frame

async def handle_offer(request):
    try:
        print("\n=== WebRTC 연결 시작 ===")
        
        # 1. 요청 데이터 확인
        params = await request.json()
        
        # 2. PeerConnection 설정
        config = RTCConfiguration(
            iceServers=[
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun1.l.google.com:19302"])
            ]
        )
        
        pc = RTCPeerConnection(configuration=config)
        print("PeerConnection 생성 성공")
        
        # 3. 비디오 트랙 설정
        print(f"현재 활성 트랙 수: {len(video_tracks)}")
        video_track = ROSVideoStreamTrack()
        video_tracks.add(video_track)
        print(f"트랙 추가 후 활성 트랙 수: {len(video_tracks)}")
        pc.addTrack(video_track)
        print("비디오 트랙 추가 완료")
        
        # 4. Remote Description 설정
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        await pc.setRemoteDescription(offer)
        print("Remote Description 설정 완료")
        
        # 5. Answer 생성
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        print("Answer 생성 및 설정 완료")
        
        # 연결 상태 모니터링
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"\n=== Connection state changed: {pc.connectionState} ===")
            print(f"현재 활성 트랙 수: {len(video_tracks)}")
            if pc.connectionState == "failed":
                print("연결 실패 - 트랙 정리")
                video_tracks.discard(video_track)
                video_track.stop()
            elif pc.connectionState == "closed":
                print("연결 종료 - 트랙 정리")
                video_tracks.discard(video_track)
                video_track.stop()
            elif pc.connectionState == "connected":
                print("연결 성공 - 트랙 활성화 확인")
                if video_track not in video_tracks:
                    video_tracks.add(video_track)
            print(f"상태 변경 후 활성 트랙 수: {len(video_tracks)}")
        
        return web.Response(
            content_type="application/json",
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            })
        )
        
    except Exception as e:
        print(f"WebRTC 연결 에러: {e}")
        import traceback
        traceback.print_exc()
        return web.Response(
            status=500,
            content_type="application/json",
            text=json.dumps({"error": str(e)})
        )

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
        app.router.add_post('/offer', handle_offer)
        app.router.add_get('/ws', handle_websocket)
        
        # 정적 파일 설정
        app.router.add_static('/static', 'static')
        app.router.add_static('/public', 'public')
        
        # 서버 디렉토리 구조 확인
        print("\n=== 서버 디렉토리 구조 ===")
        print("현재 작업 디렉토리:", os.getcwd())
        print("public/html/index.html 존재 여부:", os.path.exists('public/html/index.html'))
        
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