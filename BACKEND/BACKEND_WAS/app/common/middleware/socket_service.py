import socket
import threading
import logging
import json
import asyncio
from datetime import datetime
from app.domain.person.repository.person_repository import PersonRepository
from app.domain.person.models.person_models import ImageInfo
from typing import List
from app.domain.robot.service.robot_service import RobotService
from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ALLOWED_IP = ["121.178.98.113", "192.168.100.248", "121.178.98.234"]  # 허용된 클라이언트 IP 주소

person_repository = PersonRepository()
robot_service = RobotService()

clients = []  # 연결된 클라이언트 소켓 목록
sse_clients = []  # SSE 클라이언트 목록

async def handle_client(client_socket, address):
    """클라이언트 연결을 처리합니다."""
    logger.info(f"클라이언트 연결: {address}")
    clients.append(client_socket)  # 클라이언트 소켓을 목록에 추가
    try:
        while True:
            message = client_socket.recv(1024)
            if not message:
                break
            logger.info(f"메시지 수신: {message.decode('utf-8')}")
            #await broadcast_to_clients(message.decode('utf-8'))
            await handle_message(message.decode('utf-8'))
    except Exception as e:
        logger.error(f"클라이언트 처리 중 오류 발생: {e}")
    finally:
        logger.info(f"클라이언트 연결 종료: {address}")
        clients.remove(client_socket)  # 연결 종료 시 목록에서 제거
        client_socket.close()

async def handle_message(message: str):
    """소켓으로부터 메시지를 처리합니다."""
    try:
        # 메시지가 JSON 형식인지 확인
        data = json.loads(message)
        
        # 메시지에 response_type이 있는지 확인
        response_type = data.get("response_type")
        if not response_type:
            logger.warning("메시지에 response_type이 없습니다. 메시지 브로드캐스트를 건너뜁니다.")
            return
        
        # 현재 시간을 detected_time으로 설정
        detected_time = datetime.now().strftime("%Y-%m-%d/%H:%M:%S")
        
        # SSE를 통해 메시지 전송
        logger.info(f"SSE 메시지 전송: {response_type}")
        await send_sse_message({
            "response_type": response_type,
            "detected_time": detected_time
        })
        
    except json.JSONDecodeError:
        logger.warning("수신한 메시지가 JSON 형식이 아닙니다.")
    except Exception as e:
        logger.error(f"메시지 처리 중 오류: {str(e)}")

async def broadcast_to_clients(message: str):
    """연결된 모든 클라이언트에게 메시지를 전송합니다."""
    for client in clients:
        try:
            logger.info(f"Sending message to client: {client.getpeername()}")
            client.sendall(message.encode('utf-8'))
            logger.info(f"Message sent to client: {client.getpeername()}")
        except Exception as e:
            logger.error(f"메시지 전송 중 오류 발생: {e}")
            clients.remove(client)

def start_socket_server():
    """TCP 소켓 서버를 시작합니다."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 포트 재사용 설정
    server.bind(("0.0.0.0", 6000))  # 모든 인터페이스에서 수신
    server.listen(5)
    logger.info("TCP 소켓 서버가 시작되었습니다.")

    try:
        while True:
            client_socket, addr = server.accept()
            # 비동기적으로 클라이언트 처리
            asyncio.run(handle_client(client_socket, addr))
    except KeyboardInterrupt:
        logger.info("서버가 종료됩니다.")
    finally:
        server.close()

async def send_person_images(person_id: int, response_type: str, images: List[ImageInfo]):
    """특정 Person의 이미지 갱신 시 클라이언트에게 JSON 데이터를 보냅니다."""
    try:
        # 이미지 정보를 올바른 형식으로 변환
        image_data = [{
            'imageId': image.imageId,
            'url': image.url,
            'uploadedAt': image.uploadedAt.isoformat() if image.uploadedAt else None,
            'imageNumber': image.imageNumber
        } for image in images]

        # JSON 데이터 생성
        data = {
            "response_type": response_type,
            "person_id": person_id,
            "images": image_data  # image_urls 대신 전체 이미지 정보를 전송
        }

        # JSON 데이터를 클라이언트에게 전송
        message = json.dumps(data)
        logger.info(f"전송할 메시지: {message}")

        # 연결된 모든 클라이언트에게 메시지 전송
        for client in clients:
            try:
                client.sendall(message.encode('utf-8'))
            except Exception as e:
                logger.error(f"메시지 전송 중 오류 발생: {e}")
                clients.remove(client)
    except Exception as e:
        logger.error(f"Person 이미지 전송 중 오류 발생: {e}")

async def send_people_images(response_type: str = "ALL_UPDATE_PERSON"):
    """모든 Person의 이미지를 클라이언트에게 전송합니다."""
    try:
        # 모든 Person 정보 가져오기
        persons = await person_repository.get_all_persons()
        
        # 각 Person의 이미지 정보 목록 생성
        people_data = [
            {
                "person_id": person.seq,
                "update_time": max(image.uploadedAt for image in person.images).strftime("%Y-%m-%d %H:%M:%S") if person.images else None,
                "image_urls": [image.url for image in person.images]
            }
            for person in persons
        ]

        # JSON 데이터 생성
        data = {
            "response_type": response_type,
            "people": people_data
        }

        # JSON 데이터를 클라이언트에게 전송
        message = json.dumps(data)
        logger.info(f"전송할 메시지: {message}")

        # 연결된 모든 클라이언트에게 메시지 전송
        for client in clients:
            try:
                client.sendall(message.encode('utf-8'))
            except Exception as e:
                logger.error(f"메시지 전송 중 오류 발생: {e}")
                clients.remove(client)
    except Exception as e:
        logger.error(f"모든 Person 이미지 전송 중 오류 발생: {e}")

async def send_sse_message(message: dict):
    """SSE 클라이언트에게 메시지를 전송합니다."""
    for client in sse_clients:
        try:
            await client.send(json.dumps(message))
        except Exception as e:
            logger.error(f"SSE 메시지 전송 중 오류 발생: {e}")
            sse_clients.remove(client)

app = FastAPI()

@app.get("/sse")
async def sse_endpoint(request: Request):
    """SSE 엔드포인트"""
    client = request.scope["app"].state.broadcast
    sse_clients.append(client)  # 클라이언트를 sse_clients 리스트에 추가
    
    try:
        async def event_generator():
            while True:
                if await request.is_disconnected():
                    break
                await asyncio.sleep(1)
                yield f"data: {json.dumps({'message': 'heartbeat'})}\n\n"
        
        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream"
        )
    finally:
        sse_clients.remove(client)  # 연결이 종료되면 클라이언트 제거
