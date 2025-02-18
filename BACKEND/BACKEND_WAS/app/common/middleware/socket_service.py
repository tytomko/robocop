import socket
import threading
import logging
import json
from app.domain.person.repository.person_repository import PersonRepository
from app.domain.person.models.person_models import ImageInfo
from typing import List

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ALLOWED_IP = "121.178.98.113"  # 허용된 클라이언트 IP 주소

person_repository = PersonRepository()

clients = []  # 연결된 클라이언트 소켓 목록

def handle_client(client_socket, address):
    """클라이언트 연결을 처리합니다."""
    client_ip, _ = address
    if client_ip != ALLOWED_IP:
        logger.info(f"허용되지 않은 클라이언트 IP: {client_ip}. 연결을 종료합니다.")
        client_socket.close()
        return

    logger.info(f"클라이언트 연결: {address}")
    clients.append(client_socket)  # 클라이언트 소켓을 목록에 추가
    try:
        while True:
            message = client_socket.recv(1024)
            if not message:
                break
            logger.info(f"메시지 수신: {message.decode('utf-8')}")
            client_socket.sendall(message)  # 에코 서버처럼 받은 메시지를 그대로 보냅니다.
    except Exception as e:
        logger.error(f"클라이언트 처리 중 오류 발생: {e}")
    finally:
        logger.info(f"클라이언트 연결 종료: {address}")
        clients.remove(client_socket)  # 연결 종료 시 목록에서 제거
        client_socket.close()

def start_socket_server():
    """TCP 소켓 서버를 시작합니다."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", 5000))  # 모든 인터페이스에서 수신
    server.listen(5)
    logger.info("TCP 소켓 서버가 시작되었습니다.")

    try:
        while True:
            client_socket, addr = server.accept()
            client_handler = threading.Thread(
                target=handle_client, args=(client_socket, addr)
            )
            client_handler.start()
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
                "images": [{
                    'imageId': image.imageId,
                    'url': image.url,
                    'uploadedAt': image.uploadedAt.isoformat() if image.uploadedAt else None,
                    'imageNumber': image.imageNumber
                } for image in person.images]
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
