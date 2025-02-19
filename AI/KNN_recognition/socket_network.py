import socket
import time
import threading
import json

# 콜백 함수 저장 (메시지를 받으면 실행할 함수)
callback_function = None

def set_callback(callback):
    """
    메시지를 수신하면 실행할 콜백 함수를 설정하는 함수
    """
    global callback_function
    callback_function = callback
    print(f"✅ 콜백 함수가 등록되었습니다: {callback}")  # 디버깅 출력



# 메시지 송신
# 소켓 연결
def persistent_connect_request(IP, PORT):
    """
    대상 서버에 연결을 시도합니다.
    
    - 기존 연결이 유지되고 있으면 재연결을 하지 않습니다.
    - 연결이 끊어지면 새로 연결합니다.
    
    Returns:
        socket 객체 또는 None (연결 실패 시)
    """
    # if client_socket is not None:
    #     return None

    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((IP, PORT))
        print(f"Connected to {IP}:{PORT}")
        #client_socket.sendall("연결 성공".encode('utf-8'))
        return client_socket  # 연결 성공 시 소켓 반환

    except Exception as e:
        print(f"Error connecting to {IP}:{PORT}: {e}")
        return None  # 연결 실패 시 None 반환


def send_command(client_socket, IP, PORT, command):
    """
    기존에 유지된 소켓을 통해 메시지를 전송합니다.
    
    - 소켓이 유지되고 있다면 메시지만 전송
    - 연결이 끊어져 있으면 자동으로 재연결 후 전송
    
    Parameters:
        client_socket (socket): 소켓 객체
        IP (str): 대상 IP
        PORT (int): 대상 포트
        command (str): 전송할 명령어
    
    Returns:
        업데이트된 client_socket (연결이 끊어지면 재연결 후 반환)
    """
    
    # 소켓이 None이면 재연결 시도
    if client_socket is None:
        print(f"No active connection to {IP}. Reconnecting...")
        client_socket = persistent_connect_request(IP, PORT)

    if client_socket:
        try:
            # 소켓이 유효한지 확인
            client_socket.sendall(command.encode('utf-8'))
            print(f"Sent to {IP}: {command}")
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f"Error sending to {IP}: {e}")
            client_socket.close()
            print(f"Reconnecting to {IP}...")
            client_socket = persistent_connect_request(IP, PORT)
            if client_socket:
                try:
                    client_socket.sendall(command.encode('utf-8'))
                    print(f"Sent to {IP} after reconnecting: {command}")
                except Exception as e:
                    print(f"Failed to send to {IP} after reconnecting: {e}")
    return client_socket  # 최신 client_socket 반환


def persistent_connect_receive(client_socket, IP, PORT):
    while client_socket is None:  # 연결될 때까지 반복
        try:
            client_socket.connect((IP, PORT))
            print(f"Connected to {IP}:{PORT}")
            return client_socket  # 연결 성공 시 소켓 반환
        except Exception as e:
            print(f"Error connecting to {IP}:{PORT}: {e}")
            client_socket = None  # 연결 실패 시 다시 None으로 설정
            time.sleep(2)  # 2초 후 재시도

def receive_messages(client_socket, IP, PORT, buffer_size=1024):
    """서버에서 JSON 데이터를 계속 수신하며 끊어진 데이터도 합쳐서 처리"""
    global callback_function  
    buffer = b""  

    while True:
        if client_socket is None:
            print("🔴 No active connection. Reconnecting...")
            client_socket = persistent_connect_request(IP, PORT)

        try:
            #print("📡 [수신 대기 중] 데이터를 기다리고 있습니다...")
            chunk = client_socket.recv(buffer_size)
            
            if not chunk:
                print("🔴 서버에서 빈 패킷을 보냈거나 연결이 닫힘. 다시 연결 시도...")
                client_socket.close()
                client_socket = None
                continue

            print(f"📩 [데이터 수신] 크기: {len(chunk)} bytes")
            buffer += chunk  

            while b"}" in buffer:
                index = buffer.index(b"}") + 1
                json_data = buffer[:index]
                buffer = buffer[index:]

                # 🛠 **빈 데이터 예외 처리**
                if not json_data.strip():
                    print("⚠️ [경고] 빈 JSON 데이터가 수신됨, 무시")
                    continue

                try:
                    parsed_data = json.loads(json_data.decode("utf-8"))
                    print(f"✅ [JSON 수신 완료] {parsed_data}")

                    if callback_function:
                        callback_function(json.dumps(parsed_data), IP, PORT)
                    else:
                        print("⚠️ [경고] 콜백 함수가 설정되지 않았습니다.")

                except json.JSONDecodeError:
                    print(f"❌ [JSON 파싱 오류] 데이터가 손상되었거나 불완전합니다: {json_data}")
                    continue  # 다음 데이터 패킷을 기다림

        except (socket.timeout, ConnectionResetError, OSError) as e:
            print(f"🔴 수신 오류 발생: {e}, 다시 연결 중...")
            client_socket.close()
            client_socket = None
