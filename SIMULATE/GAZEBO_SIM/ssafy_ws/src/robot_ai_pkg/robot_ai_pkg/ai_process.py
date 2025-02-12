import socket
import threading
import time
import rclpy
from rclpy.node import Node
from robot_custom_interfaces.srv import Estop
from std_msgs.msg import String

# 명령어 전역 상수
CMD_STOP = '거수자인식-정지'
CMD_RESUME = '거수자 처리완료-재개'
CMD_DISAPPEAR = '거수자 사라짐'
IP_ADDRESS = '0.0.0.0'
PORT = 5000

class RobotAI(Node):
    def __init__(self):
        super().__init__('robot_ai')
        # 파라미터 선언 및 가져오기
        self.declare_parameter('robot_num', 1)
        self.declare_parameter('robot_name', 'robot1')
        self.robot_num = self.get_parameter('robot_num').value
        self.robot_name = self.get_parameter('robot_name').value

        # 서비스 클라이언트 생성 (로봇 번호에 따라 토픽 생성)
        self.temp_stop_client = self.create_client(Estop, f'/robot_{self.robot_num}/temp_stop')
        self.resume_client = self.create_client(Estop, f'/robot_{self.robot_num}/resume')

        # /ai_info 토픽 퍼블리셔 생성 (로봇 번호 포함)
        self.ai_info_pub = self.create_publisher(String, f'/robot_{self.robot_num}/ai_info', 10)

        # 명령 관리용 변수와 Lock
        self.current_command = None
        self.command_lock = threading.Lock()

        # 소켓 서버 초기화 및 연결 대기
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((IP_ADDRESS, PORT))
        self.socket.listen(1)
        self.get_logger().info(f'Socket server started on {IP_ADDRESS}:{PORT}, waiting for connections...')

        # 소켓 수신 루프를 별도 스레드에서 실행
        self.socket_thread = threading.Thread(target=self.accept_connections)
        self.socket_thread.daemon = True
        self.socket_thread.start()

    def accept_connections(self):
        """
        소켓으로 들어오는 데이터는 이제 "명령" 문자열만 포함합니다.
        미리 정의된 명령어(CMD_STOP, CMD_RESUME, CMD_DISAPPEAR)는 해당 처리 후,
        그 외의 명령은 수신한 내용 그대로 ai_info 토픽에 발행합니다.
        """
        while rclpy.ok():
            conn, addr = self.socket.accept()
            self.get_logger().info(f'Connection from {addr}')
            try:
                data = conn.recv(1024).decode('utf-8').strip()
            except Exception as e:
                self.get_logger().error(f'Error receiving data: {e}')
                conn.close()
                continue

            if not data:
                conn.close()
                continue

            # 수신된 데이터는 단순한 명령 문자열
            command = data

            # 최신 명령 업데이트 (스레드 안전하게)
            with self.command_lock:
                self.current_command = command

            # ai_info 토픽에 수신 메시지 발행
            info_msg = String()
            info_msg.data = f"Received command '{command}' for robot '{self.robot_name}'"
            self.ai_info_pub.publish(info_msg)

            # 미리 정의된 명령어에 따른 처리
            if command == CMD_STOP:
                self.get_logger().info('Starting service call for temporary stop.')
                threading.Thread(
                    target=self.call_service_with_cancellation,
                    args=(self.temp_stop_client, command),
                    daemon=True
                ).start()
            elif command == CMD_RESUME:
                self.get_logger().info('Starting service call for resume.')
                threading.Thread(
                    target=self.call_service_with_cancellation,
                    args=(self.resume_client, command),
                    daemon=True
                ).start()
            elif command == CMD_DISAPPEAR:
                self.get_logger().info('Received disappear command. No service call will be executed.')
            else:
                # 미리 정의되지 않은 명령은 그대로 ai_info 토픽에 발행
                unknown_msg = String()
                unknown_msg.data = command
                self.ai_info_pub.publish(unknown_msg)
                self.get_logger().info(f"Published unknown command to ai_info: {command}")

            conn.close()

    def call_service_with_cancellation(self, client, command):
        """
        서비스가 준비될 때까지 최대 5번 대기하고 호출하며, 응답의 success 필드가 True여야 성공으로 간주합니다.
        응답이 실패하거나 타임아웃이 발생하면 최대 5번까지 재시도하며, 각 재시도 사이에 1초씩 대기합니다.
        대기 중 또는 호출 후 최신 명령이 변경되면 진행 중인 호출은 취소합니다.
        rclpy.spin_until_future_complete() 대신 루프와 time.sleep()으로 future 완료 여부를 확인합니다.
        """
        max_call_attempts = 5
        for call_attempt in range(1, max_call_attempts + 1):
            # 서비스 준비 대기 (최대 5회, 각 시도 1초씩)
            max_wait_attempts = 5
            wait_attempt = 0
            service_ready = False
            while wait_attempt < max_wait_attempts:
                if client.wait_for_service(timeout_sec=1.0):
                    service_ready = True
                    break
                else:
                    wait_attempt += 1
                    self.get_logger().info(f"Attempt {wait_attempt} waiting for service {client.srv_name}...")
                    with self.command_lock:
                        if self.current_command != command:
                            self.get_logger().info("Command updated. Cancelling this service call attempt.")
                            return
            if not service_ready:
                warning_msg = f"Service {client.srv_name} not available after waiting attempts."
                self.get_logger().error(warning_msg)
                warn = String()
                warn.data = warning_msg
                self.ai_info_pub.publish(warn)
                return

            # 서비스 준비 완료 → 서비스 호출
            request = Estop.Request()
            future = client.call_async(request)

            # future가 완료될 때까지 주기적으로 확인 (명령 취소 여부 체크)
            start_time = time.time()
            timeout_duration = 10.0  # 최대 대기 시간 (초)
            while rclpy.ok() and not future.done() and (time.time() - start_time < timeout_duration):
                time.sleep(0.1)
                with self.command_lock:
                    if self.current_command != command:
                        self.get_logger().info("Command updated during service call. Cancelling waiting for result.")
                        return

            if not future.done():
                self.get_logger().error("Service call did not complete within timeout.")
                warn = String()
                warn.data = "Service call did not complete within timeout."
                self.ai_info_pub.publish(warn)
            else:
                with self.command_lock:
                    if self.current_command != command:
                        self.get_logger().info("Command updated after service call. Ignoring the result.")
                        return

                response = future.result()
                if response is not None and hasattr(response, 'success') and response.success:
                    self.get_logger().info(f"Service call to {client.srv_name} succeeded on attempt {call_attempt}.")
                    return  # 성공하면 종료
                else:
                    error_msg = f"Service call to {client.srv_name} failed on attempt {call_attempt}."
                    self.get_logger().error(error_msg)
                    warn = String()
                    warn.data = error_msg
                    self.ai_info_pub.publish(warn)
            # 1초 대기 후 재시도 (마지막 시도가 아니면)
            if call_attempt < max_call_attempts:
                time.sleep(1)

        # 모든 시도 실패 시
        final_msg = f"Service call to {client.srv_name} failed after {max_call_attempts} attempts."
        self.get_logger().error(final_msg)
        final_warn = String()
        final_warn.data = final_msg
        self.ai_info_pub.publish(final_warn)

def main(args=None):
    rclpy.init(args=args)
    robot_ai = RobotAI()
    try:
        rclpy.spin(robot_ai)
    except KeyboardInterrupt:
        pass
    robot_ai.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
