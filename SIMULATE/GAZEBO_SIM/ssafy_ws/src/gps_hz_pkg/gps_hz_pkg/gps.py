import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_custom_interfaces.msg import Status

class DownsampleNode(Node):
    def __init__(self):
        super().__init__('downsample_node')
        # 파라미터 선언 및 값 가져오기
        self.declare_parameter('robot_number', 1)
        self.robot_num = self.get_parameter('robot_number').value

        # 토픽 설정: 파라미터 robot_number를 이용해 토픽 이름 생성
        self.in_topic = f"/robot_{self.robot_num}/utm_pose"
        self.out_topic = f"/robot_{self.robot_num}/down_utm"
        
        self.in_status_topic = f"/robot_{self.robot_num}/status"
        self.out_status_topic = f"/robot_{self.robot_num}/down_status"

        # 구독자 생성: 입력 토픽으로부터 PoseStamped 메시지를 받음
        self.subscription = self.create_subscription(
            PoseStamped,
            self.in_topic,
            self.pose_callback,
            10
        )
        
        # 구독자 생성: 입력 토픽으로부터 Status 메시지를 받음
        self.status_subscription = self.create_subscription(
            Status,
            self.in_status_topic,
            self.status_callback,
            10
        )
    
        # 발행자 생성: 재발행 토픽에 메시지를 발행
        self.publisher = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_publisher = self.create_publisher(Status, self.out_status_topic, 10)    
        
        # 최신 메시지를 저장할 변수
        self.latest_msg = None
        self.latest_status = None  # Status 메시지 저장 변수 추가
        
        # 1Hz 타이머 생성: 1초마다 timer_callback 실행
        self.timer = self.create_timer(1.0, self.timer_callback)

    def pose_callback(self, msg):
        """
        입력 토픽 구독 콜백: 받은 PoseStamped 메시지를 저장
        """
        self.latest_msg = msg

    def status_callback(self, msg):
        """
        입력 토픽 구독 콜백: 받은 Status 메시지를 저장
        """
        self.latest_status = msg

    def timer_callback(self):
        """
        타이머 콜백: 저장된 최신 메시지를 1Hz 주기로 재발행.
        헤더 timestamp를 현재 시간으로 업데이트함.
        """
        if self.latest_msg is not None:
            new_msg = PoseStamped()
            new_msg.header = self.latest_msg.header
            new_msg.header.stamp = self.get_clock().now().to_msg()
            new_msg.pose = self.latest_msg.pose
            self.publisher.publish(new_msg)
            #self.get_logger().info(f"1Hz 주기로 {self.out_topic} 메시지를 발행합니다.")
            
        if self.latest_status is not None:
            new_status = Status()
            # Status 메시지의 모든 필드 복사
            new_status.id = self.latest_status.id
            new_status.name = self.latest_status.name
            new_status.mode = self.latest_status.mode
            new_status.battery = self.latest_status.battery
            new_status.temperatures = self.latest_status.temperatures
            new_status.network = self.latest_status.network
            new_status.starttime = self.latest_status.starttime
            new_status.is_active = self.latest_status.is_active

            self.status_publisher.publish(new_status)
            #self.get_logger().info(f"1Hz 주기로 {self.out_status_topic} 메시지를 발행합니다.")

def main(args=None):
    rclpy.init(args=args)
    node = DownsampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
