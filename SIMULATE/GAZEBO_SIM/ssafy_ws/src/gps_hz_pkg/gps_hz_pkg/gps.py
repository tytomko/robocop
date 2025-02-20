import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DownsampleNode(Node):
    def __init__(self):
        super().__init__('downsample_node')
        # 파라미터 선언 및 값 가져오기
        self.declare_parameter('robot_number', 1)
        self.robot_num = self.get_parameter('robot_number').value


        # 토픽 설정: 파라미터 robot_number를 이용해 토픽 이름 생성
        self.in_topic = f"/robot_{self.robot_num}/utm_pose"
        self.out_topic = f"/robot_{self.robot_num}/down_utm"
        
        # 구독자 생성: 입력 토픽으로부터 PoseStamped 메시지를 받음
        self.subscription = self.create_subscription(
            PoseStamped,
            self.in_topic,
            self.pose_callback,
            10
        )
        
        # 발행자 생성: 재발행 토픽에 PoseStamped 메시지를 발행
        self.publisher = self.create_publisher(PoseStamped, self.out_topic, 10)
        
        # 최신 메시지를 저장할 변수
        self.latest_msg = None
        
        # 1Hz 타이머 생성: 1초마다 timer_callback 실행
        self.timer = self.create_timer(1.0, self.timer_callback)

    def pose_callback(self, msg):
        """
        입력 토픽 구독 콜백: 받은 메시지를 저장
        """
        self.latest_msg = msg

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
