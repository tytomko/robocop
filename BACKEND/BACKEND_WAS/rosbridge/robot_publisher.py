#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PoseStamped
from robot_custom_interfaces.msg import Status
from geometry_msgs.msg import Twist
import time
from datetime import datetime
import random
import math

class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_publisher')
        
        # 초기값 설정
        self.heading = 0.0
        self.utm_x = 304410.0
        self.utm_y = 3892841.0
        self.speed = 0.0
        self.battery = 100.0
        self.temperature = 45.0
        self.network = 80.0
        
        # Publishers
        self.heading_pub = self.create_publisher(Float32, '/robot_1/heading', 10)
        self.utm_pose_pub = self.create_publisher(PoseStamped, '/robot_1/utm_pose', 10)
        self.speed_kph_pub = self.create_publisher(Float64, '/robot_1/speed_kph', 10)
        self.speed_mps_pub = self.create_publisher(Float64, '/robot_1/speed_mps', 10)
        self.status_pub = self.create_publisher(Status, '/robot_1/status', 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Timer - 0.1초마다 업데이트
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def update_dummy_data(self):
        # Heading: -π에서 π 사이에서 천천히 변화
        self.heading += random.uniform(-0.1, 0.1)
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi
        
        # UTM 위치: 작은 원을 그리며 이동
        radius = 5.0  # 5미터 반경
        self.utm_x += random.uniform(-0.5, 0.5)
        self.utm_y += random.uniform(-0.5, 0.5)
        
        # Speed: 0~30km/h 사이에서 변화
        self.speed += random.uniform(-1.0, 1.0)
        self.speed = max(0.0, min(30.0, self.speed))
        
        # Battery: 천천히 감소
        self.battery -= random.uniform(0.01, 0.05)
        self.battery = max(0.0, self.battery)
        
        # Temperature: 45~60도 사이에서 변화
        self.temperature += random.uniform(-0.5, 0.5)
        self.temperature = max(45.0, min(60.0, self.temperature))
        
        # Network: 50~100% 사이에서 변화
        self.network += random.uniform(-2.0, 2.0)
        self.network = max(50.0, min(100.0, self.network))
        
    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')
        
    def timer_callback(self):
        # 더미데이터 업데이트
        self.update_dummy_data()
        
        # Heading
        heading_msg = Float32()
        heading_msg.data = self.heading
        self.heading_pub.publish(heading_msg)
        
        # UTM Pose
        utm_msg = PoseStamped()
        utm_msg.header.stamp = self.get_clock().now().to_msg()
        utm_msg.header.frame_id = "map"
        utm_msg.pose.position.x = self.utm_x
        utm_msg.pose.position.y = self.utm_y
        utm_msg.pose.position.z = 50.0 + random.uniform(-0.1, 0.1)
        utm_msg.pose.orientation.w = 1.0
        self.utm_pose_pub.publish(utm_msg)
        
        # Speed KPH
        speed_kph = Float64()
        speed_kph.data = self.speed
        self.speed_kph_pub.publish(speed_kph)
        
        # Speed MPS
        speed_mps = Float64()
        speed_mps.data = self.speed / 3.6  # km/h to m/s
        self.speed_mps_pub.publish(speed_mps)
        
        # Status
        status = Status()
        status.id = 1
        status.name = "ssafy"
        status.mode = random.choice(["waiting", "moving", "charging"])
        status.battery = self.battery
        status.temperatures = self.temperature
        status.network = self.network
        status.starttime = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status.is_active = True
        self.status_pub.publish(status)

def main():
    rclpy.init()
    node = RobotPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

