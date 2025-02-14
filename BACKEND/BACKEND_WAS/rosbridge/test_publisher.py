#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, 'test_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Test message: {time.time()}'
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

