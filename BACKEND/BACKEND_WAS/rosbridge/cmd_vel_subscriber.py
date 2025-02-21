#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f'Received CMD_VEL - linear: {msg.linear.x}, {msg.linear.y}, {msg.linear.z}, '
            f'angular: {msg.angular.x}, {msg.angular.y}, {msg.angular.z}'
        )

def main():
    rclpy.init()
    node = CmdVelSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

