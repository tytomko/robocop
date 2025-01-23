import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pandas as pd
import matplotlib.pyplot as plt
import utm
import numpy as np

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot_1/utm_pose',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # 경로 데이터 로드
        self.load_path_data("patrol_path1.csv")

        # 초기 좌표 설정
        self.current_x = None
        self.current_y = None

        # 초기 시각화 설정
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        
        # NumPy 배열로 변환 후 시각화
        self.path_plot, = self.ax.plot(
            self.df['x'].to_numpy(), 
            self.df['y'].to_numpy(), 
            'r-', linewidth=3, label='Path'
        )
        
        self.current_pos_plot, = self.ax.plot([], [], 'bo', markersize=8, label='Current Position')
        self.ax.legend()

    def load_path_data(self, file_path):
        try:
            df = pd.read_csv(file_path)
            df.dropna(inplace=True)
            df['x'] = pd.to_numeric(df['x'], errors='coerce')
            df['y'] = pd.to_numeric(df['y'], errors='coerce')
            if df.isnull().values.any():
                self.get_logger().error("CSV 파일에 유효하지 않은 좌표 값이 포함되어 있습니다.")
                return
            self.df = df
            self.get_logger().info("경로 데이터가 성공적으로 로드되었습니다.")
        except Exception as e:
            self.get_logger().error(f"경로 데이터 로드 실패: {e}")

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.update_plot()

    def update_plot(self):
        if self.current_x is not None and self.current_y is not None:
            self.current_pos_plot.set_xdata([self.current_x])
            self.current_pos_plot.set_ydata([self.current_y])
            self.ax.set_xlim(self.current_x - 10, self.current_x + 10)
            self.ax.set_ylim(self.current_y - 10, self.current_y + 10)
            plt.draw()
            plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()
    rclpy.spin(path_visualizer)
    path_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
