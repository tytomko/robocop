import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
import matplotlib.pyplot as plt
import json
import networkx as nx
import os

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # 현재 위치 토픽 구독 (/robot_1/utm_pose)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_1/utm_pose',
            self.pose_callback,
            10
        )
        
        # global_path 토픽 구독 (nav_msgs/Path) → 초록색 선 표시
        self.global_path_sub = self.create_subscription(
            NavPath,
            '/robot_1/global_path',
            self.global_path_callback,
            10
        )
        
        # approach_path 토픽 구독 (nav_msgs/Path) → 빨간색 선 표시
        self.approach_path_sub = self.create_subscription(
            NavPath,
            '/robot_1/approach_path',
            self.approach_path_callback,
            10
        )
        
        # global_map.json 파일을 읽어 그래프 데이터 로드 (배경 지도)
        self.load_graph_data("global_map.json")

        # 초기 좌표 설정
        self.current_x = None
        self.current_y = None

        # 인터랙티브 모드 활성화 후 Figure, Axes 생성
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        
        # global_map.json에서 로드한 그래프가 있을 경우 배경에 표시
        if self.G is not None:
            self.pos = {node: node for node in self.G.nodes()}
            nx.draw(self.G, pos=self.pos, ax=self.ax, node_size=50, node_color='blue',
                    edge_color='gray', with_labels=False)
        else:
            self.get_logger().error("Graph 데이터가 로드되지 않았습니다.")
        
        # 로봇의 현재 위치를 표시할 빨간 점 생성
        self.current_pos_plot, = self.ax.plot([], [], 'ro', markersize=10, label='Current Position')
        
        # global_path 선 (초록색)
        self.global_path_plot, = self.ax.plot([], [], 'g-', linewidth=4, label='Global Path')
        
        # approach_path 선 (빨간색)
        self.approach_path_plot, = self.ax.plot([], [], 'r-', linewidth=4, label='Approach Path')
        
        self.ax.legend()

    def load_graph_data(self, file_name):
        """
        global_map.json 파일을 읽어 "nodes"와 "links"를 이용해 networkx Graph(또는 DiGraph)를 생성합니다.
        파일 경로는 현재 스크립트(__file__) 기준입니다.
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(script_dir, file_name)
        try:
            with open(full_path, 'r') as f:
                data = json.load(f)
            directed = data.get("directed", False)
            self.G = nx.DiGraph() if directed else nx.Graph()
            nodes = data.get("nodes", [])
            for node in nodes:
                if "id" in node and isinstance(node["id"], list) and len(node["id"]) >= 2:
                    node_id = tuple(node["id"][:2])  # X, Y만 사용
                    self.G.add_node(node_id)
            links = data.get("links", [])
            for link in links:
                if "source" in link and "target" in link:
                    source = tuple(link["source"][:2])
                    target = tuple(link["target"][:2])
                    self.G.add_edge(source, target)
            self.get_logger().info("global_map.json 그래프 데이터가 성공적으로 로드되었습니다.")
        except Exception as e:
            self.get_logger().error(f"global_map.json 그래프 데이터 로드 실패: {e}")
            self.G = None

    def pose_callback(self, msg):
        # 현재 위치 업데이트
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.update_current_position_plot()

    def global_path_callback(self, msg):
        # global_path 토픽의 경로를 초록색 선으로 업데이트
        x_vals = [pose.pose.position.x for pose in msg.poses]
        y_vals = [pose.pose.position.y for pose in msg.poses]
        self.global_path_plot.set_xdata(x_vals)
        self.global_path_plot.set_ydata(y_vals)
        plt.draw()
        plt.pause(0.1)

    def approach_path_callback(self, msg):
        # approach_path 토픽의 경로를 빨간색 선으로 업데이트
        x_vals = [pose.pose.position.x for pose in msg.poses]
        y_vals = [pose.pose.position.y for pose in msg.poses]
        self.approach_path_plot.set_xdata(x_vals)
        self.approach_path_plot.set_ydata(y_vals)
        plt.draw()
        plt.pause(0.1)

    def update_current_position_plot(self):
        if self.current_x is not None and self.current_y is not None:
            self.current_pos_plot.set_xdata([self.current_x])
            self.current_pos_plot.set_ydata([self.current_y])
            # Axes 범위 업데이트 (옵션)
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
