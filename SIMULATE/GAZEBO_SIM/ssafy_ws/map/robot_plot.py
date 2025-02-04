import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import json
import networkx as nx
import os
import math

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # ─── 구독자 설정 ─────────────────────────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_1/utm_pose',
            self.pose_callback,
            10
        )
        
        self.global_path_sub = self.create_subscription(
            NavPath,
            '/robot_1/global_path',
            self.global_path_callback,
            10
        )
        
        self.approach_path_sub = self.create_subscription(
            NavPath,
            '/robot_1/approach_path',
            self.approach_path_callback,
            10
        )
        
        self.heading_sub = self.create_subscription(
            Float32,
            '/robot_1/heading',
            self.heading_callback,
            10
        )
        
        self.target_pose_sub = self.create_subscription(
            Point,
            '/robot_1/target_point',
            self.target_pose_callback,
            10
        )

        self.target_heading_sub = self.create_subscription(
            Float32,
            '/robot_1/target_heading',
            self.target_heading_callback,
            10
        )

        # ─── 그래프 로드 (배경 지도) ──────────────────────────────────────────────────────
        self.load_graph_data("global_map.json")

        # ─── 내부 상태 값 초기화 ─────────────────────────────────────────────────────────
        self.current_x = None
        self.current_y = None
        self.current_heading = None

        self.target_x = None
        self.target_y = None
        self.target_heading = None

        # ─── Matplotlib 설정 (인터랙티브 모드) ──────────────────────────────────────────
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        
        # 배경 그래프(지도) 표시
        if self.G is not None:
            self.pos = {node: node for node in self.G.nodes()}
            nx.draw(
                self.G, 
                pos=self.pos, 
                ax=self.ax, 
                node_size=50, 
                node_color='blue',
                edge_color='gray', 
                with_labels=False
            )
        else:
            self.get_logger().error("Graph 데이터가 로드되지 않았습니다.")
        
        # ─── 플롯 요소들 초기화 ─────────────────────────────────────────────────────────
        # 현재 위치(빨간 점)
        self.current_pos_plot, = self.ax.plot([], [], 'ro', markersize=10, label='Current Position')

        # 타겟 좌표(하늘색 X 마커)
        self.target_pos_plot, = self.ax.plot([], [], 'cx', markersize=10, label='Target Position')

        # global_path 선 (초록색)
        self.global_path_plot, = self.ax.plot([], [], 'g-', linewidth=4, label='Global Path')
        
        # approach_path 선 (빨간색)
        self.approach_path_plot, = self.ax.plot([], [], 'r-', linewidth=4, label='Approach Path')

        # 내 헤딩 표시(노란색 선)
        self.heading_line, = self.ax.plot([], [], 'y-', linewidth=2, label='Current Heading')

        # 타겟 헤딩 표시(분홍색 선)
        self.target_heading_line, = self.ax.plot([], [], 'm-', linewidth=2, label='Target Heading')

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

    # ──────────────────────────────────────────────────────────────────────────
    #                         콜백 함수들
    # ──────────────────────────────────────────────────────────────────────────

    def pose_callback(self, msg: PoseStamped):
        """현재 로봇 위치 콜백"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.update_current_position_plot()

    def heading_callback(self, msg: Float32):
        """현재 로봇 헤딩 콜백"""
        self.current_heading = msg.data
        self.update_current_heading_arrow()

    def target_pose_callback(self, msg: Point):
        """타겟 좌표 콜백"""
        self.target_x = msg.x
        self.target_y = msg.y
        self.update_target_position_plot()
        self.update_target_heading_arrow()  # 타겟좌표가 갱신되면 타겟헤딩 화살표도 다시 그리기

    def target_heading_callback(self, msg: Float32):
        """타겟 헤딩 콜백"""
        self.target_heading = msg.data
        self.update_target_heading_arrow()

    def global_path_callback(self, msg: NavPath):
        """글로벌 경로 콜백"""
        x_vals = [pose.pose.position.x for pose in msg.poses]
        y_vals = [pose.pose.position.y for pose in msg.poses]
        self.global_path_plot.set_xdata(x_vals)
        self.global_path_plot.set_ydata(y_vals)
        plt.draw()
        plt.pause(0.1)

    def approach_path_callback(self, msg: NavPath):
        """어프로치 경로 콜백"""
        x_vals = [pose.pose.position.x for pose in msg.poses]
        y_vals = [pose.pose.position.y for pose in msg.poses]
        self.approach_path_plot.set_xdata(x_vals)
        self.approach_path_plot.set_ydata(y_vals)
        plt.draw()
        plt.pause(0.1)

    # ──────────────────────────────────────────────────────────────────────────
    #                         플롯 업데이트 함수들
    # ──────────────────────────────────────────────────────────────────────────

    def update_current_position_plot(self):
        """현재 로봇 위치(red dot) 업데이트"""
        if self.current_x is not None and self.current_y is not None:
            self.current_pos_plot.set_xdata([self.current_x])
            self.current_pos_plot.set_ydata([self.current_y])
            # 가시화 범위 조절 (옵션)
            self.ax.set_xlim(self.current_x - 10, self.current_x + 10)
            self.ax.set_ylim(self.current_y - 10, self.current_y + 10)
            plt.draw()
            plt.pause(0.1)
            # 헤딩 화살표도 위치가 바뀌면 재갱신
            self.update_current_heading_arrow()

    def update_current_heading_arrow(self):
        """
        현재 헤딩을 화살표처럼 표시하기 위해,
        (current_x, current_y)에서 heading 방향으로 일정 길이(arrow_length)만큼 그린다.
        """
        if self.current_x is not None and self.current_y is not None and self.current_heading is not None:
            arrow_length = 2.0
            dx = arrow_length * math.cos(self.current_heading)
            dy = arrow_length * math.sin(self.current_heading)
            x_vals = [self.current_x, self.current_x + dx]
            y_vals = [self.current_y, self.current_y + dy]
            self.heading_line.set_xdata(x_vals)
            self.heading_line.set_ydata(y_vals)
            plt.draw()
            plt.pause(0.1)

    def update_target_position_plot(self):
        """타겟 좌표 표시(하늘색 X) 업데이트"""
        if self.target_x is not None and self.target_y is not None:
            self.target_pos_plot.set_xdata([self.target_x])
            self.target_pos_plot.set_ydata([self.target_y])
            plt.draw()
            plt.pause(0.1)

    def update_target_heading_arrow(self):
        """
        타겟 헤딩을 화살표처럼 표시하기 위해,
        (target_x, target_y)에서 target_heading 방향으로 일정 길이(arrow_length)만큼 그린다.
        """
        if (self.target_x is not None and 
            self.target_y is not None and 
            self.target_heading is not None):
            arrow_length = 2.0
            dx = arrow_length * math.cos(self.target_heading)
            dy = arrow_length * math.sin(self.target_heading)
            x_vals = [self.target_x, self.target_x + dx]
            y_vals = [self.target_y, self.target_y + dy]
            self.target_heading_line.set_xdata(x_vals)
            self.target_heading_line.set_ydata(y_vals)
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
