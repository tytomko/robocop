import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import networkx as nx
import numpy as np
from scipy.spatial import KDTree
from datetime import datetime
import matplotlib
import matplotlib.pyplot as plt
import threading
import time

matplotlib.use('TkAgg')

# 보간 간격 (보간된 노드 간격)
SEGMENT_DIST = 0.3  
# 노드간 연결 최대 거리
EDGE_CONNECTION_DISTANCE = 0.6  
# (참고용) 연속 위치의 최소 차이 - 여기서는 사용하지 않음.
POSITION_TOLERANCE = 0.1        

class PathMaker(Node):
    def __init__(self):
        super().__init__('path_maker')
        # 엔터 입력 시 기록된 위치들 (사용자가 원하는 지점)
        self.positions = []  
        # 최신 위치만 업데이트 (ROS 토픽)
        self.current_position = None  
        self.graph = nx.Graph()
        self.cluster_centers = {}

        self.recording = True

        # ROS 구독: 최신 위치만 업데이트
        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot_1/utm_pose',
            self.pose_callback,
            10
        )

        # Matplotlib 초기화: 기록된 위치와 현재 위치 표시
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title("Real-time Robot Path")
        self.line_path, = self.ax.plot([], [], 'b.-', label='Recorded Points')
        self.scatter_current = self.ax.scatter([], [], c='red', s=100, label='Current Position')
        self.ax.set_xlabel('X (UTM)')
        self.ax.set_ylabel('Y (UTM)')
        self.ax.set_title('Real-time Robot Path')
        self.ax.legend()
        self.ax.grid(True)

    def pose_callback(self, msg):
        """ROS 토픽으로부터 최신 UTM 위치를 업데이트"""
        pos = (msg.pose.position.x, msg.pose.position.y)
        self.current_position = pos

    def update_live_plot(self):
        """실시간 플롯 갱신: 기록된 점들과 최신 위치 표시"""
        if len(self.positions) > 0:
            x_data, y_data = zip(*self.positions)
            self.line_path.set_xdata(x_data)
            self.line_path.set_ydata(y_data)
        if self.current_position:
            self.scatter_current.set_offsets([self.current_position])
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

    def finalize_in_main_thread(self):
        """
        녹화 종료 후 최종 처리:
        1) 보간을 통해 입력된 점들 사이에 일정 간격 노드 생성
        2) 노드간 연결(엣지 생성)
        3) JSON 파일로 저장
        4) 최종 그래프 시각화
        """
        self.recording = False
        plt.close(self.fig)

        self.generate_fixed_distance_nodes(SEGMENT_DIST)
        self.create_edges()
        self.save_graph_to_json()
        self.plot_final_graph()

    def interpolate_between_points(self, p1, p2, segment_dist):
        """
        두 점 p1, p2 사이를 선형 보간하여 segment_dist 간격의 중간 노드들을 생성.
        거리가 segment_dist 미만이면 빈 리스트 반환.
        """
        vec = np.array(p2) - np.array(p1)
        total_dist = np.linalg.norm(vec)
        if total_dist < segment_dist:
            return []
        num_segments = int(total_dist // segment_dist)
        # p1과 p2 사이에 num_segments개의 보간 점 생성
        points = [tuple(np.array(p1) + vec * (i / (num_segments + 1))) for i in range(1, num_segments + 1)]
        return points

    def generate_fixed_distance_nodes(self, segment_dist):
        """
        사용자가 엔터 입력으로 기록한 positions 사이에 보간 노드를 생성.
        시작점과 끝점는 그대로 사용하고, 중간에 보간된 노드를 추가.
        """
        if len(self.positions) < 2:
            self.get_logger().info("Not enough positions recorded to generate nodes.")
            self.cluster_centers = {}
            return

        nodes = [self.positions[0]]
        for i in range(1, len(self.positions)):
            p_prev = self.positions[i - 1]
            p_curr = self.positions[i]
            intermediate_nodes = self.interpolate_between_points(p_prev, p_curr, segment_dist)
            nodes.extend(intermediate_nodes)
            nodes.append(p_curr)
        self.cluster_centers = {i: node for i, node in enumerate(nodes)}
        self.get_logger().info(f"Generated {len(nodes)} fixed-distance nodes from {len(self.positions)} recorded points.")

    def create_edges(self):
        """노드들 간 거리가 EDGE_CONNECTION_DISTANCE 이내이면 엣지 생성 (cost=거리)"""
        self.graph.clear()
        if self.cluster_centers:
            nodes_array = np.array(list(self.cluster_centers.values()))
            kdtree = KDTree(nodes_array)
            for i, point in enumerate(nodes_array):
                indices = kdtree.query_ball_point(point, EDGE_CONNECTION_DISTANCE)
                for idx in indices:
                    if i != idx:
                        dist = float(np.linalg.norm(point - nodes_array[idx]))
                        self.graph.add_edge(tuple(nodes_array[i]), tuple(nodes_array[idx]), cost=dist)
            self.get_logger().info(f"Total nodes: {len(nodes_array)}, Total edges: {len(self.graph.edges)}")
        else:
            self.get_logger().info("No nodes available for creating edges.")

    def save_graph_to_json(self):
        """NetworkX 그래프를 node-link 형식의 JSON으로 저장"""
        if len(self.graph.nodes) > 0:
            data = nx.node_link_data(self.graph)
            filename = f'global_map_{datetime.now().strftime("%Y%m%d%H%M%S")}.json'
            with open(filename, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f'Graph saved to {filename}')
        else:
            self.get_logger().info('Graph is empty. No file saved.')

    def plot_final_graph(self):
        """노드와 엣지를 최종적으로 시각화"""
        if not self.cluster_centers:
            return
        plt.ioff()
        fig_final, ax_final = plt.subplots()
        fig_final.canvas.manager.set_window_title("Final Graph Visualization")
        centers = np.array(list(self.cluster_centers.values()))
        ax_final.scatter(centers[:, 0], centers[:, 1], c='red', marker='x', s=100, label='Nodes')
        for edge in self.graph.edges(data=True):
            p1, p2, attr = edge
            ax_final.plot([p1[0], p2[0]], [p1[1], p2[1]], 'g--', alpha=1.0)
        ax_final.set_xlabel('X (UTM)')
        ax_final.set_ylabel('Y (UTM)')
        ax_final.set_title('Final Graph (Nodes and Edges)')
        ax_final.legend()
        ax_final.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PathMaker()

    # ROS 스레드 실행
    def ros_spin():
        rclpy.spin(node)
    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    # 엔터 입력 시 현재 위치 기록
    def wait_input():
        while node.recording:
            input("Press Enter to record the current position (or Ctrl+C to finalize)...\n")
            if node.current_position is not None:
                node.positions.append(node.current_position)
                node.get_logger().info(f"Recorded position: {node.current_position}")
            else:
                node.get_logger().info("No current position available.")
    input_thread = threading.Thread(target=wait_input, daemon=True)
    input_thread.start()

    try:
        # 실시간 플롯 갱신 루프
        while rclpy.ok() and node.recording:
            node.update_live_plot()
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Finalizing...")
    finally:
        if node.recording:
            node.finalize_in_main_thread()
        node.destroy_node()
        rclpy.shutdown()

    spin_thread.join()
    input_thread.join()

if __name__ == '__main__':
    main()
