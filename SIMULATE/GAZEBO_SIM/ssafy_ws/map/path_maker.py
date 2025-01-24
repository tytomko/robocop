import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import DBSCAN
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

EPS = 1.0
MIN_SAMPLES = 3
EDGE_CONNECTION_DISTANCE = 1.5
POSITION_TOLERANCE = 0.1

class PathMaker(Node):
    def __init__(self):
        super().__init__('path_maker')
        self.positions = []
        self.last_position = None
        self.graph = nx.Graph()

        self.recording = True           # 녹화 중인지 여부
        self.request_finalize = False   # 최종 처리(엔터 입력) 요청 여부

        # ROS 구독
        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot_1/utm_pose',
            self.pose_callback,
            10
        )

        # matplotlib 초기화
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title("Real-time Robot Path")

        self.line_path, = self.ax.plot([], [], 'b.-', label='Path')
        self.scatter_current = self.ax.scatter([], [], c='red', s=100, label='Current Position')

        self.ax.set_xlabel('X (UTM)')
        self.ax.set_ylabel('Y (UTM)')
        self.ax.set_title('Real-time Robot Path')
        self.ax.legend()
        self.ax.grid(True)

        plt.ion()   # 인터랙티브 모드
        plt.show()

    def pose_callback(self, msg):
        pos = (msg.pose.position.x, msg.pose.position.y)
        if (self.last_position is None
            or np.linalg.norm(np.array(pos) - np.array(self.last_position)) > POSITION_TOLERANCE):
            self.positions.append(pos)
            self.last_position = pos

    def update_live_plot(self):
        """메인 스레드에서 주기적으로 호출"""
        if not self.recording or len(self.positions) == 0:
            return

        x_data, y_data = zip(*self.positions)
        self.line_path.set_xdata(x_data)
        self.line_path.set_ydata(y_data)

        self.scatter_current.set_offsets([x_data[-1], y_data[-1]])

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def finalize_in_main_thread(self):
        """
        메인 스레드에서만 호출해야 하는 최종 처리
        request_finalize == True 일 때 한 번만 실행
        """
        self.recording = False

        self.cluster_positions()
        self.create_edges()
        self.save_graph_to_json()
        self.plot_final_graph()

    def cluster_positions(self):
        if len(self.positions) == 0:
            return
        clustering = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(self.positions)
        labels = clustering.labels_
        unique_labels = set(labels)

        self.nodes = {label: [] for label in unique_labels if label != -1}
        for position, label in zip(self.positions, labels):
            if label != -1:
                self.nodes[label].append(position)

        self.cluster_centers = {
            label: np.mean(np.array(pos_list), axis=0).tolist()
            for label, pos_list in self.nodes.items()
        }

    def create_edges(self):
        self.graph.clear()
        if self.cluster_centers:
            cluster_points = np.array(list(self.cluster_centers.values()))
            kdtree = KDTree(cluster_points)
            for i, point in enumerate(cluster_points):
                indices = kdtree.query_ball_point(point, EDGE_CONNECTION_DISTANCE)
                for idx in indices:
                    if i != idx:
                        self.graph.add_edge(tuple(cluster_points[i]), tuple(cluster_points[idx]))
            self.get_logger().info(f"Total nodes: {len(cluster_points)}, Total edges: {len(self.graph.edges)}")

    def save_graph_to_json(self):
        if len(self.graph.nodes) > 0:
            data = nx.node_link_data(self.graph)
            filename = f'global_path_{datetime.now().strftime("%Y%m%d%H%M%S")}.json'
            with open(filename, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f'Graph saved to {filename}')

    def plot_final_graph(self):
        """
        최종 그래프도 메인 스레드에서 plt.show() 호출
        """
        if len(self.positions) == 0:
            return

        plt.ioff()
        fig_final, ax_final = plt.subplots()
        fig_final.canvas.manager.set_window_title("Final Path Visualization")

        arr = np.array(self.positions)
        ax_final.plot(arr[:, 0], arr[:, 1], 'b.-', label='Final Path')

        if hasattr(self, 'cluster_centers') and self.cluster_centers:
            centers = np.array(list(self.cluster_centers.values()))
            ax_final.scatter(centers[:, 0], centers[:, 1], c='red', marker='x', s=100, label='Cluster Centers')
            for edge in self.graph.edges:
                p1, p2 = edge
                ax_final.plot([p1[0], p2[0]], [p1[1], p2[1]], 'g--', alpha=0.5)

        ax_final.set_xlabel('X (UTM)')
        ax_final.set_ylabel('Y (UTM)')
        ax_final.set_title('Final Path Visualization')
        ax_final.legend()
        ax_final.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PathMaker()

    # ROS 스레드
    def ros_spin():
        rclpy.spin(node)

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    # 엔터 입력 스레드
    def wait_input():
        input("Press Enter to stop recording...\n")
        node.get_logger().info("Enter pressed! request finalize")
        # 메인 스레드가 finalize 처리하도록 플래그만 세팅
        node.request_finalize = True

    input_thread = threading.Thread(target=wait_input, daemon=True)
    input_thread.start()

    # 메인 스레드 루프
    try:
        while rclpy.ok():
            # 1) 실시간 그래프 갱신
            node.update_live_plot()

            # 2) finalize 요청이 있으면 여기서 처리
            if node.request_finalize:
                node.request_finalize = False
                node.finalize_in_main_thread()
                break  # 루프 탈출

            time.sleep(0.1)
    except KeyboardInterrupt:
        # 만약 Ctrl+C 등으로 중단 시
        node.get_logger().info("KeyboardInterrupt -> forced finalize (if not done)")
        if node.recording:
            node.finalize_in_main_thread()

    node.destroy_node()
    rclpy.shutdown()

    spin_thread.join()
    input_thread.join()

if __name__ == '__main__':
    main()
