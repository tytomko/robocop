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

# GUI 환경일 경우 'TkAgg' 또는 'Qt5Agg' 등을 사용할 수 있습니다.
# GUI가 없는(SSH, Docker 등) 환경이라면 'Agg'로 변경 후 plt.show() 대신 이미지를 저장해야 합니다.
matplotlib.use('TkAgg')

# DBSCAN 파라미터 (필요 시 조정)
EPS = 0.2        # 클러스터링 반경
MIN_SAMPLES = 1  # 최소 포인트 개수(군집 형성 조건)
EDGE_CONNECTION_DISTANCE = 1.5
POSITION_TOLERANCE = 0.1

class PathMaker(Node):
    def __init__(self):
        super().__init__('path_maker')
        self.positions = []
        self.last_position = None
        self.graph = nx.Graph()

        self.recording = True
        self.request_finalize = False

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

        # 이 시점에서 plt.show() / plt.ion()을 사용하면 main 스레드가 블로킹되므로,
        # 대신 update_live_plot()에서 plt.pause()를 통해 인터랙션을 관리합니다.

    def pose_callback(self, msg):
        """로봇에서 수신된 UTM 좌표를 positions에 기록"""
        pos = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Received position: {pos}")

        # 이전 위치와의 거리 차이가 POSITION_TOLERANCE보다 크면 추가
        if (self.last_position is None
            or np.linalg.norm(np.array(pos) - np.array(self.last_position)) > POSITION_TOLERANCE):
            self.positions.append(pos)
            self.last_position = pos

    def update_live_plot(self):
        """실시간 경로를 그리는 함수, 메인 스레드에서 주기적으로 호출"""
        if not self.recording or len(self.positions) == 0:
            return

        x_data, y_data = zip(*self.positions)
        self.line_path.set_xdata(x_data)
        self.line_path.set_ydata(y_data)

        self.scatter_current.set_offsets([x_data[-1], y_data[-1]])

        self.ax.relim()
        self.ax.autoscale_view()

        # plt.ion() 대신 draw + pause 사용
        plt.draw()
        plt.pause(0.01)

    def finalize_in_main_thread(self):
        """엔터 입력 혹은 KeyboardInterrupt 등으로 녹화를 중단할 때 최종 처리"""
        self.recording = False

        # (추가) 실시간 plot 닫기
        plt.close(self.fig)

        self.cluster_positions()
        self.create_edges()
        self.save_graph_to_json()
        self.plot_final_graph()

    def cluster_positions(self):
        """DBSCAN으로 positions를 클러스터링"""
        if len(self.positions) == 0:
            return

        clustering = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(self.positions)
        labels = clustering.labels_
        unique_labels = set(labels)

        self.get_logger().info(f"DBSCAN labels: {labels}")
        self.get_logger().info(f"Unique labels: {unique_labels}")

        # -1 라벨은 outlier
        self.nodes = {label: [] for label in unique_labels if label != -1}
        for position, label in zip(self.positions, labels):
            if label != -1:
                self.nodes[label].append(position)

        # 각 군집별 평균점(중심)을 구함
        self.cluster_centers = {
            label: np.mean(np.array(pos_list), axis=0).tolist()
            for label, pos_list in self.nodes.items()
        }

        self.get_logger().info(f"Cluster centers: {self.cluster_centers}")

    def create_edges(self):
        """cluster_centers들 사이 거리로 무방향 에지 생성"""
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
        else:
            self.get_logger().info("No cluster centers => no edges.")

    def save_graph_to_json(self):
        """NetworkX 그래프를 JSON으로 저장"""
        if len(self.graph.nodes) > 0:
            data = nx.node_link_data(self.graph)
            filename = f'global_path_{datetime.now().strftime("%Y%m%d%H%M%S")}.json'
            with open(filename, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f'Graph saved to {filename}')
        else:
            self.get_logger().info('No nodes in graph. Skip saving.')

    def plot_final_graph(self):
        """엔터 입력 후 최종 그래프 시각화"""
        if len(self.positions) == 0:
            return

        plt.ioff()  # 인터랙티브 모드 해제
        fig_final, ax_final = plt.subplots()
        fig_final.canvas.manager.set_window_title("Final Path Visualization")

        arr = np.array(self.positions)
        ax_final.plot(arr[:, 0], arr[:, 1], 'b.-', label='Final Path')

        # 클러스터 중심 표시
        if hasattr(self, 'cluster_centers') and self.cluster_centers:
            centers = np.array(list(self.cluster_centers.values()))
            ax_final.scatter(
                centers[:, 0], centers[:, 1],
                c='red', marker='x', s=100, label='Cluster Centers'
            )

            # 군집간 에지 시각화
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
        node.request_finalize = True

    input_thread = threading.Thread(target=wait_input, daemon=True)
    input_thread.start()

    # 메인 스레드 루프
    try:
        while rclpy.ok():
            node.update_live_plot()

            if node.request_finalize:
                node.request_finalize = False
                node.finalize_in_main_thread()
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt -> forced finalize (if not done)")
        if node.recording:
            node.finalize_in_main_thread()

    node.destroy_node()
    rclpy.shutdown()

    spin_thread.join()
    input_thread.join()

if __name__ == '__main__':
    main()
