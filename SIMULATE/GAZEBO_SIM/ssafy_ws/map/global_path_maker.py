# 1) ROS 노드(PathMaker)가 /robot_1/utm_pose를 구독해 UTM 위치를 positions에 저장

# 2) update_live_plot()에서 Matplotlib로 실시간 경로(파란 선)와 현재 위치(빨간 점) 표시

# 3) 메인 루프는 입력 스레드(엔터 대기)와 ROS 스레드(토픽 수신)와 함께 동작

# 4) 엔터 입력 시 finalize_in_main_thread() 호출 → 실시간 플롯 종료

# 5) generate_fixed_distance_nodes()로 positions를 0.5m 간격으로 나눠 평균 노드 생성

# 6) create_edges()에서 노드 간 거리가 0.6m 이하면 에지(cost=거리)로 연결

# 7) save_graph_to_json()가 무방향 그래프(노드, 링크, cost)를 JSON으로 저장

# 8) plot_final_graph()는 노드(X), 에지(초록 점선), 원본 경로(파랑)를 최종 시각화

# 9) KeyboardInterrupt나 엔터 입력 시 기록 중단 후 위 로직 수행
# 10) 모든 스레드 종료 후 rclpy.shutdown()으로 프로그램 종료


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

SEGMENT_DIST = 0.5           # 노드 간격(등간격 노드 생성용)
EDGE_CONNECTION_DISTANCE = 0.7  # 노드끼리 연결할 최대 거리
POSITION_TOLERANCE = 0.1        # 연속 포인트 최소 간격

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

    def pose_callback(self, msg):
        """
        로봇에서 수신된 UTM 좌표를 positions에 기록.
        POSITION_TOLERANCE 이하로 차이가 작으면 중복으로 간주, 추가 안 함.
        """
        pos = (msg.pose.position.x, msg.pose.position.y)
        #self.get_logger().info(f"Received position: {pos}")

        if (self.last_position is None
            or np.linalg.norm(np.array(pos) - np.array(self.last_position)) > POSITION_TOLERANCE):
            self.positions.append(pos)
            self.last_position = pos

    def update_live_plot(self):
        """실시간 경로를 그리고, plt.pause()로 갱신"""
        if not self.recording or len(self.positions) == 0:
            return

        x_data, y_data = zip(*self.positions)
        self.line_path.set_xdata(x_data)
        self.line_path.set_ydata(y_data)

        self.scatter_current.set_offsets([x_data[-1], y_data[-1]])

        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.01)

    def finalize_in_main_thread(self):
        """
        엔터 입력 후 최종 처리
        1) 실시간 플롯 닫기
        2) 등간격 노드 생성
        3) 에지 생성 (거리 cost 포함)
        4) JSON 저장
        5) 최종 플롯
        """
        self.recording = False
        plt.close(self.fig)

        # 등간격 노드 만들기
        self.generate_fixed_distance_nodes(segment_dist=SEGMENT_DIST)

        # 노드끼리 연결 (에지에 거리 cost 저장)
        self.create_edges()

        # 그래프 JSON 저장
        self.save_graph_to_json()

        # 최종 그래프 시각화
        self.plot_final_graph()

    def generate_fixed_distance_nodes(self, segment_dist=0.5):
        """
        positions를 segment_dist 간격으로 나누어 각 구간의 평균 위치를 노드로 삼음
        """
        if len(self.positions) < 2:
            self.get_logger().info("positions < 2 => 노드 생성 불가")
            self.cluster_centers = {}
            return

        nodes = []
        segment_pts = [self.positions[0]]
        last_pt = self.positions[0]
        accumulated = 0.0

        for i in range(1, len(self.positions)):
            pt = self.positions[i]
            dist = np.linalg.norm(np.array(pt) - np.array(last_pt))
            accumulated += dist
            segment_pts.append(pt)

            if accumulated >= segment_dist:
                avg = np.mean(segment_pts, axis=0)
                nodes.append(tuple(avg))
                accumulated = 0.0
                segment_pts = [pt]

            last_pt = pt

        # 마지막 구간 처리 (길이가 segment_dist 미만이라도 노드 하나 생성)
        if len(segment_pts) > 1:
            avg = np.mean(segment_pts, axis=0)
            nodes.append(tuple(avg))

        # 노드를 {인덱스: (x,y)} dict로 저장
        self.cluster_centers = {i: node for i, node in enumerate(nodes)}
        self.get_logger().info(f"Generated {len(nodes)} fixed-distance nodes.")

    def create_edges(self):
        """노드들 간 거리가 EDGE_CONNECTION_DISTANCE 이하이면 에지 생성, cost=거리"""
        self.graph.clear()

        if self.cluster_centers:
            cluster_points = np.array(list(self.cluster_centers.values()))
            kdtree = KDTree(cluster_points)

            for i, point in enumerate(cluster_points):
                # point 주변 거리 검사
                indices = kdtree.query_ball_point(point, EDGE_CONNECTION_DISTANCE)
                for idx in indices:
                    if i != idx:
                        # [추가] distance(코스트) 계산
                        dist = float(np.linalg.norm(point - cluster_points[idx]))
                        # 에지에 cost attribute 추가
                        self.graph.add_edge(
                            tuple(cluster_points[i]),
                            tuple(cluster_points[idx]),
                            cost=dist
                        )

            self.get_logger().info(
                f"Total nodes: {len(cluster_points)}, Total edges: {len(self.graph.edges)}"
            )
        else:
            self.get_logger().info("No cluster centers => no edges.")

    def save_graph_to_json(self):
        """NetworkX 그래프 => JSON(node_link_data) 저장"""
        if len(self.graph.nodes) > 0:
            data = nx.node_link_data(self.graph)
            filename = f'global_path_{datetime.now().strftime("%Y%m%d%H%M%S")}.json'
            with open(filename, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f'Graph saved to {filename}')
        else:
            self.get_logger().info('No nodes in graph. Skip saving.')

    def plot_final_graph(self):
        """최종 그래프(원본 경로 제외, 노드 + 에지) 시각화"""
        if len(self.positions) == 0:
            return

        plt.ioff()
        fig_final, ax_final = plt.subplots()
        fig_final.canvas.manager.set_window_title("Final Path Visualization")

        # [원본 경로 표시 부분 제거]
        #arr = np.array(self.positions)
        #ax_final.plot(arr[:, 0], arr[:, 1], 'b.-', label='Final Path')

        # 노드만 표시
        if hasattr(self, 'cluster_centers') and self.cluster_centers:
            centers = np.array(list(self.cluster_centers.values()))
            ax_final.scatter(
                centers[:, 0], centers[:, 1],
                c='red', marker='x', s=100, label='Nodes'
            )

            # 에지 시각화
            for edge in self.graph.edges(data=True):
                p1, p2, attr = edge
                ax_final.plot([p1[0], p2[0]], [p1[1], p2[1]], 'g--', alpha=0.5)

        ax_final.set_xlabel('X (UTM)')
        ax_final.set_ylabel('Y (UTM)')
        ax_final.set_title('Final Graph Only (No Original Path)')
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
            # 실시간 그래프 갱신
            node.update_live_plot()

            # 엔터 입력 시 finalize 후 종료
            if node.request_finalize:
                node.request_finalize = False
                node.finalize_in_main_thread()

                # [추가] 최종 처리 직후, ROS 종료 → 노드도 끝냄
                node.destroy_node()
                rclpy.shutdown()

                break  # 메인 루프 탈출

            time.sleep(0.1)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt -> forced finalize (if not done)")
        if node.recording:
            node.finalize_in_main_thread()
        # 마찬가지로 강제 종료
        node.destroy_node()
        rclpy.shutdown()

    # 모든 스레드 join 후 프로그램 완전 종료
    spin_thread.join()
    input_thread.join()

if __name__ == '__main__':
    main()
