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

# Constants
SEGMENT_DIST = 0.3  
EDGE_CONNECTION_DISTANCE = 0.6  
POSITION_TOLERANCE = 0.1        
SELECTION_TOLERANCE = 0.5  # 클릭 시 노드 선택 허용 범위

class PathMaker(Node):
    def __init__(self):
        super().__init__('path_maker')
        # 터미널 입력(엔터)로 기록된 위치들
        self.positions = []  
        # 보간 선택 단계에서 생성된 중간 노드들
        self.interpolated_nodes = []  
        # ROS 토픽으로부터 최신 위치 저장
        self.current_position = None  
        self.graph = nx.Graph()
        self.cluster_centers = {}

        self.input_phase = True           # 노드 입력 단계 플래그
        self.interpolation_phase = False  # 보간 선택 단계 플래그
        self.selected_pair = []           # 보간 선택 시 현재 선택된 노드 2개

        # ROS 구독: 최신 위치 업데이트
        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot_1/utm_pose',
            self.pose_callback,
            10
        )

        # Matplotlib 초기화: 실시간 플롯 표시
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
        pos = (msg.pose.position.x, msg.pose.position.y)
        self.current_position = pos

    def update_live_plot(self):
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

    # 노드 입력 단계 종료 (터미널에서 'q' 입력)
    def finalize_node_input(self):
        self.input_phase = False
        self.get_logger().info("Node input phase finished. Recorded {} positions.".format(len(self.positions)))

    # 보간 선택 단계 시작: 기록된 노드 표시 후 이벤트 핸들러 연결
    def start_interpolation_mode(self):
        self.interpolation_phase = True
        self.get_logger().info("Interpolation mode: Select two recorded nodes by clicking. Press 'q' to finish interpolation mode.")
        # 기록된 노드들을 파란 원으로 표시
        recorded_array = np.array(self.positions)
        self.ax.scatter(recorded_array[:, 0], recorded_array[:, 1], c='blue', s=100, marker='o', label='Recorded Nodes')
        plt.draw()
        # 마우스 클릭 및 키보드 이벤트 핸들러 연결
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    # 마우스 클릭 시: 기록된 노드 중 가장 가까운 노드를 선택하여 쌍을 구성
    def on_click(self, event):
        if not self.interpolation_phase:
            return
        if event.inaxes != self.ax:
            return
        clicked = (event.xdata, event.ydata)
        if not self.positions:
            return
        # 클릭 위치와 각 기록된 노드 간의 거리 계산
        distances = [np.linalg.norm(np.array(clicked) - np.array(pos)) for pos in self.positions]
        min_idx = np.argmin(distances)
        if distances[min_idx] > SELECTION_TOLERANCE:
            self.get_logger().info("Clicked position not close enough to any recorded node.")
            return
        selected_node = self.positions[min_idx]
        # 현재 선택된 쌍에 추가 (같은 노드 중복 선택 방지)
        if len(self.selected_pair) == 0 or self.selected_pair[-1] != selected_node:
            self.selected_pair.append(selected_node)
            self.ax.scatter(selected_node[0], selected_node[1], c='magenta', s=150, marker='o')
            plt.draw()
        # 두 노드가 선택되면 보간 진행
        if len(self.selected_pair) == 2:
            p1, p2 = self.selected_pair
            new_nodes = self.interpolate_between_points(p1, p2, SEGMENT_DIST)
            self.interpolated_nodes.extend(new_nodes)
            # 선택된 두 점 사이를 시각적으로 연결
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'c-', linewidth=2)
            plt.draw()
            self.selected_pair = []  # 다음 쌍 선택을 위해 초기화

    # 보간 선택 단계에서 'q' 키 입력 시 종료
    def on_key_press(self, event):
        if event.key == 'q':
            self.interpolation_phase = False
            # 이벤트 핸들러 해제
            self.fig.canvas.mpl_disconnect(self.cid_click)
            self.fig.canvas.mpl_disconnect(self.cid_key)
            plt.close(self.fig)
            self.finalize_interpolation_phase()

    # 보간 선택 단계 종료 후 최종 그래프 생성
    def finalize_interpolation_phase(self):
        # 원본 노드와 보간된 노드들을 합치고 중복 제거
        all_nodes = self.positions + self.interpolated_nodes
        unique_nodes = list({tuple(n) for n in all_nodes})
        self.cluster_centers = {i: node for i, node in enumerate(unique_nodes)}
        self.get_logger().info("Total unique nodes after interpolation: {}".format(len(self.cluster_centers)))
        self.create_edges()
        self.save_graph_to_json()
        self.plot_final_graph()

    # 두 점 사이 선형 보간 (중간 노드 생성)
    def interpolate_between_points(self, p1, p2, segment_dist):
        vec = np.array(p2) - np.array(p1)
        total_dist = np.linalg.norm(vec)
        if total_dist < segment_dist:
            return []
        num_segments = int(total_dist // segment_dist)
        points = [tuple(np.array(p1) + vec * (i / (num_segments + 1))) for i in range(1, num_segments + 1)]
        return points

    def create_edges(self):
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
            self.get_logger().info("Total nodes: {}, Total edges: {}".format(len(nodes_array), len(self.graph.edges)))
        else:
            self.get_logger().info("No nodes available for creating edges.")

    def save_graph_to_json(self):
        if len(self.graph.nodes) > 0:
            data = nx.node_link_data(self.graph)
            filename = f'global_map_{datetime.now().strftime("%Y%m%d%H%M%S")}.json'
            with open(filename, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f'Graph saved to {filename}')
        else:
            self.get_logger().info('Graph is empty. No file saved.')

    def plot_final_graph(self):
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

    # ROS 스핀 스레드 실행
    def ros_spin():
        rclpy.spin(node)
    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    # 터미널 입력: Enter는 현재 위치 기록, 'q'는 노드 입력 종료
    def wait_input():
        while node.input_phase:
            user_input = input("Press Enter to record the current position (or type 'q' to finish node input)...\n")
            if user_input.lower() == 'q':
                node.finalize_node_input()
                break
            else:
                if node.current_position is not None:
                    node.positions.append(node.current_position)
                    node.get_logger().info(f"Recorded position: {node.current_position}")
                else:
                    node.get_logger().info("No current position available.")
    input_thread = threading.Thread(target=wait_input, daemon=True)
    input_thread.start()

    # 실시간 플롯 갱신 (노드 입력 단계)
    try:
        while rclpy.ok() and node.input_phase:
            node.update_live_plot()
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received during node input.")
        node.finalize_node_input()

    input_thread.join()

    # 노드 입력 단계가 종료되면 보간 선택 단계 시작
    node.start_interpolation_mode()
    plt.show()  # 보간 선택 단계의 이벤트 루프가 실행됨 (종료 시 'q'를 누름)

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
