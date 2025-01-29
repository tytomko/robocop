import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from robot_custom_interfaces.srv import Homing  # 커스텀 서비스 import
from robot_custom_interfaces.srv import Navigate  # 커스텀 서비스 import
import json
import networkx as nx
import numpy as np
import os
import sys  # 종료를 명확히 하기 위해 추가
import matplotlib.pyplot as plt  # 플롯 추가

# 250129 global_path에서 global_map으로 변경(이름 중복 방지)
global_map = 'map/global_map.json'
home_pose = (304412.92687916575, 3892844.1526765698)  # UTM 좌표

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')

        # 플롯 활성화 여부 (True = 활성화, False = 비활성화)
        self.enable_plot = True  # 필요하면 False로 설정 가능

        # 파라미터 선언 및 가져오기
        self.declare_parameter('robot_number', 1)
        robot_number = self.get_parameter('robot_number').value
        pose_topic = f"/robot_{robot_number}/utm_pose"
        global_path_topic = f"/robot_{robot_number}/global_path"
        homing_service = f"/robot_{robot_number}/homing"
        navigate_service = f"/robot_{robot_number}/navigate"

        # 현재 위치 구독
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            10
        )

        # Homing 서비스 서버 생성
        self.homing_srv = self.create_service(Homing, homing_service, self.homing_callback)

        # Navigate 서비스 서버 생성
        self.nav_srv = self.create_service(Navigate, navigate_service, self.navigate_callback)

        # 경로 발행을 위한 퍼블리셔 생성
        self.path_pub = self.create_publisher(Path, global_path_topic, 10)

        # JSON 파일 로드
        self.graph = self.load_graph(global_map)

        # 그래프 로드 실패 시 종료
        if self.graph is None:
            self.get_logger().error("Failed to load the global path graph. Exiting...")
            sys.exit(1)  # 프로그램 종료

        # 목적지 설정 (초기값 없음)
        self.goal_pos = None
        self.current_pos = None
        self.origin = None  # 기준점을 최초 위치로 설정 가능

        self.get_logger().info(f"Subscribed to {pose_topic}")
        self.get_logger().info(f"Global path will be published on {global_path_topic}")
        self.get_logger().info("Homing & Navigate service servers ready")

    def pose_callback(self, msg):
        """현재 위치 콜백"""
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)

        # 기준점(Origin) 설정 (최초 위치를 기준점으로 사용 가능)
        if self.origin is None:
            self.origin = self.current_pos

    def homing_callback(self, request, response):
        """🚀 Homing 서비스 요청 처리 (request, response 추가)"""
        self.goal_pos = home_pose  # 홈 포즈로 이동
        self.get_logger().info(f"Received Homing request: Goal -> {self.goal_pos}")

        return self.process_navigation_request(response)

    def navigate_callback(self, request, response):
        """🚀 Navigate 서비스 요청 처리"""
        self.goal_pos = (request.x, request.y)  # 요청된 좌표로 이동
        self.get_logger().info(f"Received Navigate request: Goal -> {self.goal_pos}")

        return self.process_navigation_request(response)

    def process_navigation_request(self, response):
        """🚀 A* 경로 생성 및 발행을 처리하는 공통 함수"""
        if self.current_pos:
            path = self.find_shortest_path(self.current_pos, self.goal_pos)
            if path:
                self.publish_path(path)

                # 플롯 활성화 시 경로 시각화
                if self.enable_plot:
                    self.visualize_path(path)

                response.success = True
                response.message = "Path calculated and published successfully."
            else:
                response.success = False
                response.message = "Failed to find a valid path."
        else:
            response.success = False
            response.message = "Current position is unknown."
        
        return response

    def load_graph(self, file_path):
        """JSON 파일을 읽고 그래프 로드"""
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return None

        with open(file_path, 'r') as f:
            data = json.load(f)

        G = nx.Graph() if not data["directed"] else nx.DiGraph()

        # 노드 추가
        for node in data["nodes"]:
            G.add_node(tuple(node["id"]))

        # 엣지 추가
        for link in data["links"]:
            source = tuple(link["source"])
            target = tuple(link["target"])
            G.add_edge(source, target, weight=self.euclidean_distance(source, target))

        self.get_logger().info("Graph loaded successfully")
        return G

    def euclidean_distance(self, point1, point2):
        """유클리드 거리 계산"""
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def find_nearest_node(self, position):
        """주어진 위치에서 가장 가까운 노드 찾기"""
        closest_node = None
        min_distance = float('inf')
        for node in self.graph.nodes:
            dist = self.euclidean_distance(position, node)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        return closest_node

    def find_shortest_path(self, start_pos, goal_pos):
        """A* 알고리즘을 사용한 최단 경로 탐색"""
        start_node = self.find_nearest_node(start_pos)
        goal_node = self.find_nearest_node(goal_pos)

        if start_node and goal_node:
            self.get_logger().info(f"Start: {start_node}, Goal: {goal_node}")
            path = nx.astar_path(self.graph, start_node, goal_node, weight='weight')
            return path
        else:
            self.get_logger().error("Start or Goal node not found.")
            return None

    def publish_path(self, path):
        """🚀 경로를 /robot_{robot_number}/global_path 토픽으로 발행"""
        if self.origin is None:
            self.get_logger().error("Origin position is not set yet.")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"  # 좌표계 설정
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"

            # UTM 좌표 → 상대 좌표 변환 가능 (필요시 사용)
            pose.pose.position.x = node[0] - self.origin[0]
            pose.pose.position.y = node[1] - self.origin[1]
            pose.pose.position.z = 0.0
            
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published global path (relative coordinates)")

    def visualize_path(self, path):
        """🚀 Matplotlib을 사용한 경로 시각화"""
        plt.figure(figsize=(8, 6))

        # A* 경로 플롯
        x_path = [node[0] for node in path]
        y_path = [node[1] for node in path]
        plt.plot(x_path, y_path, 'ro-', label='A* Path')

        # 현재 위치 및 목표 위치 표시
        plt.scatter(self.current_pos[0], self.current_pos[1], c='blue', label='Current Position', s=100)
        plt.scatter(self.goal_pos[0], self.goal_pos[1], c='green', label='Goal Position', s=100)

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("A* Global Path Visualization")
        plt.legend()
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    planner = GlobalPathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
