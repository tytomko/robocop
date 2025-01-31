import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Path

# 커스텀 서비스
from robot_custom_interfaces.srv import Homing
from robot_custom_interfaces.srv import Navigate
from robot_custom_interfaces.srv import Patrol

import json
import networkx as nx
import numpy as np
import os
import sys
import matplotlib.pyplot as plt

# 1) 전역 상수 및 초기 설정
global_map = 'map/global_map.json'
home_pose = (304412.92687916575, 3892844.1526765698)  # UTM 좌표

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')

        # (옵션) 플롯 활성화 여부
        self.enable_plot = True

        # 2) 파라미터: robot_number
        self.declare_parameter('robot_number', 1)
        robot_number = self.get_parameter('robot_number').value

        # 3) Topic / Service 명 지정
        pose_topic = f"/robot_{robot_number}/utm_pose"
        global_path_topic = f"/robot_{robot_number}/global_path"
        homing_service = f"/robot_{robot_number}/homing"
        navigate_service = f"/robot_{robot_number}/navigate"
        patrol_service = f"/robot_{robot_number}/patrol"

        # 4) 구독자, 퍼블리셔, 서비스 서버 생성
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10
        )
        self.homing_srv = self.create_service(Homing, homing_service, self.homing_callback)
        self.nav_srv = self.create_service(Navigate, navigate_service, self.navigate_callback)
        self.patrol_srv = self.create_service(Patrol, patrol_service, self.patrol_callback)

        self.path_pub = self.create_publisher(Path, global_path_topic, 10)

        # 5) 그래프(지도) 로드
        self.graph = self.load_graph(global_map)
        if self.graph is None:
            self.get_logger().error("Failed to load the global path graph. Exiting...")
            sys.exit(1)

        # 6) 내부 상태
        self.current_pos = None  # 현재 로봇 위치 (x, y)
        self.goal_pos = None     # 마지막 목표 위치 (x, y)
        self.origin = None       # 좌표 변환용 기준점(최초 위치)

        # 로그 출력
        self.get_logger().info(f"Subscribed to {pose_topic}")
        self.get_logger().info(f"Global path will be published on {global_path_topic}")
        self.get_logger().info("Homing & Navigate & Patrol service servers ready")

    # ---------------------------------------------------------------------------- #
    #                               콜백 & 메인 로직                                #
    # ---------------------------------------------------------------------------- #

    def pose_callback(self, msg: PoseStamped):
        """로봇의 현재 UTM 위치를 수신"""
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)
        if self.origin is None:  # 최초 위치를 기준점으로 설정
            self.origin = self.current_pos

    def homing_callback(self, request, response):
        """Homing 서비스: 현재 위치 -> home_pose 경로를 찾아서 발행"""
        self.goal_pos = home_pose
        self.get_logger().info(f"[Homing] Goal -> {self.goal_pos}")
        return self.process_navigation_request(response)

    def navigate_callback(self, request, response):
        """Navigate 서비스: 현재 위치 -> request.goal(x,y) 경로를 찾아서 발행"""
        self.goal_pos = (request.goal.x, request.goal.y)
        self.get_logger().info(f"[Navigate] Goal -> {self.goal_pos}")
        return self.process_navigation_request(response)

    def patrol_callback(self, request, response):
        """
        Patrol 서비스:
          - 현재 위치 -> goals[0]
          - goals[0] -> goals[1]
          - ...
          - goals[n-2] -> goals[n-1]
          모든 구간 경로를 순차적으로 찾아서 이어붙이고, 한 번에 발행

          +++ 변경점 +++
          최종적으로 플롯을 그릴 때, request.goals 배열에 들어있는
          중간 목표 지점들도 함께 표시하도록 goals(웨이포인트 목록)를 넘김
        """
        self.get_logger().info(f"[Patrol] Received multi-goals: {request.goals}")

        if not self.current_pos:
            response.success = False
            response.message = "Current position is unknown."
            return response

        start_node = self.find_nearest_node(self.current_pos)
        if start_node is None:
            self.get_logger().error("Cannot find nearest node to current position.")
            response.success = False
            response.message = "Failed to find start node."
            return response

        full_path_nodes = []  # (x, y) 누적 경로
        prev_node = start_node

        for idx, goal in enumerate(request.goals):
            goal_pos = (goal.x, goal.y)
            goal_node = self.find_nearest_node(goal_pos)
            if goal_node is None:
                self.get_logger().error(f"Cannot find nearest node for goal#{idx} at {goal_pos}")
                response.success = False
                response.message = f"Failed to find node for goal {idx}."
                return response

            self.get_logger().info(f"Finding path from {prev_node} to {goal_node} (goal {idx})")
            sub_path = self.find_shortest_path_nodes(prev_node, goal_node)
            if not sub_path:
                response.success = False
                response.message = f"Failed to find valid path for goal {idx}."
                return response

            if idx == 0:
                full_path_nodes += sub_path
            else:
                # 첫 노드 중복 제거
                full_path_nodes += sub_path[1:]

            prev_node = goal_node

        if len(full_path_nodes) <= 1:
            response.success = False
            response.message = "No valid multi-goal path found."
            return response

        self.get_logger().info(f"Multi-goal path node count: {len(full_path_nodes)}")

        # 로컬 변수에 최종 경로 저장 (시각화용)
        # self.goal_pos는 "마지막 목표"만 저장
        last_goal = request.goals[-1]
        self.goal_pos = (last_goal.x, last_goal.y)  # 마지막 목표

        # 1) 경로 발행
        self.publish_path(full_path_nodes)

        # 2) 시각화 (중간 목표점도 표시하기 위해 goals=request.goals 전달)
        if self.enable_plot:
            self.visualize_path(full_path_nodes, multi_goal=True, goals=request.goals)

        response.success = True
        response.message = "Multi-goal path calculated and published successfully."
        return response

    def process_navigation_request(self, response):
        """
        Homing/Navigate 전용:
        (현재 위치 -> self.goal_pos) 경로를 A*로 찾고 단일 발행
        """
        if not self.current_pos:
            response.success = False
            response.message = "Current position is unknown."
            return response

        path = self.find_shortest_path(self.current_pos, self.goal_pos)
        if path:
            self.publish_path(path)
            if self.enable_plot:
                self.visualize_path(path)
            response.success = True
            response.message = "Path calculated and published successfully."
        else:
            response.success = False
            response.message = "Failed to find a valid path."
        return response

    # ---------------------------------------------------------------------------- #
    #                           그래프 & 경로 처리 로직                             #
    # ---------------------------------------------------------------------------- #

    def load_graph(self, file_path):
        """JSON 파일을 읽어 그래프 로드"""
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
            # 가중치(거리) 계산
            G.add_edge(source, target, weight=self.euclidean_distance(source, target))

        self.get_logger().info("Graph loaded successfully")
        return G

    def euclidean_distance(self, point1, point2):
        """2D 유클리드 거리 계산"""
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def find_nearest_node(self, position):
        """주어진 (x,y)에 가장 가까운 노드(그래프 상 key)를 찾는다."""
        closest_node = None
        min_distance = float('inf')
        for node in self.graph.nodes:
            dist = self.euclidean_distance(position, node)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        return closest_node

    def find_shortest_path_nodes(self, start_node, goal_node):
        """이미 노드를 알고 있을 때, A* 경로를 구해 (x,y) 리스트로 리턴"""
        try:
            path = nx.astar_path(self.graph, start_node, goal_node, weight='weight')
            return path
        except nx.NetworkXNoPath:
            self.get_logger().error(f"No path found from {start_node} to {goal_node}")
            return None

    def find_shortest_path(self, start_pos, goal_pos):
        """(x,y) -> (x,y) A* 경로"""
        start_node = self.find_nearest_node(start_pos)
        goal_node = self.find_nearest_node(goal_pos)
        if start_node and goal_node:
            self.get_logger().info(f"Start node: {start_node}, Goal node: {goal_node}")
            return self.find_shortest_path_nodes(start_node, goal_node)
        else:
            self.get_logger().error("Start or Goal node not found.")
            return None

    # ---------------------------------------------------------------------------- #
    #                   경로 발행 & 시각화 (path: [(x1,y1), (x2,y2), ...])          #
    # ---------------------------------------------------------------------------- #

    def publish_path(self, path):
        """
        path: (x,y) 튜플의 리스트
        /robot_{robot_number}/global_path 토픽 (nav_msgs/Path) 로 발행
        """
        if self.origin is None:
            self.get_logger().error("Origin is not set yet.")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"

            # UTM -> origin 기준 상대 좌표
            pose.pose.position.x = node[0] - self.origin[0]
            pose.pose.position.y = node[1] - self.origin[1]
            pose.pose.position.z = 0.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published global path with {len(path)} nodes")

    def visualize_path(self, path, multi_goal=False, goals=None):
        """
        path: (x,y) 리스트
        Matplotlib으로 시각화

        +++ 변경점 +++
        - goals: Patrol 시 여러 웨이포인트(geometry_msgs/Pose2D[])를 표시
        """
        plt.figure(figsize=(8, 6))

        x_path = [p[0] for p in path]
        y_path = [p[1] for p in path]

        # 경로 표시
        plt.plot(x_path, y_path, 'ro-', label='Path')

        # 현재 위치(파란색 점)
        if self.current_pos:
            plt.scatter(self.current_pos[0], self.current_pos[1],
                        c='blue', label='Current Position', s=100)

        # (단일 목표) Navigate/Homing만 초록색 점
        if self.goal_pos and not multi_goal:
            plt.scatter(self.goal_pos[0], self.goal_pos[1],
                        c='green', label='Goal Position', s=100)

        # (다중 목표) Patrol: request.goals 목록도 표시
        if multi_goal and goals:
            for i, g in enumerate(goals):
                # 실제 좌표 (UTM): g.x, g.y
                # 중복 라벨 방지
                label_str = 'Goals' if i == 0 else None
                plt.scatter(g.x, g.y,
                            marker='*', c='purple', s=150,
                            label=label_str)
                # 각 점에 순번 표시
                plt.text(g.x, g.y, f'{i+1}', color='black', fontsize=12)

        plt.xlabel("X")
        plt.ylabel("Y")
        title_str = "Multi-Goal Path" if multi_goal else "A* Global Path"
        plt.title(title_str)
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
