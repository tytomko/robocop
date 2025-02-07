import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Path

# 커스텀 서비스
from robot_custom_interfaces.srv import Homing, Navigate, Patrol

import json
import networkx as nx
import numpy as np
import os
import sys
import matplotlib.pyplot as plt

# 전역 상수 및 초기 설정
global_map = 'map/global_map.json'
# 이전 홈 좌표
#home_pose = (304412.92687916575, 3892844.1526765698)  # UTM 좌표
home_pose = (304411.67767234833, 3892844.1732352837)

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')

        # (옵션) 플롯 활성화 여부
        self.enable_plot = False

        # [변경] 백그라운드 맵(그래프) 표시 여부
        self.enable_background_map = True  # 필요하면 False로 바꾸면 됨

        # 파라미터: robot_number
        self.declare_parameter('robot_number', 1)
        robot_number = self.get_parameter('robot_number').value

        # Topic / Service 명 지정
        pose_topic = f"/robot_{robot_number}/utm_pose"
        global_path_topic = f"/robot_{robot_number}/global_path"
        approach_path_topic = f"/robot_{robot_number}/approach_path"
        
        # 서비스명을 다르게하던가 해야함.
        # status에 들어가는 서비스랑 이름이 같아서 충돌남
        # 따라서 _plan을 붙여서 서비스명을 다르게 해줌
        homing_service = f"/robot_{robot_number}/homing_plan"
        navigate_service = f"/robot_{robot_number}/navigate_plan"
        patrol_service = f"/robot_{robot_number}/patrol_plan"

        # 구독자, 퍼블리셔, 서비스 서버 생성
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10
        )
        self.homing_srv = self.create_service(Homing, homing_service, self.homing_callback)
        self.nav_srv = self.create_service(Navigate, navigate_service, self.navigate_callback)
        self.patrol_srv = self.create_service(Patrol, patrol_service, self.patrol_callback)

        # 퍼블리셔 분리: 글로벌 패트롤 경로와 접근 경로를 각각 발행
        self.global_path_pub = self.create_publisher(Path, global_path_topic, 10)
        self.approach_path_pub = self.create_publisher(Path, approach_path_topic, 10)

        # 그래프(지도) 로드
        self.graph = self.load_graph(global_map)
        if self.graph is None:
            self.get_logger().error("Failed to load the global path graph. Exiting...")
            sys.exit(1)

        # 내부 상태
        self.current_pos = None  # 현재 로봇 위치 (x, y)
        self.goal_pos = None     # 마지막 목표 위치 (x, y)
        self.origin = None       # 좌표 변환용 기준점(최초 위치)

        # 로그 출력
        self.get_logger().info(f"Subscribed to {pose_topic}")
        self.get_logger().info(f"Global path will be published on {global_path_topic}")
        self.get_logger().info("Homing & Navigate & Patrol service servers ready")

    # ------------------------------------------------------------------------- #
    #                             콜백 및 서비스 로직                           #
    # ------------------------------------------------------------------------- #

    def pose_callback(self, msg: PoseStamped):
        """로봇의 현재 UTM 위치를 수신"""
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)

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
        1. 사용자가 전달한 각 점들을 이용해 Global Path를 생성  
           - 각 목표점 사이를 Global_map에서 A* 알고리즘을 이용해 최단 경로로 연결  
        2. 현재 위치에서 Patrol Path(사용자가 준 모든 목표 지점을 포함하는 글로벌 경로)까지
           실제 이동 경로(내비게이션 경로 길이)를 A* 알고리즘으로 계산하여, 가장 짧은 경로를 Approach Path로 생성  
           수신 측은 Approach Path를 따라 이동한 후, Patrol Path를 따라 순찰을 수행한다.
        """
        self.get_logger().info(f"[Patrol] Received multi-goals: {request.goals}")

        # 현재 위치 체크
        if not self.current_pos:
            response.success = False
            response.message = "Current position is unknown."
            return response

        # --- Step 1: Global Path 생성 (목표점 간 경로 계산) ---
        global_path_nodes = []  # Global Path에 해당하는 노드들의 리스트
        prev_goal_node = None

        for idx, goal in enumerate(request.goals):
            goal_pos = (goal.x, goal.y)
            goal_node = self.find_nearest_node(goal_pos)
            if goal_node is None:
                self.get_logger().error(f"Cannot find nearest node for goal#{idx} at {goal_pos}")
                response.success = False
                response.message = f"Failed to find node for goal {idx}."
                return response

            if idx == 0:
                # 첫 목표점은 바로 global_path에 추가
                global_path_nodes.append(goal_node)
            else:
                # 이전 목표 노드와 현재 목표 노드 사이의 최단 경로 계산
                sub_path = self.find_shortest_path_nodes(prev_goal_node, goal_node)
                if not sub_path:
                    self.get_logger().error(f"Failed to find valid global path for goal {idx}.")
                    response.success = False
                    response.message = f"Failed to find valid global path for goal {idx}."
                    return response

                # 중복되는 첫 노드 제거 후 경로 연결
                global_path_nodes.extend(sub_path[1:])

            prev_goal_node = goal_node

        if len(global_path_nodes) == 0:
            response.success = False
            response.message = "No valid global path found."
            return response

        self.get_logger().info(f"Global path node count: {len(global_path_nodes)}")

        # --- 헬퍼 함수: 경로 길이 계산 ---
        def compute_path_length(path):
            length = 0.0
            for i in range(len(path) - 1):
                length += self.euclidean_distance(path[i], path[i + 1])
            return length

        # --- Step 2: Approach Path 생성 (현재 위치에서 Patrol Path까지) ---
        current_node = self.find_nearest_node(self.current_pos)
        if current_node is None:
            self.get_logger().error("Cannot find nearest node to current position.")
            response.success = False
            response.message = "Failed to find current position node."
            return response

        min_length = float('inf')
        chosen_index = None
        approach_path = None

        for idx, node in enumerate(global_path_nodes):
            path = self.find_shortest_path_nodes(current_node, node)
            if not path:
                continue  # 해당 노드로 가는 경로가 없으면 건너뜀

            length = compute_path_length(path)
            if length < min_length:
                min_length = length
                chosen_index = idx
                approach_path = path

        if chosen_index is None:
            self.get_logger().error("Failed to generate an approach path to the patrol path.")
            response.success = False
            response.message = "Failed to generate approach path."
            return response

        self.get_logger().info(f"Chosen approach path length: {min_length}")

        threshold = 1.0  # 예: 1미터 이하이면 이미 경로 위로 간주
        if min_length < threshold:
            approach_path = []
            self.get_logger().info("Current position is on or very near the patrol path. No approach path needed.")
        else:
            self.get_logger().info(f"Approach path node count: {len(approach_path)}")

        # --- Step 3: 경로 발행 및 시각화 ---
        if approach_path:
            self.publish_approach_path(approach_path)

        self.global_patrol_path = global_path_nodes
        self.publish_global_path(self.global_patrol_path)

        if self.enable_plot:
            self.visualize_path(approach_path, multi_goal=True, goals=request.goals, patrol_path=self.global_patrol_path)

        response.success = True
        response.message = "Approach path and global patrol path calculated and published successfully."
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
            self.publish_global_path(path)
            if self.enable_plot:
                self.visualize_path(path)
            response.success = True
            response.message = "Path calculated and published successfully."
        else:
            response.success = False
            response.message = "Failed to find a valid path."
        return response

    # ------------------------------------------------------------------------- #
    #                         그래프 & 경로 처리 로직                           #
    # ------------------------------------------------------------------------- #

    def load_graph(self, file_path):
        """JSON 파일을 읽어 그래프를 로드하고, 노드 배열을 캐싱"""
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return None

        with open(file_path, 'r') as f:
            data = json.load(f)

        # directed 여부에 따라 그래프 타입 선택
        G = nx.Graph() if not data.get("directed", False) else nx.DiGraph()

        # 노드 추가
        for node in data["nodes"]:
            G.add_node(tuple(node["id"]))

        # 엣지 추가 (가중치는 유클리드 거리)
        for link in data["links"]:
            source = tuple(link["source"])
            target = tuple(link["target"])
            G.add_edge(source, target, weight=self.euclidean_distance(source, target))

        # 최적화를 위해 노드 리스트와 numpy 배열 생성 (find_nearest_node에 사용)
        self.node_list = list(G.nodes)
        self.node_array = np.array(self.node_list)

        self.get_logger().info("Graph loaded successfully")
        return G

    def euclidean_distance(self, point1, point2):
        """2D 유클리드 거리 계산"""
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def find_nearest_node(self, position):
        """
        주어진 (x,y)에 가장 가까운 노드(그래프 상 key)를 벡터화된 방식으로 찾는다.
        → self.node_array를 이용해 한 번에 모든 노드와의 거리를 계산
        """
        pos = np.array(position)
        distances = np.linalg.norm(self.node_array - pos, axis=1)
        min_index = np.argmin(distances)
        return self.node_list[min_index]

    def find_shortest_path_nodes(self, start_node, goal_node):
        """이미 노드를 알고 있을 때, A* 경로를 구해 (x,y) 리스트로 리턴"""
        try:
            path = nx.astar_path(self.graph, start_node, goal_node, weight='weight')
            return path
        except nx.NetworkXNoPath:
            self.get_logger().error(f"No path found from {start_node} to {goal_node}")
            return None

    def find_shortest_path(self, start_pos, goal_pos):
        """(x,y) → (x,y) A* 경로 계산 (노드 검색 포함)"""
        start_node = self.find_nearest_node(start_pos)
        goal_node = self.find_nearest_node(goal_pos)
        if start_node and goal_node:
            self.get_logger().info(f"Start node: {start_node}, Goal node: {goal_node}")
            return self.find_shortest_path_nodes(start_node, goal_node)
        else:
            self.get_logger().error("Start or Goal node not found.")
            return None

    # ------------------------------------------------------------------------- #
    #               경로 메시지 생성 및 퍼블리셔를 통한 발행 함수               #
    # ------------------------------------------------------------------------- #

    def create_path_msg(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            # -- 절대 좌표 그대로 반영 --
            pose.pose.position.x = node[0]
            pose.pose.position.y = node[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        return path_msg

    def publish_global_path(self, path):
        path_msg = self.create_path_msg(path)
        self.global_path_pub.publish(path_msg)
        self.get_logger().info(f"Published global path with {len(path)} nodes")

    def publish_approach_path(self, path):
        path_msg = self.create_path_msg(path)
        self.approach_path_pub.publish(path_msg)
        self.get_logger().info(f"Published approach path with {len(path)} nodes")

    def visualize_path(self, path, multi_goal=False, goals=None, patrol_path=None):
        """
        path: (x,y) 리스트 (예: Approach Path)
        multi_goal: 여러 목표인 경우 True
        goals: geometry_msgs/Pose2D[] (Patrol 시 웨이포인트 표시)
        patrol_path: Global Patrol Path (추가 시각화)
        """
        plt.figure(figsize=(8, 6))

        # [변경] enable_background_map이 True일 때, 그래프의 모든 엣지를 회색으로 표시
        if self.enable_background_map and self.graph is not None:
            for edge in self.graph.edges():
                x0, y0 = edge[0]
                x1, y1 = edge[1]
                plt.plot([x0, x1], [y0, y1], color='gray', linestyle='-', alpha=0.5, linewidth=1)

        if path:
            x_path = [p[0] for p in path]
            y_path = [p[1] for p in path]
            # Approach Path 또는 단일 경로 표시
            plt.plot(x_path, y_path, 'ro-', label='Approach Path' if patrol_path else 'Path')

        # 현재 위치 (파란색)
        if self.current_pos:
            plt.scatter(self.current_pos[0], self.current_pos[1],
                        c='blue', label='Current Position', s=100)

        # 단일 목표 (Navigate/Homing 시 초록색)
        if self.goal_pos and not multi_goal:
            plt.scatter(self.goal_pos[0], self.goal_pos[1],
                        c='green', label='Goal Position', s=100)

        # 다중 목표 (Patrol 시 request.goals 목록 표시)
        if multi_goal and goals:
            for i, g in enumerate(goals):
                label_str = 'Goals' if i == 0 else None
                plt.scatter(g.x, g.y,
                            marker='*', c='blue', s=200,
                            label=label_str)
                plt.text(g.x, g.y, f'{i+1}', color='black', fontsize=12)

        # Global Patrol Path 추가 시 (녹색 점선)
        if patrol_path:
            x_patrol = [p[0] for p in patrol_path]
            y_patrol = [p[1] for p in patrol_path]
            plt.plot(x_patrol, y_patrol, 'go--', label='Global Patrol Path')

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
