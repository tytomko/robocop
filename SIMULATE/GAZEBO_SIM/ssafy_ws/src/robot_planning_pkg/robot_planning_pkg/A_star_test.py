import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

global_path = 'map/global_path.json'
temp_goal = (                304412.92687916575,
                3892844.1526765698)
class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')

        # 파라미터 선언 및 가져오기
        self.declare_parameter('robot_number', 1)
        robot_number = self.get_parameter('robot_number').value
        pose_topic = f"/robot_{robot_number}/utm_pose"

        # PoseStamped 구독
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            10
        )

        # JSON 파일 로드
        self.graph = self.load_graph(global_path)

        # 목적지 설정 (임시 값)
        self.goal_pos = temp_goal

        self.current_pos = None
        self.get_logger().info(f"Subscribed to {pose_topic}")
    
    def pose_callback(self, msg):
        """현재 위치 콜백"""
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Received current position: {self.current_pos}")

        if self.current_pos:
            path = self.find_shortest_path(self.current_pos, self.goal_pos)
            if path:
                self.visualize_path(path)

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

    def visualize_path(self, path):
        """
        Visualizes the path using Matplotlib.
        This function visualizes the global path and the shortest path on a 2D plot.
        It also marks the current position and the goal position.
        Args:
            path (list of tuples): The shortest path to be visualized, where each tuple represents a node (x, y).
        Notes:
            - 'ro-' in plt.plot(x_path, y_path, 'ro-', label='Shortest Path') means:
                'r': red color for the line and markers
                'o': circle markers
                '-': solid line style
            - Other marker styles include:
                'b' : blue color
                'g' : green color
                'x' : x markers
                '--': dashed line style
                '-.' : dash-dot line style
                ':' : dotted line style
        """
        """Matplotlib을 사용한 경로 시각화"""
        plt.figure(figsize=(10, 8))

        # 글로벌 경로 시각화
        for edge in self.graph.edges:
            x_values = [edge[0][0], edge[1][0]]
            y_values = [edge[0][1], edge[1][1]]
            plt.plot(x_values, y_values, 'gray', alpha=0.5)

        # 최단 경로 시각화
        x_path = [node[0] for node in path]
        y_path = [node[1] for node in path]
        
        plt.plot(x_path, y_path, 'ro-', label='Shortest Path', alpha=0.8)

        # 현재 위치 및 목적지 표시
        
        plt.scatter(self.current_pos[0], self.current_pos[1], c='blue', label='Current Position', s=100)
        plt.scatter(self.goal_pos[0], self.goal_pos[1], c='green', label='Goal Position', s=100)

        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Global Path Visualization')
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
