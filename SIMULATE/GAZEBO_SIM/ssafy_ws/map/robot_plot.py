import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor  # 멀티스레드 실행자 추가
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import json
import networkx as nx
import os
import math
from functools import partial
import threading

class MultiRobotPathVisualizer(Node):
    def __init__(self, num_robots):
        super().__init__('multi_robot_path_visualizer')
        self.num_robots = num_robots
        self.robots = {}

        # ─── 그래프 로드 (배경 지도) ─────────────────────────────
        self.load_graph_data("global_map.json")

        # ─── Matplotlib 설정 (인터랙티브 모드 비활성화) ─────────────────
        # GUI 관련 코드는 메인 스레드에서 처리하기 위해 interactive mode를 사용하지 않습니다.
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

        # 배경 지도(그래프) 표시
        if self.G is not None:
            self.pos = {node: node for node in self.G.nodes()}
            nx.draw(
                self.G, 
                pos=self.pos, 
                ax=self.ax, 
                node_size=20, 
                node_color='lightgray',  # 파란색 대신 연한 회색 사용
                edge_color='gray', 
                with_labels=False
            )
            # 배경 지도에 맞춰 축 범위를 고정 (예시: 그래프의 노드 좌표의 최소/최대값으로 설정)
            all_x = [coord[0] for coord in self.G.nodes()]
            all_y = [coord[1] for coord in self.G.nodes()]
            self.ax.set_xlim(min(all_x) - 5, max(all_x) + 5)
            self.ax.set_ylim(min(all_y) - 5, max(all_y) + 5)
        else:
            self.get_logger().error("Graph 데이터가 로드되지 않았습니다.")

        cmap = plt.get_cmap('tab10')

        # ─── 각 로봇별 구독자 및 플롯 요소 생성 ───────────────
        for robot_id in range(1, self.num_robots + 1):
            color = cmap((robot_id - 1) % 10)
            self.robots[robot_id] = {
                "current_x": None,
                "current_y": None,
                "current_heading": None,
                "target_x": None,
                "target_y": None,
                "target_heading": None,
                "current_pos_plot": self.ax.plot([], [], marker='o', color=color, markersize=10, label=f'Robot {robot_id} Position')[0],
                "target_pos_plot": self.ax.plot([], [], marker='x', color=color, markersize=10, label=f'Robot {robot_id} Target')[0],
                "global_path_plot": self.ax.plot([], [], linestyle='-', color=color, linewidth=2, label=f'Robot {robot_id} Global Path')[0],
                "approach_path_plot": self.ax.plot([], [], linestyle='--', color=color, linewidth=2, label=f'Robot {robot_id} Approach Path')[0],
                "heading_line": self.ax.plot([], [], linestyle='-', color=color, linewidth=2, label=f'Robot {robot_id} Heading')[0],
                "target_heading_line": self.ax.plot([], [], linestyle='-', color=color, linewidth=2, label=f'Robot {robot_id} Target Heading')[0],
            }

            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/utm_pose',
                partial(self.pose_callback, robot_id=robot_id),
                10
            )
            self.create_subscription(
                NavPath,
                f'/robot_{robot_id}/global_path',
                partial(self.global_path_callback, robot_id=robot_id),
                10
            )
            self.create_subscription(
                NavPath,
                f'/robot_{robot_id}/approach_path',
                partial(self.approach_path_callback, robot_id=robot_id),
                10
            )
            self.create_subscription(
                Float32,
                f'/robot_{robot_id}/heading',
                partial(self.heading_callback, robot_id=robot_id),
                10
            )
            self.create_subscription(
                Point,
                f'/robot_{robot_id}/target_point',
                partial(self.target_pose_callback, robot_id=robot_id),
                10
            )
            self.create_subscription(
                Float32,
                f'/robot_{robot_id}/target_heading',
                partial(self.target_heading_callback, robot_id=robot_id),
                10
            )

        # 범례 제거(필요하면 주석 해제)
        # self.ax.legend()

    def load_graph_data(self, file_name):
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
                    node_id = tuple(node["id"][:2])
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

    def pose_callback(self, msg: PoseStamped, robot_id):
        self.robots[robot_id]["current_x"] = msg.pose.position.x
        self.robots[robot_id]["current_y"] = msg.pose.position.y
        self.update_current_position_plot(robot_id)

    def heading_callback(self, msg: Float32, robot_id):
        self.robots[robot_id]["current_heading"] = msg.data
        self.update_current_heading_arrow(robot_id)

    def target_pose_callback(self, msg: Point, robot_id):
        self.robots[robot_id]["target_x"] = msg.x
        self.robots[robot_id]["target_y"] = msg.y
        self.update_target_position_plot(robot_id)
        self.update_target_heading_arrow(robot_id)

    def target_heading_callback(self, msg: Float32, robot_id):
        self.robots[robot_id]["target_heading"] = msg.data
        self.update_target_heading_arrow(robot_id)

    def global_path_callback(self, msg: NavPath, robot_id):
        x_vals = [pose.pose.position.x for pose in msg.poses]
        y_vals = [pose.pose.position.y for pose in msg.poses]
        self.robots[robot_id]["global_path_plot"].set_xdata(x_vals)
        self.robots[robot_id]["global_path_plot"].set_ydata(y_vals)
        # GUI 갱신은 메인 스레드의 타이머에서 처리합니다.

    def approach_path_callback(self, msg: NavPath, robot_id):
        x_vals = [pose.pose.position.x for pose in msg.poses]
        y_vals = [pose.pose.position.y for pose in msg.poses]
        self.robots[robot_id]["approach_path_plot"].set_xdata(x_vals)
        self.robots[robot_id]["approach_path_plot"].set_ydata(y_vals)
        # GUI 갱신은 메인 스레드의 타이머에서 처리합니다.

    def update_current_position_plot(self, robot_id):
        current_x = self.robots[robot_id]["current_x"]
        current_y = self.robots[robot_id]["current_y"]
        if current_x is not None and current_y is not None:
            self.robots[robot_id]["current_pos_plot"].set_xdata([current_x])
            self.robots[robot_id]["current_pos_plot"].set_ydata([current_y])
            # plt.draw(), plt.pause() 제거
            self.update_current_heading_arrow(robot_id)

    def update_current_heading_arrow(self, robot_id):
        current_x = self.robots[robot_id]["current_x"]
        current_y = self.robots[robot_id]["current_y"]
        current_heading = self.robots[robot_id]["current_heading"]
        if current_x is not None and current_y is not None and current_heading is not None:
            arrow_length = 2.0
            dx = arrow_length * math.cos(current_heading)
            dy = arrow_length * math.sin(current_heading)
            x_vals = [current_x, current_x + dx]
            y_vals = [current_y, current_y + dy]
            self.robots[robot_id]["heading_line"].set_xdata(x_vals)
            self.robots[robot_id]["heading_line"].set_ydata(y_vals)
            # plt.draw(), plt.pause() 제거

    def update_target_position_plot(self, robot_id):
        target_x = self.robots[robot_id]["target_x"]
        target_y = self.robots[robot_id]["target_y"]
        if target_x is not None and target_y is not None:
            self.robots[robot_id]["target_pos_plot"].set_xdata([target_x])
            self.robots[robot_id]["target_pos_plot"].set_ydata([target_y])
            # plt.draw(), plt.pause() 제거

    def update_target_heading_arrow(self, robot_id):
        target_x = self.robots[robot_id]["target_x"]
        target_y = self.robots[robot_id]["target_y"]
        target_heading = self.robots[robot_id]["target_heading"]
        if target_x is not None and target_y is not None and target_heading is not None:
            arrow_length = 2.0
            dx = arrow_length * math.cos(target_heading)
            dy = arrow_length * math.sin(target_heading)
            x_vals = [target_x, target_x + dx]
            y_vals = [target_y, target_y + dy]
            self.robots[robot_id]["target_heading_line"].set_xdata(x_vals)
            self.robots[robot_id]["target_heading_line"].set_ydata(y_vals)
            # plt.draw(), plt.pause() 제거

def main(args=None):
    num_robots = int(input("로봇 대수를 입력하세요: "))
    rclpy.init(args=args)
    multi_robot_visualizer = MultiRobotPathVisualizer(num_robots)

    # ROS 실행은 별도 스레드에서 진행
    executor = MultiThreadedExecutor()
    executor.add_node(multi_robot_visualizer)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Matplotlib GUI는 메인 스레드에서 실행
    # 타이머를 등록하여 주기적으로 캔버스 갱신 (100ms 간격)
    timer = multi_robot_visualizer.fig.canvas.new_timer(interval=100)
    timer.add_callback(lambda: multi_robot_visualizer.fig.canvas.draw_idle())
    timer.start()

    try:
        plt.show()  # 메인 스레드에서 GUI 이벤트 루프 실행
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        multi_robot_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
