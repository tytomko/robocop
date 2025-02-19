import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random
import math
from scipy.interpolate import splprep, splev

# -----------------------------------------------------------------------------
# 노드 클래스
# -----------------------------------------------------------------------------
class Node:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

# -----------------------------------------------------------------------------
# Cubic Spline 보간 함수
# -----------------------------------------------------------------------------
def smooth_path(path, num_points: int = 200, smoothing_factor: float = 0.5):
    """
    주어진 경로(path)를 cubic spline을 사용하여 부드럽게 보간합니다.
    
    Args:
        path: [[x1, y1], [x2, y2], ...] 형태의 경로 리스트
        num_points: 보간 후 생성할 점의 수
        smoothing_factor: splprep의 s 파라미터 (값이 클수록 더 부드럽게)
    
    Returns:
        보간된 경로를 (N,2) 배열 형태로 반환
    """
    path = np.array(path)
    if len(path) < 3:
        return path
    tck, u = splprep([path[:, 0], path[:, 1]], s=smoothing_factor)
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine = splev(u_fine, tck)
    return np.vstack((x_fine, y_fine)).T

# -----------------------------------------------------------------------------
# Bidirectional RRT* 알고리즘 클래스
# -----------------------------------------------------------------------------
class BidirectionalRRTStar:
    def __init__(self, start, goal, bounds, obstacles, max_iter=1000, step_size=0.5, search_radius=1.2, connection_threshold=0.5):
        """
        Args:
            start, goal: (x, y) 튜플
            bounds: (xmin, xmax, ymin, ymax) 튜플, 샘플링 영역
            obstacles: [(xmin, xmax, ymin, ymax), ...] 형식의 장애물 리스트
            max_iter: 최대 반복 횟수
            step_size: 한 번 확장할 때 이동할 거리
            search_radius: 확장 시 근접 노드 탐색 반경 (여기서는 단순 확장에 사용)
            connection_threshold: 두 트리 사이를 연결할 최대 거리
        """
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.bounds = bounds
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.connection_threshold = connection_threshold

        # 두 트리 초기화: 하나는 시작점에서, 하나는 목표점에서 확장
        self.nodes_start = [self.start]
        self.nodes_goal = [self.goal]
        self.path_found = False
        self.final_path = None

    def is_in_obstacle(self, x: float, y: float) -> bool:
        for (oxmin, oxmax, oymin, oymax) in self.obstacles:
            if oxmin <= x <= oxmax and oymin <= y <= oymax:
                return True
        return False

    def get_random_node(self) -> Node:
        while True:
            x = random.uniform(self.bounds[0], self.bounds[1])
            y = random.uniform(self.bounds[2], self.bounds[3])
            if not self.is_in_obstacle(x, y):
                return Node(x, y)

    def get_nearest_node(self, tree: list, random_node: Node) -> Node:
        return min(tree, key=lambda node: math.hypot(random_node.x - node.x, random_node.y - node.y))

    def steer(self, from_node: Node, to_node: Node) -> Node:
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        if distance <= self.step_size:
            new_node = to_node
        else:
            theta = math.atan2(dy, dx)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            new_node = Node(new_x, new_y)
        if self.is_in_obstacle(new_node.x, new_node.y):
            return None
        return new_node

    def is_collision_free(self, node1: Node, node2: Node) -> bool:
        resolution = 0.05
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        distance = math.hypot(dx, dy)
        steps = int(distance / resolution)
        for i in range(steps + 1):
            t = i / steps if steps != 0 else 0
            x = node1.x + t * dx
            y = node1.y + t * dy
            if self.is_in_obstacle(x, y):
                return False
        return True

    def get_path(self, node: Node, tree: list) -> list:
        path = []
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        return path[::-1]

    def extend_tree(self, tree: list, random_node: Node) -> Node:
        nearest = self.get_nearest_node(tree, random_node)
        new_node = self.steer(nearest, random_node)
        if new_node is not None and self.is_collision_free(nearest, new_node):
            new_node.cost = nearest.cost + math.hypot(new_node.x - nearest.x, new_node.y - nearest.y)
            new_node.parent = nearest
            tree.append(new_node)
            return new_node
        return None

    def plan_iterative(self):
        for iter_count in range(self.max_iter):
            # 번갈아 가며 두 트리 중 하나를 확장
            if iter_count % 2 == 0:
                tree_from = self.nodes_start
                tree_to = self.nodes_goal
            else:
                tree_from = self.nodes_goal
                tree_to = self.nodes_start

            rnd = self.get_random_node()
            new_node = self.extend_tree(tree_from, rnd)
            if new_node is None:
                yield iter_count
                continue

            # 반대 트리와 연결 시도
            nearest_in_other = self.get_nearest_node(tree_to, new_node)
            if self.is_collision_free(new_node, nearest_in_other):
                dist = math.hypot(new_node.x - nearest_in_other.x, new_node.y - nearest_in_other.y)
                if dist <= self.connection_threshold:
                    # 연결 성공!
                    self.path_found = True
                    # 두 경로 연결: 시작 트리의 경로와 목표 트리의 경로 (반대 트리의 경로 뒤집기)
                    if tree_from is self.nodes_start:
                        path_start = self.get_path(new_node, self.nodes_start)
                        path_goal = self.get_path(nearest_in_other, self.nodes_goal)
                    else:
                        path_start = self.get_path(nearest_in_other, self.nodes_start)
                        path_goal = self.get_path(new_node, self.nodes_goal)
                    self.final_path = path_start + path_goal[::-1]
                    yield iter_count
                    break
            yield iter_count
        yield None

# -----------------------------------------------------------------------------
# 도면 그리기 관련 함수
# -----------------------------------------------------------------------------
def draw_obstacles(ax, obstacles):
    for (oxmin, oxmax, oymin, oymax) in obstacles:
        rect = plt.Rectangle((oxmin, oymin), oxmax - oxmin, oymax - oymin, color='gray')
        ax.add_patch(rect)

def draw_tree(ax, nodes, color="-g"):
    for node in nodes:
        if node.parent is not None:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], color, linewidth=0.5)

# -----------------------------------------------------------------------------
# 애니메이션 함수
# -----------------------------------------------------------------------------
def animate_bidirectional_rrt():
    # 시작점, 목표점, 영역 및 미로 구성 (이전 미로 구성 유지)
    start = (1, 1)
    goal = (9, 9)
    bounds = (0, 10, 0, 10)
    obstacles = [
        (0, 10, 9.5, 10),
        (0, 10, 0, 0.5),
        (0, 0.5, 0, 10),
        (9.5, 10, 0, 10),
        (2, 2.5, 0.5, 7),
        (2, 2.5, 7.5, 9.5),
        (3.5, 4, 2, 10),
        (4.5, 5, 0, 4),
        (4.5, 5, 5, 7),
        (6, 6.5, 2, 8),
        (7.5, 9, 4, 4.5),
        (7.5, 9, 6, 6.5)
    ]

    bidir_rrt = BidirectionalRRTStar(start, goal, bounds, obstacles,
                                       max_iter=1000, step_size=0.5, search_radius=1.2, connection_threshold=0.5)
    
    fig, ax = plt.subplots()
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Bidirectional RRT* Maze Escape with Spline Smoothing")
    ax.grid(False)
    draw_obstacles(ax, obstacles)

    gen = bidir_rrt.plan_iterative()

    def update(frame):
        ax.cla()
        ax.set_xlim(bounds[0], bounds[1])
        ax.set_ylim(bounds[2], bounds[3])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("Bidirectional RRT* Maze Escape with Spline Smoothing")
        ax.grid(False)
        draw_obstacles(ax, obstacles)
        ax.plot(start[0], start[1], "bs", markersize=8, label="Start")
        ax.plot(goal[0], goal[1], "gs", markersize=8, label="Goal")
        draw_tree(ax, bidir_rrt.nodes_start, "-g")
        draw_tree(ax, bidir_rrt.nodes_goal, "-b")
        
        # 경로가 연결되었으면 최종 경로 추출 및 cubic spline 보간
        if bidir_rrt.path_found and bidir_rrt.final_path is not None:
            smooth_arr = smooth_path(bidir_rrt.final_path, num_points=200, smoothing_factor=0.5)
            ax.plot(smooth_arr[:,0], smooth_arr[:,1], "-r", linewidth=2, label="Smoothed Path")
        
        ax.legend()
        return []

    ani = animation.FuncAnimation(fig, update, frames=gen, interval=10, repeat=False)
    plt.show()

# -----------------------------------------------------------------------------
# 메인 실행
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    animate_bidirectional_rrt()
