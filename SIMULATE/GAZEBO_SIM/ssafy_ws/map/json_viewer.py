import os
import json
import networkx as nx
import matplotlib.pyplot as plt

# 현재 코드가 있는 디렉토리에서 JSON 파일 검색
current_directory = os.path.dirname(os.path.abspath(__file__))

# 노드 레이블 표시 여부
Labelset = True

def load_json_files(directory):
    """현재 디렉토리의 모든 JSON 파일을 로드하여 그래프 목록 반환"""
    graphs = []

    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            print(f"Loading: {file_path}")
            
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
                    graphs.append((filename, data))
            except Exception as e:
                print(f"Error loading {filename}: {e}")

    return graphs

def visualize_graphs(graph_data_list):
    """로드된 JSON 데이터를 시각화"""
    for file_name, data in graph_data_list:
        G = nx.Graph() if not data.get("directed", False) else nx.DiGraph()

        # 노드 추가
        for node in data["nodes"]:
            node_id = tuple(node["id"])
            G.add_node(node_id)

        # 엣지 추가
        for link in data["links"]:
            source = tuple(link["source"])
            target = tuple(link["target"])
            G.add_edge(source, target)

        plt.figure(figsize=(8, 6))
        plt.title(f"Graph Visualization - {file_name}")

        # 노드 위치를 좌표 값 그대로 사용
        pos = {node: node for node in G.nodes()}
        nx.draw(G, pos, node_size=50, node_color="blue", edge_color="gray", with_labels=Labelset, font_size=8)

        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.grid()
        plt.show()

def main():
    json_graphs = load_json_files(current_directory)

    if not json_graphs:
        print("No JSON files found in the current directory.")
        return

    visualize_graphs(json_graphs)

if __name__ == "__main__":
    main()
