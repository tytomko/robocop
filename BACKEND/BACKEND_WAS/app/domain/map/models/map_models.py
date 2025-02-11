from pydantic import BaseModel
from typing import List, Tuple, Dict, Any

class Node(BaseModel):
    id: List[float]  # [x, y] 좌표값

class Link(BaseModel):
    cost: float
    source: List[float]  # 시작점 [x, y] 좌표값
    target: List[float]  # 끝점 [x, y] 좌표값

class PointsData(BaseModel):
    directed: bool
    multigraph: bool
    graph: Dict[str, Any]
    nodes: List[Node]
    links: List[Link]
    
    def get_coordinates(self) -> List[Tuple[float, float]]:
        """노드의 id 리스트를 (x, y) 좌표 튜플 리스트로 변환"""
        return [(node.id[0], node.id[1]) for node in self.nodes]
    
    def get_links(self) -> List[Dict[str, Any]]:
        """링크 정보를 딕셔너리 리스트로 변환"""
        return [
            {
                'cost': link.cost,
                'source': (link.source[0], link.source[1]),
                'target': (link.target[0], link.target[1])
            }
            for link in self.links
        ]

    class Config:
        from_attributes = True
