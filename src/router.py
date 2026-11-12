"""
Drone Route Planner & Spatial Graph Search
Implements A* heuristic search on coordinate-mapped delivery networks.
"""

import heapq
from typing import Dict, List, Optional, Tuple

# Default topological delivery graph with edge weights (distances / energy cost)
DEFAULT_GRAPH: Dict[str, Dict[str, float]] = {
    'W': {'A': 200, 'B': 190, 'C': 500},
    'A': {'C': 320},
    'B': {'D': 260},
    'D': {'C': 240},
    'C': {'E': 460},
    'E': {}
}

# 2D Spatial coordinates for Euclidean heuristic calculation (X, Y)
DEFAULT_COORDINATES: Dict[str, Tuple[int, int]] = {
    'W': (50, 50),
    'A': (150, 50),
    'B': (50, 150),
    'C': (250, 150),
    'D': (150, 250),
    'E': (250, 250)
}


class DroneRoutePlanner:
    """Autonomous drone pathfinding engine utilizing admissible Euclidean heuristics."""

    def __init__(
        self,
        graph: Optional[Dict[str, Dict[str, float]]] = None,
        coordinates: Optional[Dict[str, Tuple[int, int]]] = None
    ):
        self.graph = graph if graph is not None else DEFAULT_GRAPH
        self.coordinates = coordinates if coordinates is not None else DEFAULT_COORDINATES

    def heuristic(self, node: str, goal: str) -> float:
        """Calculate the admissible Euclidean distance between two nodes."""
        if node not in self.coordinates or goal not in self.coordinates:
            return 0.0
        x1, y1 = self.coordinates[node]
        x2, y2 = self.coordinates[goal]
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def find_shortest_path(self, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
        """
        Executes A* algorithm to find the optimal delivery trajectory.
        Returns (path_list, total_cost).
        """
        if start not in self.graph or goal not in self.graph:
            return None, 0.0

        if start == goal:
            return [start], 0.0

        g_score: Dict[str, float] = {start: 0.0}
        parent: Dict[str, Optional[str]] = {start: None}

        # Min-heap priority queue storing (f_score, node)
        open_set: List[Tuple[float, str]] = []
        heapq.heappush(open_set, (self.heuristic(start, goal), start))

        visited = set()

        while open_set:
            current_f, current_node = heapq.heappop(open_set)

            if current_node == goal:
                # Reconstruct path
                path = []
                curr = current_node
                while curr is not None:
                    path.append(curr)
                    curr = parent[curr]
                path.reverse()
                total_cost = g_score[goal]
                return path, total_cost

            visited.add(current_node)

            for neighbor, weight in self.graph.get(current_node, {}).items():
                tentative_g = g_score[current_node] + weight

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    parent[neighbor] = current_node
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None, 0.0

    def calculate_path_cost(self, path: List[str]) -> float:
        """Calculate exact cumulative edge cost for a given sequence of waypoints."""
        if not path or len(path) < 2:
            return 0.0
        total = 0.0
        for i in range(len(path) - 1):
            src, dst = path[i], path[i + 1]
            total += self.graph.get(src, {}).get(dst, 0.0)
        return total

