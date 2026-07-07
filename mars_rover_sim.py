from __future__ import annotations
import heapq
import random
from dataclasses import dataclass
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Set
from typing import Tuple

try:
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
except ImportError:
    plt = None
    ListedColormap = None


@dataclass(frozen=True, slots=True)
class Position:
    row: int
    col: int


class MarsMap:
    def __init__(self, rows: int, cols: int, obstacle_ratio: float, seed: int = 7) -> None:
        self.rows = rows
        self.cols = cols
        self.obstacles: Set[Position] = set()
        random.seed(seed)
        obstacle_count = int(rows * cols * obstacle_ratio)
        while len(self.obstacles) < obstacle_count:
            candidate = Position(random.randrange(rows), random.randrange(cols))
            if candidate not in (Position(0, 0), Position(rows - 1, cols - 1)):
                self.obstacles.add(candidate)

    def in_bounds(self, p: Position) -> bool:
        return 0 <= p.row < self.rows and 0 <= p.col < self.cols

    def passable(self, p: Position) -> bool:
        return p not in self.obstacles

    def neighbors(self, p: Position) -> Iterable[Position]:
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = Position(p.row + dr, p.col + dc)
            if self.in_bounds(nxt) and self.passable(nxt):
                yield nxt

    def render(self, rover: Position, goal: Position, path: Set[Position]) -> str:
        lines: List[str] = []
        for r in range(self.rows):
            row_cells: List[str] = []
            for c in range(self.cols):
                p = Position(r, c)
                if p == rover:
                    row_cells.append("R")
                elif p == goal:
                    row_cells.append("G")
                elif p in self.obstacles:
                    row_cells.append("#")
                elif p in path:
                    row_cells.append("*")
                else:
                    row_cells.append(".")
            lines.append(" ".join(row_cells))
        return "\n".join(lines)


class AStarPlanner:
    @staticmethod
    def heuristic(a: Position, b: Position) -> int:
        return abs(a.row - b.row) + abs(a.col - b.col)

    def find_path(self, world: MarsMap, start: Position, goal: Position) -> Optional[List[Position]]:
        open_heap: List[Tuple[int, int, Position]] = []
        heapq.heappush(open_heap, (0, 0, start))
        came_from: Dict[Position, Optional[Position]] = {start: None}
        g_cost: Dict[Position, int] = {start: 0}
        tie = 1

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current == goal:
                return self._reconstruct(came_from, goal)
            for nxt in world.neighbors(current):
                tentative_g = g_cost[current] + 1
                if nxt not in g_cost or tentative_g < g_cost[nxt]:
                    g_cost[nxt] = tentative_g
                    f_score = tentative_g + self.heuristic(nxt, goal)
                    heapq.heappush(open_heap, (f_score, tie, nxt))
                    tie += 1
                    came_from[nxt] = current

        return None

    def _reconstruct(self, came_from: Dict[Position, Optional[Position]], goal: Position) -> List[Position]:
        current: Optional[Position] = goal
        rev_path: List[Position] = []
        while current is not None:
            rev_path.append(current)
            current = came_from[current]
        return list(reversed(rev_path))


class Rover:
    def __init__(self, start: Position, goal: Position, planner: AStarPlanner) -> None:
        self.position = start
        self.goal = goal
        self.planner = planner
        self.travel_log: List[Position] = [start]

    def plan(self, world: MarsMap) -> Optional[List[Position]]:
        return self.planner.find_path(world, self.position, self.goal)

    def follow_path(self, path: List[Position]) -> None:
        for step in path[1:]:
            self.position = step
            self.travel_log.append(step)


class Simulation:
    def __init__(self, rows: int = 12, cols: int = 20, obstacle_ratio: float = 0.20) -> None:
        self.world = MarsMap(rows, cols, obstacle_ratio)
        start = Position(0, 0)
        goal = Position(rows - 1, cols - 1)
        self.rover = Rover(start, goal, AStarPlanner())

    def run(self) -> bool:
        path = self.rover.plan(self.world)
        if path is None:
            print("Mission result: FAILED (no valid path found)")
            return False
        self.rover.follow_path(path)
        print(self.world.render(self.rover.position, self.rover.goal, set(path)))
        print(f"\nMission result: SUCCESS in {len(path) - 1} moves")
        self._export_graphs(path)
        return True

    def _export_graphs(self, path: List[Position]) -> None:
        if plt is None or ListedColormap is None:
            print("Graphs skipped: install matplotlib to generate mission_graphs.png")
            return

        terrain: List[List[int]] = [[0 for _ in range(self.world.cols)] for _ in range(self.world.rows)]
        for obstacle in self.world.obstacles:
            terrain[obstacle.row][obstacle.col] = 1

        for step in path:
            terrain[step.row][step.col] = 2

        terrain[path[0].row][path[0].col] = 3
        terrain[path[-1].row][path[-1].col] = 4

        distances = [AStarPlanner.heuristic(step, self.rover.goal) for step in path]

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        cmap = ListedColormap(["#f5f1e6", "#2b2d42", "#ef8354", "#06d6a0", "#118ab2"])
        axes[0].imshow(terrain, cmap=cmap, interpolation="nearest")
        axes[0].set_title("Terrain and Planned Path")
        axes[0].set_xlabel("Column")
        axes[0].set_ylabel("Row")

        axes[1].plot(range(len(distances)), distances, marker="o", linewidth=2, color="#d62828")
        axes[1].set_title("Distance to Goal per Step")
        axes[1].set_xlabel("Step")
        axes[1].set_ylabel("Manhattan Distance")
        axes[1].grid(alpha=0.3)

        output_path = "mission_graphs.png"
        plt.tight_layout()
        fig.savefig(output_path, dpi=140)
        plt.close(fig)
        print(f"Graphs saved: {output_path}")


if __name__ == "__main__":
    sim = Simulation()
    sim.run()
