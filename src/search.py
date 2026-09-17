"""
search.py

Frames robot grid navigation as a classic search problem and solves it.

Search problem formulation
---------------------------
State space   : every free (row, col) cell on the grid
Initial state : grid.start
Goal test     : state == grid.goal
Actions       : move Up / Down / Left / Right into an in-bounds, non-obstacle cell
Step cost     : 1 per move (uniform cost)
Path          : sequence of states from start to goal

Two algorithms are provided:

- breadth_first_search : uninformed search, optimal on uniform-cost grids,
  explores in "rings" outward from the start.
- a_star_search        : informed search using the Manhattan distance heuristic,
  which is admissible and consistent on a 4-connected grid with unit costs,
  so A* is also guaranteed optimal here but typically expands far fewer nodes.
"""

import heapq
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from grid import Coord, Grid


@dataclass
class SearchResult:
    path: Optional[List[Coord]]      # None if no path exists
    nodes_expanded: int
    frontier_max_size: int

    @property
    def path_length(self) -> Optional[int]:
        return None if self.path is None else len(self.path) - 1  # steps, not cells


def _reconstruct_path(came_from: Dict[Coord, Coord], start: Coord, goal: Coord) -> List[Coord]:
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def breadth_first_search(grid: Grid) -> SearchResult:
    """Uninformed search. Guarantees the shortest path on an unweighted grid."""
    frontier = deque([grid.start])
    came_from: Dict[Coord, Coord] = {}
    visited = {grid.start}
    nodes_expanded = 0
    frontier_max_size = 1

    while frontier:
        frontier_max_size = max(frontier_max_size, len(frontier))
        current = frontier.popleft()
        nodes_expanded += 1

        if current == grid.goal:
            return SearchResult(
                path=_reconstruct_path(came_from, grid.start, grid.goal),
                nodes_expanded=nodes_expanded,
                frontier_max_size=frontier_max_size,
            )

        for neighbor in grid.neighbors(current):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                frontier.append(neighbor)

    return SearchResult(path=None, nodes_expanded=nodes_expanded, frontier_max_size=frontier_max_size)


def manhattan_distance(a: Coord, b: Coord) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(grid: Grid) -> SearchResult:
    """Informed search using f(n) = g(n) + h(n), h = Manhattan distance."""
    start, goal = grid.start, grid.goal

    counter = 0  # tie-breaker so heap never compares Coord tuples directly
    frontier: List[Tuple[int, int, Coord]] = [(manhattan_distance(start, goal), counter, start)]
    came_from: Dict[Coord, Coord] = {}
    g_score: Dict[Coord, int] = {start: 0}
    visited = set()
    nodes_expanded = 0
    frontier_max_size = 1

    while frontier:
        frontier_max_size = max(frontier_max_size, len(frontier))
        _, _, current = heapq.heappop(frontier)

        if current in visited:
            continue
        visited.add(current)
        nodes_expanded += 1

        if current == goal:
            return SearchResult(
                path=_reconstruct_path(came_from, start, goal),
                nodes_expanded=nodes_expanded,
                frontier_max_size=frontier_max_size,
            )

        for neighbor in grid.neighbors(current):
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan_distance(neighbor, goal)
                counter += 1
                heapq.heappush(frontier, (f_score, counter, neighbor))

    return SearchResult(path=None, nodes_expanded=nodes_expanded, frontier_max_size=frontier_max_size)


ALGORITHMS = {
    "bfs": breadth_first_search,
    "astar": a_star_search,
}
