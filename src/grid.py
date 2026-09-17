"""
grid.py

Defines the Grid environment the robot operates in:
- free cells
- obstacle cells
- a start cell and a goal cell

Grid files are plain text, e.g.:

    S . . # .
    . # . . .
    . # . # .
    . . . # G

Where:
    S -> start
    G -> goal
    # -> obstacle
    . -> free cell
"""

from dataclasses import dataclass
from typing import List, Set, Tuple

Coord = Tuple[int, int]  # (row, col)


@dataclass
class Grid:
    rows: int
    cols: int
    obstacles: Set[Coord]
    start: Coord
    goal: Coord

    @classmethod
    def from_file(cls, path: str) -> "Grid":
        with open(path, "r") as f:
            lines = [line.rstrip("\n") for line in f if line.strip() != ""]

        cells = [line.split() for line in lines]
        rows = len(cells)
        cols = len(cells[0])

        obstacles: Set[Coord] = set()
        start = None
        goal = None

        for r, row in enumerate(cells):
            if len(row) != cols:
                raise ValueError(
                    f"Row {r} has {len(row)} columns, expected {cols}. "
                    "Make sure every row has the same number of space-separated symbols."
                )
            for c, symbol in enumerate(row):
                if symbol == "#":
                    obstacles.add((r, c))
                elif symbol == "S":
                    start = (r, c)
                elif symbol == "G":
                    goal = (r, c)
                elif symbol == ".":
                    pass
                else:
                    raise ValueError(f"Unknown symbol '{symbol}' at ({r}, {c})")

        if start is None or goal is None:
            raise ValueError("Grid file must contain exactly one 'S' and one 'G'.")

        return cls(rows=rows, cols=cols, obstacles=obstacles, start=start, goal=goal)

    def in_bounds(self, coord: Coord) -> bool:
        r, c = coord
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_free(self, coord: Coord) -> bool:
        return coord not in self.obstacles

    def neighbors(self, coord: Coord) -> List[Coord]:
        """4-connected movement: up, down, left, right."""
        r, c = coord
        candidates = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
        return [n for n in candidates if self.in_bounds(n) and self.is_free(n)]

    def render(self, path: List[Coord] = None) -> str:
        """Return an ASCII rendering of the grid, optionally with a path drawn on it."""
        path_set = set(path or [])
        lines = []
        for r in range(self.rows):
            row_symbols = []
            for c in range(self.cols):
                coord = (r, c)
                if coord == self.start:
                    row_symbols.append("S")
                elif coord == self.goal:
                    row_symbols.append("G")
                elif coord in self.obstacles:
                    row_symbols.append("#")
                elif coord in path_set:
                    row_symbols.append("*")
                else:
                    row_symbols.append(".")
            lines.append(" ".join(row_symbols))
        return "\n".join(lines)
