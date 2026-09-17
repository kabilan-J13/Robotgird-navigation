"""
test_search.py

Run with:  python -m pytest tests/
or:        python -m unittest discover tests

Covers:
- grid loading from file
- BFS finds a path and it is valid (each step moves to an adjacent free cell)
- A* finds a path of the same length as BFS (both are optimal on unit-cost grids)
- unreachable goal is correctly reported as no path
- A* expands no more nodes than BFS on the same grid (sanity check that the
  heuristic is actually helping, not hurting)
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from grid import Grid  # noqa: E402
from search import breadth_first_search, a_star_search  # noqa: E402

FIXTURES = os.path.dirname(__file__)


def load(name):
    return Grid.from_file(os.path.join(FIXTURES, name))


def is_valid_path(grid, path):
    if path[0] != grid.start or path[-1] != grid.goal:
        return False
    for cell in path:
        if cell in grid.obstacles or not grid.in_bounds(cell):
            return False
    for a, b in zip(path, path[1:]):
        dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
        if (dr, dc) not in [(0, 1), (1, 0)]:
            return False  # not a single 4-connected step
    return True


class TestGridLoading(unittest.TestCase):
    def test_loads_dimensions_and_markers(self):
        grid = load("sample_grid_1.txt")
        self.assertEqual(grid.rows, 5)
        self.assertEqual(grid.cols, 5)
        self.assertEqual(grid.start, (0, 0))
        self.assertEqual(grid.goal, (4, 4))
        self.assertIn((0, 3), grid.obstacles)


class TestBFS(unittest.TestCase):
    def test_finds_valid_path(self):
        grid = load("sample_grid_1.txt")
        result = breadth_first_search(grid)
        self.assertIsNotNone(result.path)
        self.assertTrue(is_valid_path(grid, result.path))

    def test_larger_grid(self):
        grid = load("sample_grid_2.txt")
        result = breadth_first_search(grid)
        self.assertIsNotNone(result.path)
        self.assertTrue(is_valid_path(grid, result.path))

    def test_reports_no_path_when_unreachable(self):
        grid = load("sample_grid_3_unreachable.txt")
        result = breadth_first_search(grid)
        self.assertIsNone(result.path)


class TestAStar(unittest.TestCase):
    def test_finds_valid_path(self):
        grid = load("sample_grid_1.txt")
        result = a_star_search(grid)
        self.assertIsNotNone(result.path)
        self.assertTrue(is_valid_path(grid, result.path))

    def test_reports_no_path_when_unreachable(self):
        grid = load("sample_grid_3_unreachable.txt")
        result = a_star_search(grid)
        self.assertIsNone(result.path)

    def test_matches_bfs_optimal_length(self):
        for fixture in ["sample_grid_1.txt", "sample_grid_2.txt"]:
            grid = load(fixture)
            bfs_result = breadth_first_search(grid)
            astar_result = a_star_search(grid)
            self.assertEqual(bfs_result.path_length, astar_result.path_length,
                              f"A* and BFS disagree on optimal length for {fixture}")

    def test_expands_no_more_nodes_than_bfs(self):
        for fixture in ["sample_grid_1.txt", "sample_grid_2.txt"]:
            grid = load(fixture)
            bfs_result = breadth_first_search(grid)
            astar_result = a_star_search(grid)
            self.assertLessEqual(astar_result.nodes_expanded, bfs_result.nodes_expanded)


if __name__ == "__main__":
    unittest.main()
