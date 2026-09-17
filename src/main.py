"""
main.py

Command-line entry point for Robot Grid Navigation.

Usage:
    python src/main.py --grid tests/sample_grid_1.txt --algo astar
    python src/main.py --grid tests/sample_grid_2.txt --algo bfs --plot docs/path_plot.png
"""

import argparse
import sys
import time

from grid import Grid
from search import ALGORITHMS
from visualize import plot_grid


def main():
    parser = argparse.ArgumentParser(description="Robot Grid Navigation via search.")
    parser.add_argument("--grid", required=True, help="Path to a grid text file.")
    parser.add_argument("--algo", choices=ALGORITHMS.keys(), default="astar",
                         help="Search algorithm to use (default: astar).")
    parser.add_argument("--plot", default=None, help="Optional path to save a PNG visualization.")
    args = parser.parse_args()

    grid = Grid.from_file(args.grid)
    algorithm = ALGORITHMS[args.algo]

    start_time = time.perf_counter()
    result = algorithm(grid)
    elapsed_ms = (time.perf_counter() - start_time) * 1000

    print(f"Algorithm       : {args.algo}")
    print(f"Grid size       : {grid.rows} x {grid.cols}")
    print(f"Start -> Goal   : {grid.start} -> {grid.goal}")
    print(f"Nodes expanded  : {result.nodes_expanded}")
    print(f"Max frontier    : {result.frontier_max_size}")
    print(f"Time            : {elapsed_ms:.3f} ms")

    if result.path is None:
        print("Result          : NO PATH FOUND")
        sys.exit(1)

    print(f"Path length     : {result.path_length} steps ({len(result.path)} cells)")
    print()
    print(grid.render(result.path))

    if args.plot:
        title = f"{args.algo.upper()} - {result.path_length} steps, {result.nodes_expanded} nodes expanded"
        plot_grid(grid, result.path, title, args.plot)
        print(f"\nSaved visualization to {args.plot}")


if __name__ == "__main__":
    main()
