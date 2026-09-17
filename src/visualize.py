"""
visualize.py

Renders the grid, obstacles, start/goal and the discovered path to a PNG
using matplotlib. Used both for docs/screenshots and for quick debugging.
"""

from typing import List, Optional

import matplotlib

matplotlib.use("Agg")  # headless backend, no display needed
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from grid import Coord, Grid


def plot_grid(grid: Grid, path: Optional[List[Coord]], title: str, out_path: str) -> None:
    fig, ax = plt.subplots(figsize=(grid.cols * 0.6 + 1, grid.rows * 0.6 + 1))

    # Draw cell grid
    for r in range(grid.rows):
        for c in range(grid.cols):
            face = "white"
            if (r, c) in grid.obstacles:
                face = "#333333"
            rect = patches.Rectangle((c, grid.rows - 1 - r), 1, 1, edgecolor="#bbbbbb", facecolor=face)
            ax.add_patch(rect)

    # Draw path
    if path:
        xs = [c + 0.5 for _, c in path]
        ys = [grid.rows - 1 - r + 0.5 for r, _ in path]
        ax.plot(xs, ys, color="#1f77b4", linewidth=3, marker="o", markersize=5, zorder=3)

    # Start / goal markers
    sr, sc = grid.start
    gr, gc = grid.goal
    ax.text(sc + 0.5, grid.rows - 1 - sr + 0.5, "S", ha="center", va="center",
             fontsize=14, fontweight="bold", color="green", zorder=4)
    ax.text(gc + 0.5, grid.rows - 1 - gr + 0.5, "G", ha="center", va="center",
             fontsize=14, fontweight="bold", color="red", zorder=4)

    ax.set_xlim(0, grid.cols)
    ax.set_ylim(0, grid.rows)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
