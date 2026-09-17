# Robot Grid Navigation

A robot finds its way to a destination on a 2D grid with obstacles, framed and solved as a
search problem.

## Problem

Given a grid with a start cell `S`, a goal cell `G`, and a set of impassable obstacle cells
`#`, find a shortest sequence of up/down/left/right moves from `S` to `G`, or correctly
report that no path exists.

## Approach

Robot movement is framed as a classic search problem:

| Component     | Definition                                                  |
|----------------|--------------------------------------------------------------|
| State space    | Every free `(row, col)` cell on the grid                    |
| Initial state  | The robot's start cell                                       |
| Goal test      | `current cell == goal cell`                                  |
| Actions        | Move Up / Down / Left / Right into an in-bounds, free cell    |
| Step cost      | 1 per move (uniform cost)                                    |

Two algorithms are implemented on top of this formulation:

- **Breadth-First Search** (`src/search.py::breadth_first_search`) — uninformed baseline,
  guaranteed optimal on a uniform-cost grid.
- **A\*** (`src/search.py::a_star_search`) — the chosen algorithm, using Manhattan distance
  as an admissible, consistent heuristic. Remains optimal, but expands fewer nodes than BFS
  by searching toward the goal instead of outward in every direction.

Full derivation and results are in [`docs/report.pdf`](docs/report.pdf).

## Project structure

```
robot-grid-navigation/
├── src/            # grid.py, search.py, visualize.py, main.py
├── tests/          # sample grids + unit tests (unittest)
├── docs/           # report.pdf, path visualizations
├── README.md
├── requirements.txt
├── .gitignore
└── LICENSE
```

## How to run

```bash
pip install -r requirements.txt

# Run A* on a sample grid and save a visualization
python src/main.py --grid tests/sample_grid_2.txt --algo astar --plot docs/astar_path.png

# Run BFS instead
python src/main.py --grid tests/sample_grid_1.txt --algo bfs --plot docs/bfs_path.png
```

Run the tests:

```bash
python -m unittest discover tests
```

## Grid file format

Plain text, space-separated symbols, one row per line:

```
S . . # .
. # . . .
. # . # .
. . . # .
# . . . G
```

- `S` — start (exactly one)
- `G` — goal (exactly one)
- `#` — obstacle
- `.` — free cell

## Sample I/O

Input: `tests/sample_grid_1.txt`

```
S . . # .
. # . . .
. # . # .
. . . # .
# . . . G
```

Command:

```bash
python src/main.py --grid tests/sample_grid_1.txt --algo astar
```

Output:

```
Algorithm       : astar
Grid size       : 5 x 5
Start -> Goal   : (0, 0) -> (4, 4)
Nodes expanded  : 19
Max frontier    : 4
Time            : 0.07 ms
Path length     : 8 steps (9 cells)

S . . # .
* # . . .
* # . # .
* * . # .
# * * * G
```

`tests/sample_grid_3_unreachable.txt` demonstrates the no-path case: the goal sits in a
region fully walled off from the start, and both algorithms correctly report
`NO PATH FOUND` (exit code 1).

## License

MIT — see [LICENSE](LICENSE).
