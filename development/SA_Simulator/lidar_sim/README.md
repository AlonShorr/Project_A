# Lidar Pygame Simulator

A grid-based simulator for developing and testing a probabilistic localization and autonomous navigation system. The robot carries five ToF (Time-of-Flight) distance sensors and uses a Bayes filter to estimate its position on the map, then plans a path to a goal using RRT.

---

## Running the Simulator

```bash
cd lidar_sim/
pip install -r requirements.txt
python3 lidar_pygame_sim.py
```

---

## Controls

| Key | Action |
|---|---|
| `W / A / S / D` | Move robot (up / left / down / right) |
| `Q / E` | Rotate robot counter-clockwise / clockwise (45° steps) |
| `1 – 5` | Toggle individual sensors on/off |
| `R` | Start / stop autonomous mode (localize → plan → navigate) |
| `B` | Toggle map builder mode (click cell edges to add/remove walls) |
| `P` | Toggle analysis mode (click cells to inspect their sensor data) |
| `[ / ]` | Decrease / increase auto-step delay |
| Left-click (map) | Teleport robot to that cell |
| Right-click (map) | Set navigation goal |

---

## Module Overview

```
lidar_sim/
│
├── config.py            # All constants (map size, sensor specs, colors, thresholds)
├── sim_robot.py         # Robot class (sensors, movement, ray-casting)
├── map_builder.py       # Map definition, wall editing, debug data dump
├── localization.py      # Bayes filter localization algorithm
├── motion_planning.py   # RRT path planning wrapper
├── rrt_planner.py       # Standalone RRT planner implementation
├── rrt_demo.py          # Console verification script for the RRT planner
├── lidar_pygame_sim.py  # Main entry point — pygame loop, rendering, and input
└── requirements.txt     # Python dependencies
```

---

## Module Descriptions

### `config.py`
Single source of truth for every constant in the project. Nothing here is computed at runtime — change a value here and it propagates everywhere.

Key groups:
- **Display** — window size, grid size, pixel dimensions
- **Physics/Sensor** — cell size in mm, sensor range, noise level, sensor angles
- **Wall indices** — `WALL_UP/DOWN/LEFT/RIGHT` (used throughout the codebase)
- **Algorithm thresholds** — localization confidence levels, step delays
- **Colors** — all pygame color tuples

### `sim_robot.py` — `Robot`
Represents the simulated robot. Owns its grid position, heading, and sensor state.

| Method | Description |
|---|---|
| `move_to(r, c)` | Teleport to a cell, update pixel coords |
| `move_rel(dr, dc, wall_map)` | Step one cell, blocked by walls. Returns `True` on success |
| `rotate(direction)` | +1 = clockwise, -1 = counter-clockwise (45° steps) |
| `toggle_sensor(idx)` | Enable/disable one of the 5 sensors |
| `measure(wall_map)` | Ray-cast all active sensors; returns distances in mm and hit points in pixels |

The `measure` method steps a ray along each sensor's absolute angle (robot heading + sensor offset), crossing cell boundaries until it hits a wall or reaches `MAX_RANGE_MM`. Distances are converted from pixels to mm and optionally perturbed with Gaussian noise to simulate the real VL53L4CD sensor.

### `map_builder.py`
Everything related to defining and editing the map.

| Function | Description |
|---|---|
| `generate_wall_map()` | Builds the Alon House map as a `(Rows, Cols, 4)` NumPy array |
| `toggle_wall_click(wall_map, mx, my)` | Flips the wall edge nearest to a mouse click; syncs both sides of the shared edge |
| `dump_selected_cells(data, cells)` | Prints pre-computed sensor readings for selected cells to the console (debug) |

The wall map encodes each cell's four edges as `0 = wall` / `1 = open`. Every function that reads or writes walls uses the indices from `config.py` (`WALL_UP`, `WALL_DOWN`, `WALL_LEFT`, `WALL_RIGHT`).

### `localization.py`
Implements the probabilistic localization pipeline (histogram/Bayes filter).

| Function | Description |
|---|---|
| `precompute_all_orientations(wall_map)` | Ray-casts a virtual robot at every (cell, angle) and caches the results |
| `update_probability(prob, measured, expected, angle)` | **Correction step** — weights each cell by Gaussian likelihood of the measurements |
| `predict_motion(prob, dr, dc, wall_map)` | **Prediction step** — shifts the probability cloud then blurs it with a 3×3 kernel |
| `get_best_estimate(prob)` | Returns `(row, col, peak_probability)` of the most likely cell |
| `do_localization_step(robot, wall_map, spin, moves)` | Drives the spin-then-explore recovery sequence one frame at a time |

**Filter sigma:** The Gaussian likelihood uses `σ = 20 mm`. The real sensor is accurate to ±1 mm, but a cell is 180 mm wide. The larger sigma prevents the filter from collapsing when the robot is slightly off-center in a cell.

**Blur kernel:** The motion blur distributes 40% of each cell's mass to itself, 10% to each cardinal neighbor, and 5% to each diagonal neighbor, then re-normalizes. This prevents the filter from becoming overconfident after movement.

### `motion_planning.py`
A thin wrapper that calls into `rrt_planner.py` and returns a path in a format the simulator loop can directly consume.

```python
path = rrt(start=(r, c, theta_idx), goal=(r, c), wall_map=wall_map)
# Returns [(r0,c0), (r1,c1), ...] from start to goal inclusive,
# or [] if no path was found.
```

### `rrt_planner.py`
Standalone RRT (Rapidly-exploring Random Tree) implementation. Has no dependency on pygame or the simulator state.

**Key design rule:** the planner outputs *waypoint positions* (`result["path"]`), not movement commands. The simulator's navigation loop is responsible for turning the robot toward each waypoint and stepping forward.

Key functions:

| Function | Description |
|---|---|
| `plan_rrt(wall_map, start, goal, ...)` | Main entry point — returns a result dict with `"path"`, `"path_with_theta"`, and `"debug"` |
| `validate_path(wall_map, start, goal, path)` | Checks that every step in a path is adjacent and wall-free |
| `path_cells_to_world(path)` | Converts `[(r,c), ...]` to physical `[(x_m, y_m), ...]` coordinates |
| `choose_localization_improving_waypoint(belief, wall_map, pos)` | Suggests a nearby cell with a distinctive wall signature to improve localization |

The planner internally works in `(r, c, theta_idx)` state space. After finding a path, it runs a BFS post-process to replace the RRT path with the shortest valid cell path if that is cheaper.

### `lidar_pygame_sim.py`
Entry point. Contains `main()` and nothing else — all logic lives in the modules above.

Responsibilities:
- Initialize pygame, fonts, and display
- Run the event loop (keyboard, mouse)
- Dispatch to `do_localization_step`, `rrt`, `predict_motion`, `update_probability` at the right times
- Manage the auto-mode state machine (`IDLE → LOCALIZING → PLANNING → MOVING → DONE`)
- Draw the map, probability heatmap, robot, sensor rays, planned path, and sidebar

---

## Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                     lidar_pygame_sim.py                     │
│                                                             │
│  Events ──► Robot.move_rel / rotate / toggle_sensor         │
│                                                             │
│  Each frame:                                                │
│    Robot.measure(wall_map)                                  │
│         │                                                   │
│         ▼                                                   │
│    update_probability(prob, dists, expected_data, angle)    │
│         │                                                   │
│         ▼                                                   │
│    predict_motion(prob, dr, dc, wall_map)  (if moved)       │
│         │                                                   │
│         ▼                                                   │
│    get_best_estimate(prob)                                  │
│         │                                                   │
│         ├─ peak ≥ threshold ──► rrt(start, goal, wall_map)  │
│         │                           │                       │
│         │                           ▼                       │
│         │                      planned_path[]               │
│         │                           │                       │
│         └─────────────────────────► follow path step-by-step│
└─────────────────────────────────────────────────────────────┘

Supporting data structures:
  wall_map      (Rows, Cols, 4)  numpy int array — the map
  expected_data (Rows, Cols, 8, 5) numpy float array — precomputed sensor readings
  prob_matrix   (Rows, Cols)     numpy float array — localization belief
```

---

## Map Format

The map is a NumPy array of shape `(Rows, Cols, 4)`.

- `wall_map[r, c, WALL_UP]` — top edge of cell `(r, c)`
- `wall_map[r, c, WALL_DOWN]` — bottom edge
- `wall_map[r, c, WALL_LEFT]` — left edge
- `wall_map[r, c, WALL_RIGHT]` — right edge

`0 = wall present`, `1 = passage open`.

Every wall is stored on both sides of the shared edge (e.g., the right wall of `(r, c)` equals the left wall of `(r, c+1)`). `map_builder.py` keeps these in sync.

## Heading Convention

Angle index `0` points East; indices increase clockwise at 45° steps.

| Index | Direction |
|---|---|
| 0 | East |
| 1 | South-East |
| 2 | South |
| 3 | South-West |
| 4 | West |
| 5 | North-West |
| 6 | North |
| 7 | North-East |

The RRT planner only moves on cardinal headings (0, 2, 4, 6). Diagonal headings are used only for rotation.
