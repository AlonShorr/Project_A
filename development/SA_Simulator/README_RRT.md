RRT Planner Handoff
===================

Files
-----

This handoff covers:

- rrt_planner.py
- rrt_demo.py

Both files live in:

    development/SA_Simulator/

The planner is standalone. It does not modify the existing simulator loop, and
it does not modify the physical robot control code.


Purpose
-------

The goal is to provide a standalone RRT-based path planner for the Python
simulator.

The key design rule is that the planner outputs waypoint positions, not robot
movement commands.

Official output:

    result["path"] == [(r1, c1), (r2, c2), ...]

The physical robot already has a control loop that receives target positions
and drives the robot toward them. Therefore the planner should provide target
cells or physical target coordinates.

Debug movement actions such as FORWARD, TURN_LEFT, and TURN_RIGHT are generated
only for human inspection. They are not the official control output.


Map Representation
------------------

The planner is compatible with the simulator wall map representation:

    wall_map.shape == (rows, cols, 4)

Wall indices:

    0 = up
    1 = down
    2 = left
    3 = right

Wall values:

    0 = wall
    1 = open

The planner checks movement by looking at the wall value in the current cell
and the direction of travel. It also prevents movement outside the map.


Heading Convention
------------------

The planner uses the Pygame simulator's 8-heading convention:

    0 = east
    1 = south-east
    2 = south
    3 = south-west
    4 = west
    5 = north-west
    6 = north
    7 = north-east

Forward movement is allowed only for cardinal headings:

    0 = east
    2 = south
    4 = west
    6 = north

Diagonal headings do not move forward. This prevents diagonal movement through
cell corners.


rrt_planner.py
--------------

Main API:

    def plan_rrt(
        wall_map,
        start,
        goal,
        start_theta=None,
        goal_theta=None,
        max_iter=2000,
        goal_sample_rate=0.15,
        random_seed=None,
        return_debug=True,
    ):

Inputs:

- wall_map: simulator wall map
- start: (r, c) or (r, c, theta_idx)
- goal: (r, c) or (r, c, theta_idx)
- start_theta: optional theta if start is only (r, c)
- goal_theta: optional final theta requirement
- max_iter: maximum RRT expansion attempts
- goal_sample_rate: probability of sampling the goal directly
- random_seed: deterministic seed for repeatable runs
- return_debug: include tree edges and debug data

Internal planner state:

    (r, c, theta_idx)

Official external output:

    [(r, c), ...]

The start cell is intentionally excluded from result["path"]. The first element
is the next target after the current robot position.


Important Functions
-------------------

normalize_state(...)

Converts (r, c) or (r, c, theta) into:

    (r, c, theta_idx)


is_state_valid(...)

Checks that a state is inside the map and has a valid theta index.


theta_to_cardinal_move(...)

Maps a cardinal theta to:

    (dr, dc, wall_index)

Diagonal theta values return None.


can_move_forward(...)

Checks if the robot can move forward one cell:

- state is valid
- heading is cardinal
- destination is inside the map
- the wall in the movement direction is open

This is the main compatibility point with the simulator wall map.


apply_internal_action(...)

Applies one internal/debug action:

    TURN_LEFT
    TURN_RIGHT
    TURN_AROUND
    FORWARD

These actions are used for RRT expansion and debug replay only.


plan_rrt(...)

High-level algorithm:

1. Normalize start and goal.
2. Validate start and goal.
3. Create an RRT tree rooted at the start state.
4. Repeatedly sample a random state, with occasional goal bias.
5. Find the nearest existing tree node.
6. Try primitive actions from that node.
7. Add a valid new state to the tree.
8. Stop when the tree reaches the goal cell, and goal theta if required.
9. Reconstruct the state path.
10. Convert it into official waypoint cells.
11. Validate the waypoint path against the wall map.
12. Optionally post-process to a shortest valid cell path.
13. Return the result dictionary.

The planner includes debug metadata so it is clear whether the official output
comes directly from the RRT tree or from valid post-processing:

    debug["path_source"]

Possible values:

    "rrt_tree"
    "shortest_valid_postprocess"

The raw sampled-tree path is also kept:

    debug["raw_rrt_path"]
    debug["raw_rrt_path_with_theta"]


validate_path(...)

Checks the official path:

- path excludes the start cell
- final cell equals the goal cell
- every step is adjacent
- every step stays inside the map
- every step respects walls


state_path_to_debug_actions(...)

Converts a state path into human-readable debug actions.

Example:

    ["FORWARD", "FORWARD", "TURN_RIGHT", "TURN_RIGHT", "FORWARD"]

These actions are not for the real robot control layer.


cell_to_world(...)

Converts a cell center to physical coordinates:

    x = (c + 0.5) * cell_size_m
    y = (r + 0.5) * cell_size_m

Default cell size:

    0.18 meters


path_cells_to_world(...)

Converts:

    [(r1, c1), (r2, c2), ...]

into:

    [(x1_m, y1_m), (x2_m, y2_m), ...]

This is the helper to use when the physical robot control layer needs target
positions in meters.


route_observability_score(...)

Scores paths higher when they pass through less-common wall signatures and
changing wall patterns.

This is a simple localization-awareness helper, not the main route objective.


choose_localization_improving_waypoint(...)

Optional helper for cases where localization is uncertain.

It suggests a nearby waypoint that may improve localization belief.

Return format:

    {
        "success": True,
        "waypoint": (r, c),
        "score": float,
        "reason": str,
    }

It still returns a waypoint, not movement commands.


Planner Output
--------------

Success:

    {
        "success": True,
        "path": [(r1, c1), (r2, c2), ...],
        "path_with_theta": [(r1, c1, theta1), ...],
        "debug": {
            "iterations": int,
            "num_nodes": int,
            "reason": "goal reached",
            "tree_edges": [
                ((r_parent, c_parent, theta_parent),
                 (r_child, c_child, theta_child)),
                ...
            ],
            "debug_actions": ["FORWARD", "TURN_RIGHT", ...],
            "path_cost": float,
            "path_source": str,
            "raw_rrt_path": [...],
            "raw_rrt_path_with_theta": [...],
        }
    }

Failure:

    {
        "success": False,
        "path": [],
        "path_with_theta": [],
        "debug": {
            "reason": "...",
            "iterations": int,
            "num_nodes": int,
        }
    }


Using the Planner in the Simulator
----------------------------------

Example:

    from rrt_planner import plan_rrt, path_cells_to_world

    result = plan_rrt(
        wall_map=wall_map,
        start=(localized_r, localized_c, localized_theta),
        goal=(goal_r, goal_c),
        random_seed=123,
    )

    if result["success"]:
        path_cells = result["path"]
        path_world = path_cells_to_world(path_cells)

        # Segev's planning block can insert path_cells or path_world into the
        # target array. The real control layer should receive the next target
        # position from this array.
    else:
        print("RRT failed:", result["debug"]["reason"])

Use this as the target list:

    result["path"]

or, for physical coordinates:

    path_cells_to_world(result["path"])

Do not use this as robot commands:

    result["debug"]["debug_actions"]


rrt_demo.py
-----------

The demo is a verification script for the planner.

Run it from the simulator directory using the simulator virtual environment:

    cd /home/alonshorr/dev/Project_A/development/SA_Simulator
    ./robot_sim/bin/python rrt_demo.py

The demo intentionally requires robot_sim. If run with system Python, it exits
and prints the correct command.


Map Used by the Demo
--------------------

The demo uses the real Pygame simulator map:

    lidar_pygame_sim.generate_wall_map

It prints:

    Loaded map source: real Pygame simulator map
    Loaded map factory: lidar_pygame_sim.generate_wall_map
    Loaded map shape: (15, 20, 4)

The demo also validates the map representation before planning:

- shape is (rows, cols, 4)
- wall values are 0 or 1
- borders are closed
- neighboring cells agree on shared walls


Demo Checks
-----------

The demo checks:

- wall map representation is compatible
- moving outside the map fails
- moving into a wall fails
- plot coordinates use (c, r)
- every case reaches the goal
- every path step respects walls
- result["path"] excludes the start cell
- final path cell equals the goal cell
- debug_actions replay into path_with_theta
- every FORWARD action moves exactly one cell
- turn actions do not create cell waypoints

For each case it prints:

    Case 0:
    map source: ...
    start: ...
    goal: ...
    seed: ...
    success: ...
    iterations: ...
    tree nodes: ...
    path source: ...
    raw RRT path cells excluding start: ...
    path cells excluding start: ...
    full cell path including start: ...
    path_with_theta: ...
    debug_actions: ...
    world path: ...
    path cost: ...
    step-by-step:
      state (...) --ACTION--> state (...)
    verification: True (valid)


Seed and Iterations
-------------------

seed is the deterministic random seed passed into the planner.

Using the same seed should reproduce the same sampled-tree behavior.

iterations is the number of RRT sampling and expansion attempts used before
the planner connected to the goal.

Current demo seeds:

    Case 0: seed 100
    Case 1: seed 101
    Case 2: seed 102


Demo Plots
----------

The demo generates one plot per case:

    rrt_demo_case_0.png
    rrt_demo_case_1.png
    rrt_demo_case_2.png

Each plot shows:

- one visible tile per map row and column
- black walls
- faint blue RRT tree edges
- red official waypoint path
- green start
- red goal

Plot convention:

    x-axis = column c
    y-axis = row r
    row 0 is at the top
    path/start/goal are plotted as (c, r)

This matches the Pygame simulator grid.

The plot title includes:

- case number
- map source
- map shape
- seed
- iterations
- coordinate convention


Debugging Workflow
------------------

Run:

    cd /home/alonshorr/dev/Project_A/development/SA_Simulator
    ./robot_sim/bin/python rrt_demo.py

Check the terminal output for:

    Wall map compatibility: True
    verification: True (valid)
    path/debug validation passed for all cases: True

Inspect the official output:

    path cells excluding start: [...]

That is what a planning/control integration should consume.

Inspect the full path for manual checking:

    full cell path including start: [...]

Inspect debug replay:

    step-by-step:
      state (...) --ACTION--> state (...)

Rules:

- FORWARD should move exactly one cell.
- TURN_LEFT, TURN_RIGHT, and TURN_AROUND should not change the cell.
- The final cell should equal the goal cell.
- The red plot path should not cross black walls.


Integration Warning
-------------------

Do not feed debug actions to the robot.

Wrong:

    result["debug"]["debug_actions"]

Correct:

    result["path"]

or:

    path_cells_to_world(result["path"])

