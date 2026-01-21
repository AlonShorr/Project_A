# Micromouse Python Simulator

[![ci](https://github.com/Roynecro97/micromouse-py-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/Roynecro97/micromouse-py-sim/actions/workflows/ci.yml)

Simulator, algorithms and utilities for the [Micromouse][video] competition.

## Installation

***Important!*** This simulator requires Python 3.12 or later.

***Note:*** Mazes are not bundled with the Python package, it is recommended to
keep the `mazes` directory.

### Installing from Source

This method is recommended for general use.

1. Clone the repository:

    ```sh
    git clone https://github.com/Roynecro97/micromouse-py-sim &&
    cd micromouse-py-sim
    ```

2. Install with the GUI (recommended):

    ```sh
    pip install .[gui]
    ```

    Or without the GUI:

    ```sh
    pip install .
    ```

3. You can now safely erase the git clone.

### Installing from the URL

With the GUI (recommended):

```sh
pip install https://github.com/Roynecro97/micromouse-py-sim[gui]
```

Without the GUI:

```sh
pip install https://github.com/Roynecro97/micromouse-py-sim
```

### Developer Mode

This method is recommended for development and users that intend to tweak the
simulator's code.

1. Clone the repository:

    ```sh
    git clone https://github.com/Roynecro97/micromouse-py-sim &&
    cd micromouse-py-sim
    ```

2. Create a virtual env (optional):

    ```sh
    python3.12 -m virtualenv .venv
    source .venv/bin/activate
    ```

3. Install all dev requirements:

    ```sh
    pip install -r dev-requirements.txt
    ```

## Running the Simulation

<!-- TODO: Add images-->

With the simulation installed, the `micromouse` command should be automatically
added to the path:

```sh
$ micromouse -h
usage: micromouse [-h] {sim,tool} ...

Micromouse simulator.

options:
  -h, --help  show this help message and exit

subcommands:
  {sim,tool}
    sim       Run the simulator.
    tool      Run a tool.
```

To run the simulator, use the `micromouse sim` command:

```sh
$ micromouse sim -h
usage: micromouse sim [-h] [-m MAZE] [-s START_POS] [-d START_DIRECTION] [-g GOALS] [-p PRESET] [-e {default}]

Run the simulator.

options:
  -h, --help            show this help message and exit
  -m MAZE, --maze MAZE  The maze to load. May be a file or a maze input. (default: 1x1 maze)
  -s START_POS, --start-pos START_POS
                        The starting position. (default: the top-left corner - (0, 0))
  -d START_DIRECTION, --start-direction START_DIRECTION
                        The starting direction. (default: a valid direction for the starting position)
  -g GOALS, --goals GOALS
                        The goal positions. (default: the bottom-right corner - (M-1, N-1) in a MxN maze)
  -p PRESET, --preset PRESET
                        A maze+start+goals preset to load.
  -e {default}, --engine {default}
                        The render engine to use. (default: a simple pygame GUI renderer, requires the 'gui' feature)
```

### Simulation Flags

+ `-m`/`--maze` `MAZE` - Accepts a [`maze` argument](#the-maze-type).
  Can be repeated to override previous occurances or to partially override a
  preset.
+ `-s`/`--start-pos` - Accepts a [`position` argument](#the-position-type).
  Can be repeated to override previous occurances or to partially override a
  preset.
+ `-d`/`--start-direction` - Accepts a [`direction` argument](#the-direction-type).
  Can be repeated to override previous occurances or to partially override a
  preset.
+ `-g`/`--goals` - Accepts a [`position_set` argument](#the-position-set-type).
  Can be repeated to override previous occurances or to partially override a
  preset.
+ `-p`/`--preset` - Accepts a preset name from the `presets.json` file.
  A preset bundles a maze, a starting position, an initial heading and a set of
  goals.
  Can be repeated to override previous settings.
+ `-e`/`--engine` - A UI engine.
  Defaults to the `default` GUI Pygame-based GUI that is installed with this
  package.

  Alternative engines can be added using the [`micromouse.gui`](#adding-other-renderers)
  entrypoint.

For example, running with the `bad_deadend*.maze` mazes are compatible with the
`semifinal` preset:

```sh
micromouse sim -p semifinal -m mazes/bad_deadends.maze
```

## Using the Simulator as a Package

The simulation can also be accessed through the `sim` Python package for
implementing your own Micromouse solving algorithms, custom simulation, or
tools.

The primary modules are:

### `sim.directions`

Direction enums for navigating a maze.

+ `Direction` - an enum for the cardinal directions (north/east/south/west).
  Supports turning, converting to degrees/radians below the X-axis (clockwise,
  assuming East is the X-axis's positive direction),
  converting to/from strings.
+ `RelativeDirection` - an enum for the relative directions.

### `sim.maze`

Utilities for representing and rendering mazes.

The `Walls` enum is a flags enum for managing a cell's wall configuration.
The flag values match the [`.maz` format](#the-maz-format)'s flags.
The `Walls` enum also supports conversion to/from `bytes` and various rotations
and inversions.

The `Maze` class represents a basic maze.
Supports:

+ `empty(height: int, width: int)` - Create an empty maze with the provided
  size. An empty maze only has its enclosing walls.
+ `full(height: int, width: int)` - Create a full maze with the provided size.
  A full maze has all possible walls.
+ `full_from_maze(maze: Maze)` - Create a new maze of the same size with the
  same walls (clone a maze).
+ `from_maz_file(...)`, `from_maz(...)` - Create a maze from a
  [`.maz`](#the-maz-format) file or `bytes`-like object.
+ `from_maze_file(...)`, `from_maze_text(...)` - Create a maze from a
  [`.maze`](#the-maze-format) file or string.
+ `from_num_file(...)` - Create a maze from a [`.num`](#the-num-format) file.
+ `from_csv_file(...)`, `from_csv(...)` - Create a maze from a
  [`.csv`](#the-csv-format) file or string.
+ `from_file(...)` - Create a maze from any file, the format is detected based
  on the file suffix.
+ `size` - A `(height, width)` size of the maze.
+ `height` - The number of rows in the maze.
+ `width` - The number of columns in the maze.
+ `cell_count` - The number of cells in the maze.
+ `get(row: int, col: int, default = None)` - Get a cell in the maze, if the
  coordinates are out-of-bounds, returns `default`.
+ `__getitem__(idx: tuple[int, int])` - Get a cell in the maze, if the
  coordinates are out-of-bounds, raises an `IndexError`.
+ `__setitem__(idx: tuple[int, int], value: Walls)` - Set the walls of a cell
  in the maze, adjacent cells are updated appropriately.
+ `add_walls(row: int, col: int, walls: Walls)` - Add the specified walls to a
  cell in the maze, adjacent cells are updated appropriately.
+ `remove_walls(row: int, col: int, walls: Walls)` - Remove the specified walls
  from a cell in the maze, adjacent cells are updated appropriately.
+ Iteration - Iterate over `(row, col, cell_walls)` tuples for all cells in the
  maze (in a row-stack order).
+ `render_screen(...)` - Render the maze into a `numpy.ndarray` of strings.
+ `render(...)` - Render the maze into a string.

The `ExtendedMaze` class derives from the `Maze` class and is the type that is
used by the simulation. It adds additional information for each cell and some
functionality:

+ `reset_info()` - Reset the additional information for the entire maze.
+ `extra_info` - A `numpy.ndarray` of `ExtraCellInfo` (see below) with exactly
  the same dimensions as the maze itself.

  Deleting this property (`del maze.extra_info`) resets the additional
  information.
+ `iter_info()` - Iterate over `(row, col, info)` tuples for all cells in the
  maze (in a row-stack order).
+ `iter_all()` - Iterate over `(row, col, walls, info)` tuples for all cells in
  the maze (in a row-stack order).
+ `route` - A list of cells, the simulation UI can display this route for the
  user. Deleting this property (`del maze.route`) resets the route.
+ `explored_cells_count()` - Calculate the number of cells that have been
  explored (visited) by the robot.
+ `explored_cells_percentage()` - `explored_cells_count() / cell_count`.
+ `connectivity` - A [`UnionFind`](#simunionfind) containing connected cell
  sets within the maze. In most mazes, there should only be 1 set that contains
  all cells in the maze (most mazes are fully connected).
+ `changed()` - Checks whether the maze has changed since the last call to this
  method.
+ `mark_changed()` - Mark a change for the `changed()` method. This method is
  automatically called when a wall is added/removed.
+ `render_extra(...)` - Render the maze with additional information such as
  cell weights, the robot's position + orientation and the goals.

The additional information added for each cell is represented by the
`ExtraCellInfo` dataclass and holds:

+ `weight: float | None` - A weight for the cell. Defaults to `None`.
+ `color: ColorName | RGBTuple | None` - A color for the cell. Defaults to
  `None`.
+ `visited: int` - A visit counter. This field is incremented by the simulator
  but can be manipulated by the robot for its own use. Defaults to `0`.

This module also contains text-based rendering utilities:

+ `LineDirection` - Represents lines in a table drawing.
+ `Charset` - type alias for a `(LineDirection) -> str` function.
+ `ascii_charset` - A simple `Charset` that uses ASCII characters (`|`, `-`,
  `+`).
+ `utf8_charset` - A simple `Charset` that uses UTF-8 single-line box
  characters.

### `sim.robots`

Contains robot implementations and utilities.

Type hints:

+ `sim.robots.Robot` - A function that accepts an [`ExtendedMaze`](#simmaze) and
  a set of cell coordinates (`tuple[int, int]`) and returns a generator that
  yields `Action`s and receives `RobotState`s (via `.send()`).
  The first yielded action must be `Action.READY`.
+ `sim.robots.Algorithm` - A function that can be called with no parameters and
  returns a `Robot`.

Types:

+ `sim.robots.RobotState` - A `(row, col, heading)` named tuple (`row` and `col`
  are `int`s and `heading` is a `Direction`).
+ `sim.robots.Action` - An enum of valid robot actions.

Robot management utils:

+ `sim.robots.ROBOTS` - A `dict[str, Algorithm]` containing all registered
  robots. These are the robots available for selection from the simulator UI.
+ `sim.robots.register_robot` - A function that can be used as a decorator or
  directly to register a robot in the `ROBOTS` dictionary.
+ `sim.robots.load_robots` - A function that loads all robots that register via
  the [`micromouse.robot`](#using-the-micromouserobot-entrypoint) entrypoint.

The `sim.robots.utils` sub-module provides generic utilities for implementing
robots.

This module also provides some built-in robots and helper robots.
Since robots are generators, a more complex robot can use other robots as
algorithms.
For example, a robot that uses Dijkstra's algorithm to find the best path to
the goal can use one of the predetermined robots (see below) to convert the
path to actions and yield them.

Built-in robots:

+ `sim.robots.idle` - A robot that immediately gives up.
  + `idle_robot` - Yields the `READY` action and gives up. Useful as a "null"
    robot.
    (also available as `sim.robots.idle_robot`)
+ `sim.robots.const` - Predetermined path robots. These robots accept the path
  as a parameter and follow it. They are meant to be used as tools for robots
  that calculate paths using advanced algorithms and need to convert them to
  `Action`s.
  + `predetermined_robot` - Accepts one of the following keyword arguments:
    + `actions: Iterable[Action]` - Return a `predetermined_action_robot`.
    + `route: Iterable[Direction]` - Return a `predetermined_directions_robot`.
    + `path: Iterable[tuple[int, int]]` and `initial_heading: Direction` -
      Return a `predetermined_path_robot`.
    (also available as `sim.robots.predetermined_robot`)
  + `predetermined_action_robot` - A robot that follows a predetermined list
    of actions. The actions iterable does not need to contain the `READY`
    action.
  + `predetermined_directions_robot` - A robot that follows a predetermined
    list of cardinal directions. Each direction means "move 1 cell in the X
    direction".
  + `predetermined_path_robot` - A robot that follows a predetermined path
    through the maze. The path is described by cell coordinates and must
    contain all cells -> every two consecutive cells in the path must be
    adjacent.
+ `sim.robots.random` - Robots that perform random movements.
  + `random_robot` - Moves in a random available direction.
    (also available as `sim.robots.random_robot`)
  + `better_random_robot` - Moves in a random available direction forward,
    meaning, it attempts to avoid going back to the cell it came from.
+ `sim.robots.wall_follower` - Robots that solve the maze by following the
  right/left wall.
  + `wall_follower_robot` - Accepts a `RelativeDirection` and returns an
    `Algorithm` for a robot that follows the corresponding wall.
    Such robots usually can't solve a Micromouse competition maze.
    (also available as `sim.robots.wall_follower_robot`)
+ `sim.robots.flood_fill` - Robots that solve the maze using the Flood-Fill
  algorithm and additional related utilities.

  Robots:

  + `simple_flood_fill` - A robot that solves the maze using a simple
    implementation of the flood-fill algorithm.
    (also available as `sim.robots.simple_flood_fill`)
  + `basic_weighted_flood_fill` - A robot that explores the maze using the
    Flood-Fill algorithm and then looks for the fastest path using Dijkstra's
    algorithm.
    (also available as `sim.robots.basic_weighted_flood_fill`)
  + `thorough_flood_fill` - A `two_step_robot` (see below) that uses the
    `flood_fill_thorough_explorer` (see below) to explore the maze.
    (also available as `sim.robots.thorough_flood_fill`)

  Utilities:

  + `calc_flood_fill` - A function that calculates Flood-Fill weights.
  + `single_flood_fill` - A robot that uses the Flood-Fill algorithm to reach
    the goal. This robot is meant to be used to implement other Flood-Fill
    based maze traversals.
  + `flood_fill_explore` - A robot that uses the Flood-Fill algorithm to reach
    the goal and return to the starting point.
  + `flood_fill_robot` - A basic robot that explores the maze and then solves
    it using the Flood-Fill algorithm.
  + `dijkstra_solver` - A robot that uses Dijkstra's algorithm to find the
    fastest path to the goal and follows it.
  + `two_step_robot` - A robot that combines 2 robots to solve the maze: an
    explorer (by default `flood_fill_explore`) and a solver (by default:
    `dijkstra_solver`). The explorer must finish its exploration at the
    starting cell.
  + `flood_fill_thorough_explorer` - An explorer robot that uses a combination
    of the Flood-Fill algorithm and Dijkstra's algorithm to explore as much of
    the maze as needed, avoiding detectable dead-ends.

An example of a right-following robot:

```python
from __future__ import annotations

from typing import TYPE_CHECKING

from sim.robots import Action
from sim.robots.utils import direction_to_wall

if TYPE_CHECKING:
    from collections.abc import Set

    from sim.maze import Maze
    from sim.robots import Robot


def my_wall_follower(maze: Maze, goals: Set[tuple[int, int]]) -> Robot:
    """A basic wall-follower robot."""
    row, col, heading = yield Action.READY  # The first action must be "READY"

    while (row, col) not in goals:
        walls = maze[row, col]
        if direction_to_wall(heading.turn_right()) not in walls:
            # No right wall -> should turn right
            yield Action.TURN_RIGHT
        elif direction_to_wall(heading) not in walls:
            # Right wall, but we can move forward -> no turn needed
            pass
        elif direction_to_wall(heading.turn_left()) not in walls:
            # Right and front walls -> turn left
            yield Action.TURN_LEFT
        elif direction_to_wall(heading.turn_back()) not in walls:
            # Dead-end -> turn back
            yield Action.TURN_LEFT
            yield Action.TURN_LEFT
        else:
            # The robot is in a 1x1 box and it is not the goal, give up
            return

        # Step forward
        row, col, heading = yield Action.FORWARD
```

### `sim.simulator`

The simulator's primary logic implementation.

This module implements the `Simulator` class.

The `Simulator` is initialized with a `Maze`, a starting position+heading, a
set of goals and an initial algorithm (usually the `idle_robot`).
These parameters are validated to make sure the maze is solvable and the
starting point is valid (start position is connected to the goal, robot doesn't
start facing a wall, the goal is not an empty set of cells).

The `Simulator` holds 2 [`ExtendedMaze`s](#simmaze) - 1 is the full maze, and 1
is the maze as seen by the robot.

The `Simulator` provides the following functionality:

+ `restart(alg: Algorithm)` - Restart the simulation with the provided
  algorithm. This resets most internal counters and the robot's maze and
  consumes the robot's `READY` action.

  If the robot has no actions, the robot's first action is not `READY`, or the
  robot encounters an error before te first action, the simulation status
  becomes `ERROR` and a `RuntimeError` is raised.

  Otherwise, the simulation's status after this function is `READY`.
+ `step()` - Perform a single action from the robot.

  If the simulation's status is `ERROR`, no action is performed.

  The action is retrieved by calling `robot.send(robot_state)` where
  `robot_state` is a [`RobotState`](#simrobots) instance representing the
  current robot's position and heading.

  After this function, the simulation's status is:

  + `ERROR` if the robot encounters an error or performs an illegal action (for
    example, attempts to go through a wall), in this case, a `RuntimeError` is
    also raised.
  + `FINISHED` if the robot raised `StopIteration` and is located in one of the
    goal cells.
  + `IN_PROGRESS_FOUND_DEST` if the robot found the goal but has more actions.
  + `IN_PROGRESS` otherwise.

+ `maze` - The simulation's maze.

  The optimal route, calculated by Dijkstra's algorithm with full knowledge of
  the maze is saved in this maze's `route` property.
+ `begin` - A `(row, col, heading)` tuple of the robot's initial state.
+ `end` - A set of `(row, col)` coordinates of the goal cells. This set is
  passed to the robot.
+ `robot_maze` - The maze, as seen by the robot with all additional information
  added by the robot.
+ `robot_pos` - The robot's current position + heading.
+ `status` - The current simulation status.
+ `counter` - A set of counters maintained by the simulator. These counters
  include visited cell count, action weight, and action count for both the
  current run (since the last `RESET` action) and the total for entire
  simulation (since the lase `simulator.reset(...)` call).
+ `connected(...)` - Check if cells are connected.

### `sim.front`

Defines classes and utility functions for writing frontends with this package.

The [command-line types](#command-line-types) and the base classes for the
supported custom entrypoints are defined here.

### `sim.tools`

The [provided tools](#provided-tools) are implemented here.

### `sim.unionfind`

A simple Union-Find (Disjoint-Set) implementation.

## Robots

A `Robot` is a generator that yields `Action`s and receives `RobotState`s via
the `send(...)` function. (see [`sim.robots`](#simrobots))

### Provided Robots

There are several robots provided with this package (for algorithm explanations,
see [`sim.robots`](#simrobots)):

+ *Idle* - uses `idle_robot`.
+ *Random* - uses `random_robot`.
+ *Left Wall Follower* - uses `wall_follower_robot(RelativeDirection.LEFT)`.
+ *Right Wall Follower* - uses `wall_follower_robot(RelativeDirection.RIGHT)`.
+ *Flood Fill* - uses `simple_flood_fill`.
+ *Flood Fill -> Dijkstra* - uses `basic_weighted_flood_fill`.
+ *Thorough Flood Fill* - uses `thorough_flood_fill`.
+ *Better Random (sim)* - uses `better_random_robot`.

### Adding Your Own

There are 2 primary methods for adding your own robots to the simulator:

#### Using `register_robot()`

The `sim.robots.register_robot()` function can be used to add robots to the
simulator either via direct calls or when used as a decorator:

```python
# Directly:
register_robot("My Robot's Name", my_robot)

# As a decorator:
@register_robot("My Robot's Name")
def my_robot(maze: ExtendedMaze, goals: Set[tuple[int, int]]) -> Robot:
    ...
```

The caveat for using this method directly is that some UI implementations (such
as the default GUI) require the robots to be registered before starting the UI
and ignore later updates.

#### Using the `micromouse.robot` Entrypoint

You can register your robot using the `micromouse.gui` entrypoint group via
your `pyproject.toml`, `setup.cfg` or `setup.py`. This method guarantees the
robot will be loaded when calling `sim.robots.load_robots()` which is done
early on by the package's main function.

The entrypoint accepts either an `Algorithm` function or a module.

If a module is provided, the entrypoint's name is ignored and the module is
loaded. The module should use [`register_robot(...)`](#using-register_robot) to
register robots defined in it.

If the entrypoint's value is not a module, it is assumed to be an `Algorithm`
and the entrypoint's name is converted from "snake_case" to "Title Case" for
the robot's name.

`sim.robots.random:better_random_robot` is provided as an example for using
this method in this package's `pyproject.toml`.

Examples:

+ In `pyproject.toml`:

    ```toml
    [project.entry-points."micromouse.robot"]
    better_random = "sim.robots.random:better_random_robot"
    ```

+ In `setup.cfg`

    ```ini
    [options.entry_points]
    micromouse.robot =
        better_random = sim.robots.random:better_random_robot
    ```

+ In `setup.py`:

    ```python
    from setuptools import setup

    setup(
        # ...,
        entry_points = {
            'micromouse.robot': [
                "better_random = sim.robots.random:better_random_robot",
            ],
        },
    )
    ```

## GUI Renderers

The simulator supports various UI renderers that can be selected using the
`micromouse sim` command's `-e/--engine` flag.

### Provided Renderer

This package provides a basic GUI renderer (`sim.gui.GUIRenderer`) based on
`pygame`.

To use it, the package should be installed with the `gui` feature enabled.

### Adding Other Renderers

Custom renderers can be added using the `micromouse.gui` entrypoint.

The entrypoint's name is used to identify the engine.

The entrypoint's value should be a class inheriting from the
`sim.front.Renderer` class.

The `default` renderer uses this method to register itself as well.

For example, let's add a "better" renderer for the ***Macro**mouse*
competition.

First, let's implement it in our package, `macromouse.py`:

```python
from sim.gui import GUIRenderer

# GUIRenderer inherits from Renderer so we don't have to do so explicitly.
class MacroRenderer(GUIRenderer):
    """The best renderer."""

    def __init__(self, sim: Simulator):
        super().__init__(sim)
        pg.display.set_caption('Macromouse')
```

Now, we'll add it in our `pyproject.toml`:

```toml
# ...
[project.entry-points."micromouse.gui"]
better = "macromouse:MacroRenderer"
# ...
```

After installing our new package, we can use the new engine:

```sh
$ micromouse sim -h
usage: micromouse sim [-h] [-m MAZE] [-s START_POS] [-d START_DIRECTION] [-g GOALS] [-p PRESET] [-e {default,better}]

Run the simulator.

options:
  -h, --help            show this help message and exit
  -m MAZE, --maze MAZE  The maze to load. May be a file or a maze input. (default: 1x1 maze)
  -s START_POS, --start-pos START_POS
                        The starting position. (default: the top-left corner - (0, 0))
  -d START_DIRECTION, --start-direction START_DIRECTION
                        The starting direction. (default: a valid direction for the starting position)
  -g GOALS, --goals GOALS
                        The goal positions. (default: the bottom-right corner - (M-1, N-1) in a MxN maze)
  -p PRESET, --preset PRESET
                        A maze+start+goals preset to load.
  -e {default,better}, --engine {default,better}
                        The render engine to use. (default: a simple pygame GUI renderer, requires the 'gui' feature)
```

(note that the new engine appeared as an option)

## Tools

This package also allows bundling various Micromouse-related tools.

Tools are accessible via the `micromouse tool` command as subcommands:

```sh
$ micromouse tool -h
usage: micromouse tool [-h] {maze} ...

Run a tool.

options:
  -h, --help  show this help message and exit

tools:
  All registered tools.

  {maze}
    maze      Utilities for viewing and manipulating a maze.
```

### Provided Tool(s)

#### Maze Editor

This package provides a *MazeEditor* tool for basic creation and manipulation
of mazes.

```sh
$ micromouse tool maze -h
usage: micromouse tool maze [-h] {generate,render,rotate,transpose,flip} ...

Utilities for viewing and manipulating a maze.

options:
  -h, --help            show this help message and exit

subcommands:
  {generate,render,rotate,transpose,flip}
    generate            Generate a new maze.
    render              Render a maze (can be used to convert other formats into .maze format).
    rotate              Rotate a maze (can be used to convert other formats into .maze format).
    transpose           Transpose a maze.
    flip                Flip a maze.

Loaded from sim.tools:MazeEditor
```

The avaliable commands for the *MazeEditor* tool are:

##### `generate` - Generate a maze

```sh
$ micromouse tool maze generate -h
usage: micromouse tool maze generate [-h] [-s SIZE] [--empty | --full] output_file

Generate a new maze.

positional arguments:
  output_file           The path to save the new maze at ('-' for stdout).

options:
  -h, --help            show this help message and exit
  -s SIZE, --size SIZE  The size of the maze to create. (default: 16x16)
  --empty               Generate an empty maze (no walls).
  --full                Generate a full maze (all walls). (default)
```

This command is useful for creating new mazes by creating a full maze and
manually removing walls:

```sh
micromouse tool maze generate 16x16 > new.maze
```

##### `render` - Render a maze

```sh
$ micromouse tool maze render -h
usage: micromouse tool maze render [-h] [-A] [-U] [--cell-width CELL_WIDTH] [--cell-height CELL_HEIGHT] [--no-force-corners] [-o OUTPUT_FILE] maze

Render a maze.

positional arguments:
  maze                  The maze to load. May be a file or a maze input.

options:
  -h, --help            show this help message and exit
  -A, --ascii           Use ASCII characters to draw the maze. (default)
  -U, --unicode         Use Unicode (UTF-8) characters to draw the maze.
  --cell-width CELL_WIDTH
                        The amount of characters between cell corners horizontally. (default: 3).
  --cell-height CELL_HEIGHT
                        The amount of characters between cell corners vertically. (default: 1).
  --no-force-corners    Don't draw corners between cells where no walls are attached.
  -o OUTPUT_FILE, --output-file OUTPUT_FILE
                        Render the maze into a file. Use '-' for stdout. (default: stdout)
```

This command can be used for viewing a maze or converting a maze to the
[`.maze` format](#the-maze-format):

```sh
micromouse tool maze render ./mazes/example.num > ./mazes/example.maze
```

##### `rotate` - Rotate a maze

```sh
$ micromouse tool maze rotate -h
usage: micromouse tool maze rotate [-h] [-o OUTPUT_FILE] [-l | -r] [-n ROTATIONS] maze

Rotate a maze.

positional arguments:
  maze                  The maze to load. May be a file or a maze input.

options:
  -h, --help            show this help message and exit
  -o OUTPUT_FILE, --output-file OUTPUT_FILE
                        Rotate the maze into a file. Use '-' for stdout. (default: stdout)
  -l, --left            Rotate the maze to the left (counter-clockwise).
  -r, --right           Rotate the maze to the right (clockwise). (default)
  -n ROTATIONS, --rotations ROTATIONS
                        The number of 90-degree rotations to apply. (default: 1)
```

##### `transpose` - Transpose a maze

```sh
$ micromouse tool maze transpose -h
usage: micromouse tool maze transpose [-h] [-o OUTPUT_FILE] [-s] maze

Transpose a maze.

positional arguments:
  maze                  The maze to load. May be a file or a maze input.

options:
  -h, --help            show this help message and exit
  -o OUTPUT_FILE, --output-file OUTPUT_FILE
                        Transpose the maze into a file. Use '-' for stdout. (default: stdout)
  -s, --secondary-diagonal
                        Transpose the maze along the secondary diagonal. (default: primary diagonal)
```

##### `flip` - Flip a maze

```sh
$ micromouse tool maze flip -h
usage: micromouse tool maze flip [-h] [-o OUTPUT_FILE] [-a {horizontal,vertical}] maze

Flip a maze.

positional arguments:
  maze                  The maze to load. May be a file or a maze input.

options:
  -h, --help            show this help message and exit
  -o OUTPUT_FILE, --output-file OUTPUT_FILE
                        Flip the maze into a file. Use '-' for stdout. (default: stdout)
  -a {horizontal,vertical}, --axis {horizontal,vertical}
                        The axis to flip.
```

### Adding More Tools

Additional tools can be added using the `micromouse.tool` entrypoint.

The entrypoint's name is used for the subcommand's name.

The entrypoint's value should be a class inheriting from the
`sim.front.Tool` class.

The class's docstring is used for for the subcommand's help string.
If the docstring is missing or empty, the entrypoint's value (`module:attr`) is
used for the help instead.

By default, the docstring is also used for the subcommand's description.

The tool's class may also define the `PARSER_ARGS` dictionary to override other
`argparse` keyword arguments when creating the subparser.
The available arguments are: `aliases`, `usage`, `description`, `epilog`,
`formatter_class`, `prefix_chars`, `fromfile_prefix_chars`, `conflict_handler`,
`add_help`, and `allow_abbrev`.

The epilog is always appended with information about the origins of the tool.
For example, the [*Maze Editor*](#maze-editor)'s epilog states it was loaded
from the `MazeEditor` class in the `sim.tools` module.

The tool's class must override the `build_parser(...)` class method that
accepts the tool's (`argparse`) parser and adds arguments to it, and the
`main(...)` class method that accepts the `argparse.Namespace` object that
returned from the `parse_args(...)` call and performs the tool's logic.

For an example of a simple tool see the [*Maze Editor*](#maze-editor) tool's
[implementation](./sim/tools.py).

## Maze Formats

This package supports 4 different maze formats.
These formats are commonly used in other Micromouse-related projects.

### The ".maze" Format

This format is essentially an ASCII-art drawing of the maze but any UTF-8
character is allowed.

Cells are `5x3` by default and share walls in the drawing (for example, a `2x3`
maze will take `5x13` characters).
However, only the center of every wall is checked (by default against a space
character: `' '`).

For example:

```sh
$ cat mazes/example.maze
+---+---+---+
|       |   |
+   +   +   +
|   |       |
+---+---+---+
```

This is the default format.

### The ".maz" Format

This format uses bit flags for the walls of each cell.

```plain
0x1 - North
0x2 - East
0x4 - South
0x8 - West
```

An entire byte is saved per cell although only a nibble per cell would suffice.

The maze is represented as a row stack, meaning that for an `nxm` maze, the
first byte is the `(0, 0)` cell, the second `(0, 1)`, the cell at index `m` is
`(1, 0)` and so on.

Since this is a binary format and cannot communicate maze boundaries, the
dimensions of the maze should be communicated in the filename suffix:
`.{height}x{width}.maz`. If dimensions are not specified but the content's
length (the number of cells) is an integer square, the maze is assumed to be a
square maze of that size.

For example:

```sh
$ hexdump -C ./mazes/example.2x3.maz
00000000  09 03 0b 0e 0c 06                                 |......|
00000006
$ micromouse tool maze render ./mazes/example.2x3.maz
+---+---+---+
|       |   |
+   +   +   +
|   |       |
+---+---+---+
```

### The ".num" Format

This format is a simple text-based format where each line is in the format
`X Y N E S W` - 6 integers that represent:

+ `X` & `Y` - The cell's XY coordinates. Where `(0, 0)` is the ***bottom-left***
  corner (not the *top-left* corner as used everywhere else in this project).
+ `N`, `E`, `S`, `W` - North, East, South, West - 4 booleans (`0`/`1`)
  representing where the cell has walls.

The `Y` coordinate is inverted because the format is based on
[this simulator](https://github.com/mackorone/mms) that uses `(0, 0)` as the
bottom-left corner.

When parsing this format, the maze starts at the invalid size `0x0` and grows
as needed to accommodate new cells (while keeping a rectangular shape).

The lines may be reordered without affecting the maze.

For example:

```sh
$ cat ./mazes/example.num
0 0 0 1 1 1
0 1 1 0 0 1
1 0 0 0 1 1
1 1 1 1 0 0
2 0 0 1 1 0
2 1 1 1 0 1
$ micromouse tool maze render ./mazes/example.num
+---+---+---+
|       |   |
+   +   +   +
|   |       |
+---+---+---+
```

### The ".csv" Format

This format is a CSV file where each line is a row of cells in the maze.

Every cell is represented by a number between `1` and `16`, each number
represents a wall configuration:

![CSV Values]

For example:

```sh
$ cat ./mazes/example.csv
8,7,13
11,5,6
$ micromouse tool maze render ./mazes/example.csv
+---+---+---+
|       |   |
+   +   +   +
|   |       |
+---+---+---+
```

## Command-line Types

### The Maze Type

The maze type accepts an optional [*maze format*](#maze-formats) followed by
either a *maze file* or a *maze literal*.

A *maze file* is a path to a file containing a maze in one of the supported
formats. If the *maze format* is not specified, it is deduced from the file
extension (defaults to [the `.maze` format](#the-maze-format) if unknown).

A *maze literal* is the content of a maze file as a single string. The *maze
format* is required in this case.

For example, the `micromouse tool maze render` accepts a `maze` positional argument:

```sh
$ micromouse tool maze render ./mazes/simple.maze
+---+---+---+---+---+
|   |               |
+   +   +---+   +---+
|   |       |       |
+   +---+---+   +   +
|               |   |
+   +---+   +   +   +
|   |       |       |
+   +---+---+---+   +
|   |               |
+---+---+---+---+---+
$ micromouse tool maze render $'csv:13,8 ,10,2 ,12\n9 ,5 ,12,1 ,7\n1 ,2 ,2 ,3 ,9\n9 ,5 ,6 ,5 ,3\n11,14,10,10,6'
+---+---+---+---+---+
|   |               |
+   +   +---+   +---+
|   |       |       |
+   +---+---+   +   +
|               |   |
+   +   +   +   +   +
|   |       |       |
+   +---+---+---+   +
|   |               |
+---+---+---+---+---+
```

### The Position Type

The position type represents a pair of coordinates withing the maze in a
`({row}, {col})` format.
It is comprised of 2 comma-separated integer coordinates. The coordinates can
be surrounded by parentheses and whitespaces are ignored.

For example, valid coordinates (assuming a `16x16` maze) are:

```plain
15, 0
0,15
(15,15)
(10, 12)
```

### The Position Set Type

The position set type represents a set of [positions](#the-position-type) within
the maze.
It is a list of [positions](#the-position-type), separated by semicolons (`:`).
(The same rules for parsing a single position apply for each position in the
set).

For example:

```plain
7,7:7,8:8,7:8,8
```

### The Size Type

The position type represents a maze size in a `{height}x{width}` format.
It is comprised of 2 `x`-separated integer dimensions.

For example:

```plain
16x16
5x10
```

### The Direction Type

The direction type represents a cardinal direction.
It accepts either a cardinal direction's name or its abbreviation
(case-insensitive).

For example:

```plain
North
E
south
w
```

[video]: https://www.youtube.com/watch?v=ZMQbHMgK2rw "Veritasium video about Micromouse"
[CSV Values]: documents/images/CSV_format_diagram.png
