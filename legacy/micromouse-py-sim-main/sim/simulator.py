"""Simulator
"""
from __future__ import annotations

import sys
import time

from contextlib import contextmanager
from dataclasses import dataclass
from enum import auto, Enum
from typing import Self, TYPE_CHECKING

from .directions import Direction, RelativeDirection
from .maze import ExtendedMaze, Maze
from .robots import Action, RobotState
from .robots.utils import build_weighted_graph, dijkstra, direction_to_wall

if TYPE_CHECKING:
    from collections.abc import Iterable

    from .robots import Algorithm, Robot
    from .maze import ExtraCellInfo


@dataclass
class CounterData:
    """A counter with current and total."""
    current: int = 0
    total: int = 0

    def __iadd__(self, amount: int) -> Self:
        """Increment counters by amount."""
        self.current += amount
        self.total += amount
        return self


class Counter:
    """A counter for various robot actions."""

    def __init__(self) -> None:
        self._step = CounterData()
        self._weight = CounterData()
        self._cell = CounterData()

    def count_action(self, action: Action) -> None:
        """Update counters according to the robot's action.

        Args:
            action (Action): The robot's action.
        """
        self._step += 1
        match action:
            case Action.READY:
                weight = 0
                cell_advanced = False
            case Action.RESET:
                weight = 0
                cell_advanced = False
                self.reset_current()
            case Action.FORWARD:
                weight = 1
                cell_advanced = True
            case Action.BACKWARDS:
                weight = 2
                cell_advanced = True
            case Action.TURN_LEFT | Action.TURN_RIGHT:
                weight = 4 - 1
                cell_advanced = False

        self._weight += weight
        self._cell += int(cell_advanced)

    def reset_current(self) -> None:
        """Reset current counters."""
        self._step.current = 0
        self._weight.current = 0
        self._cell.current = 0

    @property
    def current_step(self) -> int:
        """The number of steps that the robot made this run."""
        return self._step.current

    @property
    def total_step(self) -> int:
        """The total number of steps that the robot made."""
        return self._step.total

    @property
    def current_weight(self) -> int:
        """The weight of actions that the robot made this run."""
        return self._weight.current

    @property
    def total_weight(self) -> int:
        """The total weight of actions that the robot made."""
        return self._weight.total

    @property
    def current_cell(self) -> int:
        """The number of cells that the robot stepped through this run."""
        return self._cell.current

    @property
    def total_cell(self) -> int:
        """The total number of cells that the robot stepped through."""
        return self._cell.total


class SimulationStatus(Enum):
    """Represents the simulation's status."""

    READY = auto()
    IN_PROGRESS = auto()
    IN_PROGRESS_FOUND_DEST = auto()
    FINISHED = auto()
    ERROR = auto()


@contextmanager
def timed(title: str = ""):
    """A contextmanager for measuring time (in seconds).

    Args:
        title (str, optional): An optional title to add to the time measurement. Defaults to "".
    """
    t0 = time.perf_counter()
    try:
        yield
    finally:
        t1 = time.perf_counter()
        elapsed = t1 - t0
        if exc := sys.exception():
            result = 'failed'
            tail = f" ({type(exc).__name__}: {exc!s})"
        else:
            result = 'completed'
            tail = ""
        print(f"{title}{title and ' '}{result} in {elapsed:0.6f}s{tail}")


class Simulator:  # pylint: disable=too-many-instance-attributes
    """A micromouse simulator."""
    _status: SimulationStatus
    __robot_pos: tuple[int, int, Direction]
    _counter: Counter

    def __init__(self, alg: Algorithm, maze: Maze, begin: tuple[int, int, Direction], end: Iterable[tuple[int, int]]):
        """Initialize a new simulator.

        Args:
            alg (Algorithm): The initial robot's algorithm.
            maze (Maze): The maze for the simulation.
            begin (tuple[int, int, Direction]): The robot's starting position + heading.
            end (Iterable[tuple[int, int]]): The goal cells.

        Raises:
            ValueError: The robot starts facing a wall.
            ValueError: There are no end cells.
            ValueError: The goals are not reachable from the starting position.
        """
        self._maze = ExtendedMaze.full_from_maze(maze)
        self._begin = begin
        self._end = frozenset(end)

        if self._begin[:-1] not in self._end and direction_to_wall(self._begin[-1]) in self._maze[self._begin[:-1]]:
            raise ValueError("robot starts facing a wall")
        if not self._end:
            raise ValueError("must specify at least 1 end cell")
        if not self.connected(self._begin[:-1], self._end):
            raise ValueError("the starting position (begin) is not connected to all goal positions (end)")

        self._maze.route = min(
            dijkstra(
                build_weighted_graph(
                    self._maze,
                    {
                        RelativeDirection.FRONT: 1,
                        RelativeDirection.BACK: 2,
                        RelativeDirection.LEFT: 4,
                        RelativeDirection.RIGHT: 4,
                    },
                    start=self._begin,
                ),
                self._begin[:-1],
                goals=self._end,
            ).values(),
        )[1]

        self.restart(alg)

    def restart(self, alg: Algorithm):
        """Restart the simulator with a new algorithm.
        The first READY action is consumed in this function.

        Args:
            alg (Algorithm): The new algorithm to use.

        Raises:
            RuntimeError: The robot yielded no actions.
            RuntimeError: The robot encountered an error on init (before yielding READY).
            RuntimeError: The robot's first action is not READY.
        """
        print(f"sim: restarting with a {self.maze.height}x{self.maze.width} maze")
        print(f"sim: robot will start at {self._begin[:-1]} facing {self._begin[-1]}")
        self._maze.reset_info()
        self._robot_maze = ExtendedMaze.empty(self.maze.height, self.maze.width)
        self._robot_pos = self._begin

        self._robot = alg(self._robot_maze, self._end)

        self._status = SimulationStatus.ERROR
        try:
            with timed("sim: robot init"):
                state = next(self._robot)
        except StopIteration as stop:
            raise RuntimeError("robot failed to start - stopped before yielding any action") from stop
        except Exception as err:
            raise RuntimeError("robot failed to start - encountered an error") from err
        if state is not Action.READY:
            raise RuntimeError(f"robot malfunction - yielded {state} instead of {Action.READY}")
        self._status = SimulationStatus.READY
        print("sim: robot is ready")
        self._counter = Counter()

    def step(self) -> SimulationStatus:
        """Perform a single robot action.

        Raises:
            RuntimeError: The robot encountered an error.
            RuntimeError: The robot performed an illegal action (illegal RESET).

        Returns:
            SimulationStatus: The status of the simulation after the step.
        """
        # if self._status not in (SimulationStatus.IN_PROGRESS, SimulationStatus.READY):
        #     # Simulation is not running.
        #     return self._status
        # self._status = SimulationStatus.IN_PROGRESS

        if self._status is SimulationStatus.ERROR:
            print(f"sim: refusing to step, status is {self.status}")
            return self._status

        if self._status is SimulationStatus.READY:
            print("sim: starting progress")
            self._status = SimulationStatus.IN_PROGRESS

        row, col, heading = self._robot_pos
        print(f"sim: robot is at {(row, col)} facing {heading}")

        try:
            with timed("sim: robot step"):
                action = self._robot.send(RobotState(*self._robot_pos))
        except StopIteration:
            self._status = SimulationStatus.FINISHED if (row, col) in self._end else SimulationStatus.ERROR
            print(f"sim: robot stopped, status is {self.status}")
            return self._status
        except Exception as err:
            self._status = SimulationStatus.ERROR
            raise RuntimeError("robot encountered an error") from err

        print(f"sim: selected action is {action}")

        match action:
            case Action.READY:
                pass
                # self._status = SimulationStatus.ERROR
                # raise RuntimeError(f"robot malfunction - yielded {action} instead of moving")
            case Action.RESET:
                print("sim: robot asked for reset")
                if self._robot_pos[:-1] != self._begin[:-1] or self._status is not SimulationStatus.IN_PROGRESS_FOUND_DEST:
                    self._status = SimulationStatus.ERROR
                    raise RuntimeError("reset must be done from the starting position and after finding the goal")
                # self._maze.reset_info()  # Reset would affect explore percentage & reset the heatmap
                self._robot_pos = self._begin
                self._status = SimulationStatus.READY  # Allow the robot to call ready again
            case Action.FORWARD | Action.BACKWARDS:
                if not self._robot_step(heading if action is Action.FORWARD else heading.turn_back()):
                    print("sim: step error")
                    self._status = SimulationStatus.ERROR
                elif self._robot_pos[:-1] in self._end:
                    self._status = SimulationStatus.IN_PROGRESS_FOUND_DEST
            case Action.TURN_LEFT:
                self._robot_pos = (row, col, heading.turn_left())
                print("sim: robot turned left")
            case Action.TURN_RIGHT:
                self._robot_pos = (row, col, heading.turn_right())
                print("sim: robot turned right")

        self._counter.count_action(action)
        return self._status

    def _robot_step(self, direction: Direction) -> bool:
        """Try to advance the robot in the given direction, return False if not possible."""
        row, col, heading = self._robot_pos

        print(f"sim: stepping from {(row, col)} to {direction} (while facing {heading})")

        if direction_to_wall(direction) in self._maze[row, col]:
            print(f"sim: crashed! {direction_to_wall(direction)=!s} to {self._maze[row, col]=!s}")
            # Robot crashed into a wall
            return False

        # No need to check boundaries because our maze is enclosed by walls
        match direction:
            case Direction.NORTH: self._robot_pos = (row - 1, col, heading)
            case Direction.EAST: self._robot_pos = (row, col + 1, heading)
            case Direction.SOUTH: self._robot_pos = (row + 1, col, heading)
            case Direction.WEST: self._robot_pos = (row, col - 1, heading)
            case _: raise AssertionError(f"only the primary directions are supported right now (not {direction})")

        print(f"sim: robot is now at {self._robot_pos[:-1]} facing {self._robot_pos[-1]}")
        return True

    @property
    def _robot_pos(self) -> tuple[int, int, Direction]:
        return self.__robot_pos

    @_robot_pos.setter
    def _robot_pos(self, new_pos: tuple[int, int, Direction]):
        self.__robot_pos = new_pos

        robot_pos = self.__robot_pos[:-1]
        self._robot_maze[robot_pos] = self._maze[robot_pos]
        self._robot_maze.extra_info[robot_pos].visit_cell()
        info: ExtraCellInfo = self._maze.extra_info[robot_pos]
        info.visit_cell()

    @property
    def maze(self) -> ExtendedMaze:  # TODO: readonly version
        """The maze used in the simulator."""
        return self._maze

    @property
    def begin(self) -> tuple[int, int, Direction]:
        """The starting position in the maze."""
        return self._begin

    @property
    def end(self) -> frozenset[tuple[int, int]]:
        """The goal position(s) in the maze."""
        return self._end

    @property
    def robot_maze(self) -> ExtendedMaze:  # TODO: readonly version
        """The maze that the robot sees."""
        return self._robot_maze

    @property
    def robot_pos(self) -> tuple[int, int, Direction]:
        """The robot's current position."""
        return self._robot_pos

    @property
    def status(self) -> SimulationStatus:
        """The status of the simulator."""
        return self._status

    @property
    def counter(self) -> Counter:  # TODO: readonly version
        """The counter used by the simulator."""
        return self._counter

    def connected(self, a: tuple[int, int] | Iterable[tuple[int, int]], b: tuple[int, int] | Iterable[tuple[int, int]]) -> bool:
        """Check if two cells (or cell groups) are connected in the maze.

        Args:
            a (tuple[int, int] | Iterable[tuple[int, int]]): Cell group A.
            b (tuple[int, int] | Iterable[tuple[int, int]]): Cell group B.

        Returns:
            bool: True if all cells in group A are connected to all cells in group B.
        """
        # Calculate connectivity
        connectivity = self._maze.connectivity

        # Normalize input to 2 sets
        def _normalize(maybe_group: tuple[int, int] | Iterable[tuple[int, int]]) -> frozenset[tuple[int, int]]:
            if not isinstance(maybe_group, tuple):
                return frozenset(maybe_group)
            if len(maybe_group) != 2 or isinstance(maybe_group[0], tuple):
                return frozenset(maybe_group)  # type: ignore
            return frozenset({maybe_group})  # type: ignore

        # Check connectivity
        return all(all(connectivity.connected(point_a, point_b) for point_b in _normalize(b)) for point_a in _normalize(a))
