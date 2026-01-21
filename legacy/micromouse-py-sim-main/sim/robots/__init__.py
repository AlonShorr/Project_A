"""Various mouse robot algorithms.

Simple:
+ Idle
+ Random
+ Wall Follower (left/right)
+ Predetermined

Advanced:
+ BFS [TODO]
+ DFS [TODO]
+ Flood Fill:
  + Simple - no diagonals, shortest path
  + Weighted - no diagonals, fastest path by time
  + Thorough explorer - no diagonals, explore (almost) the entire maze, fastest path by time
"""
from __future__ import annotations

from functools import cache
from typing import overload, TYPE_CHECKING

from . import const, flood_fill, idle, random, utils, wall_follower
from .const import predetermined_robot
from .flood_fill import simple_flood_fill, basic_weighted_flood_fill, thorough_flood_fill, dijkstra_flood_fill
from .idle import idle_robot
from .random import random_robot
from .utils import Action, RobotState
from .wall_follower import wall_follower_robot

from ..directions import RelativeDirection

if TYPE_CHECKING:
    from typing import Callable
    from .utils import Algorithm, Robot

ROBOTS: dict[str, Algorithm] = {
    'Idle': idle_robot,
    'Random': random_robot,
    'Left Wall Follower': wall_follower_robot(RelativeDirection.LEFT),
    'Right Wall Follower': wall_follower_robot(RelativeDirection.RIGHT),
    'Flood Fill': simple_flood_fill,
    'Flood Fill -> Dijkstra': basic_weighted_flood_fill,
    'Thorough Flood Fill': thorough_flood_fill,
    'Dijkstra Flood Fill': dijkstra_flood_fill,
}


@overload
def register_robot(name: str, robot: Algorithm) -> Algorithm: ...
@overload
def register_robot(name: str, robot: None = None) -> Callable[[Algorithm], Algorithm]: ...


def register_robot(name: str, robot: Algorithm | None = None) -> Algorithm | Callable[[Algorithm], Algorithm]:
    """Register a robot to the robot registry.

    Args:
        name (str): A nice name for the robot, used to identify the robot.
        robot (Algorithm | None, optional):
            The robot's algorithm, if omitted, this function returns a decorator that registers
            the robot. Defaults to None.

    Returns:
        Algorithm | Callable[[Algorithm], Algorithm]:
            ``robot`` if robot is not ``None``, otherwise, a decorator that accepts an Algorithm,
            registers it and returns it.

    >>> def my_robot(maze, goals):
    ...     yield Action.READY
    ...     yield Action.FORWARD
    >>> _ = register_robot('My Robot', my_robot)
    >>> ROBOTS.get('My Robot') is my_robot
    True
    >>> @register_robot('My Spinner')
    ... def my_spinner(maze, goals):
    ...     yield Action.READY
    ...     while True:
    ...         yield Action.TURN_LEFT
    >>> ROBOTS.get('My Spinner') is my_spinner
    True
    """
    def _decorator(robot: Algorithm) -> Algorithm:
        ROBOTS[name] = robot
        return robot

    if robot is None:
        return _decorator
    return _decorator(robot)


@cache
def load_robots() -> None:
    """Load all robots registered as an entrypoint."""
    import re  # pylint: disable=import-outside-toplevel
    from importlib.metadata import entry_points  # pylint: disable=import-outside-toplevel
    from types import ModuleType  # pylint: disable=import-outside-toplevel

    def capitalize_match(m: re.Match[str]) -> str:
        return m.group().capitalize()

    for robot in entry_points(group='micromouse.robot'):
        try:
            alg = robot.load()
        except (AttributeError, ImportError) as err:
            print(f"warning: failed to load robot {robot.name}: {err}")
            continue
        if isinstance(alg, ModuleType):
            continue
        pretty_name = re.sub(r'\w+', capitalize_match, robot.name.replace('_', ' '))
        module = m.group() if (m := re.match(r'[^:.]+', robot.value)) else robot.value
        register_robot(f"{pretty_name} ({module})", alg)


if TYPE_CHECKING:
    del Callable

del RelativeDirection, annotations, cache, overload, TYPE_CHECKING
