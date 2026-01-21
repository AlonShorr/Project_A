"""idle robot

The idle_robot is a robot that doesn't move from the starting point.
It's purpose is to allow simulations easier handling of an idle robot.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .utils import Action

if TYPE_CHECKING:
    from collections.abc import Set

    from .utils import Robot
    from ..maze import Maze


def idle_robot(maze: Maze, goals: Set[tuple[int, int]]) -> Robot:
    """A robot with random movements.

    Args:
        maze (Maze): The maze (ignored).
        goals (Set[tuple[int, int]]): The goal cells (ignored).

    Returns:
        Robot: The robot's brain.
    """
    _ = maze
    _ = goals
    yield Action.READY
