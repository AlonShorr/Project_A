# pylint: disable=missing-function-docstring,missing-module-docstring
from __future__ import annotations

from itertools import chain
from typing import TYPE_CHECKING

import pytest

from sim.directions import Direction
from sim.maze import ExtendedMaze, Maze
from sim.robots import Action, RobotState
from sim.robots.const import predetermined_action_robot, predetermined_directions_robot, predetermined_path_robot, predetermined_robot

from tests.robots.utils import iter_robot

if TYPE_CHECKING:
    from collections.abc import Iterable


TEST_MAZE = Maze.empty(51, 51)
EXT_TEST_MAZE = ExtendedMaze.full_from_maze(TEST_MAZE)
CENTER = TEST_MAZE.height // 2, TEST_MAZE.width // 2


@pytest.mark.parametrize('actions', [
    [],
    [Action.FORWARD],
    [Action.FORWARD, Action.TURN_LEFT, Action.FORWARD, Action.BACKWARDS, Action.TURN_RIGHT, Action.FORWARD],
    [Action.TURN_LEFT, Action.TURN_LEFT, Action.FORWARD, Action.TURN_RIGHT, Action.FORWARD],
])
def test_predetermined_action_robot(actions: list[Action]):
    robot = predetermined_action_robot(TEST_MAZE, set(), actions=actions)
    assert list(iter_robot(robot, RobotState(*CENTER, Direction.EAST))) == actions


@pytest.mark.parametrize('initial,directions,actions', [
    (Direction.EAST, [],
     []),
    (Direction.EAST, [Direction.EAST],
     [Action.FORWARD]),
    (Direction.EAST, [Direction.EAST, Direction.NORTH, Direction.SOUTH, Direction.WEST],
     [Action.FORWARD, Action.TURN_LEFT, Action.FORWARD, Action.TURN_LEFT, Action.TURN_LEFT,
      Action.FORWARD, Action.TURN_RIGHT, Action.FORWARD]),
    (Direction.WEST, [Direction.NORTH, Direction.NORTH, Direction.WEST],
     [Action.TURN_RIGHT, Action.FORWARD, Action.FORWARD, Action.TURN_LEFT, Action.FORWARD]),
])
def test_predetermined_directions_robot(initial: Direction, directions: Iterable[Direction], actions: list[Action]):
    robot = predetermined_directions_robot(TEST_MAZE, set(), route=chain([initial], directions))
    assert list(iter_robot(robot, RobotState(*CENTER, initial))) == actions


@pytest.mark.parametrize('initial,path,actions', [
    (Direction.EAST, [CENTER],
     []),
    (Direction.EAST, [(0, 0), (0, 1)],
     [Action.FORWARD]),
    (Direction.EAST, [(25, 0), (25, 1), (24, 1), (25, 1), (25, 0)],
     [Action.FORWARD, Action.TURN_LEFT, Action.FORWARD, Action.TURN_LEFT, Action.TURN_LEFT,
      Action.FORWARD, Action.TURN_RIGHT, Action.FORWARD]),
    (Direction.WEST, [(25, 25), (24, 25), (23, 25), (23, 24)],
     [Action.TURN_RIGHT, Action.FORWARD, Action.FORWARD, Action.TURN_LEFT, Action.FORWARD]),
])
def test_predetermined_path_robot(initial: Direction, path: Iterable[tuple[int, int]], actions: list[Action]):
    iter_path = iter(path)  # not using ``path = iter(path)`` because this hides the parameter's value in pytest
    first = next(iter_path)
    robot = predetermined_path_robot(TEST_MAZE, set(), path=chain([first], iter_path), initial_heading=initial)
    assert list(iter_robot(robot, RobotState(*first, initial))) == actions


@pytest.mark.parametrize('actions', [
    [],
    [Action.FORWARD],
    [Action.FORWARD, Action.TURN_LEFT, Action.FORWARD, Action.BACKWARDS, Action.TURN_RIGHT, Action.FORWARD],
    [Action.TURN_LEFT, Action.TURN_LEFT, Action.FORWARD, Action.TURN_RIGHT, Action.FORWARD],
])
def test_predetermined_robot_with_actions(actions: list[Action]):
    robot = predetermined_robot(actions=actions)(EXT_TEST_MAZE, set())
    assert list(iter_robot(robot, RobotState(*CENTER, Direction.EAST))) == actions


@pytest.mark.parametrize('initial,directions,actions', [
    (Direction.EAST, [],
     []),
    (Direction.EAST, [Direction.EAST],
     [Action.FORWARD]),
    (Direction.EAST, [Direction.EAST, Direction.NORTH, Direction.SOUTH, Direction.WEST],
     [Action.FORWARD, Action.TURN_LEFT, Action.FORWARD, Action.TURN_LEFT, Action.TURN_LEFT,
      Action.FORWARD, Action.TURN_RIGHT, Action.FORWARD]),
    (Direction.WEST, [Direction.NORTH, Direction.NORTH, Direction.WEST],
     [Action.TURN_RIGHT, Action.FORWARD, Action.FORWARD, Action.TURN_LEFT, Action.FORWARD]),
])
def test_predetermined_robot_with_directions(initial: Direction, directions: Iterable[Direction], actions: list[Action]):
    robot = predetermined_robot(route=chain([initial], directions))(EXT_TEST_MAZE, set())
    assert list(iter_robot(robot, RobotState(*CENTER, initial))) == actions


@pytest.mark.parametrize('initial,path,actions', [
    (Direction.EAST, [CENTER],
     []),
    (Direction.EAST, [(0, 0), (0, 1)],
     [Action.FORWARD]),
    (Direction.EAST, [(25, 0), (25, 1), (24, 1), (25, 1), (25, 0)],
     [Action.FORWARD, Action.TURN_LEFT, Action.FORWARD, Action.TURN_LEFT, Action.TURN_LEFT,
      Action.FORWARD, Action.TURN_RIGHT, Action.FORWARD]),
    (Direction.WEST, [(25, 25), (24, 25), (23, 25), (23, 24)],
     [Action.TURN_RIGHT, Action.FORWARD, Action.FORWARD, Action.TURN_LEFT, Action.FORWARD]),
])
def test_predetermined_robot_with_path(initial: Direction, path: Iterable[tuple[int, int]], actions: list[Action]):
    iter_path = iter(path)  # not using ``path = iter(path)`` because this hides the parameter's value in pytest
    first = next(iter_path)
    robot = predetermined_robot(path=chain([first], iter_path), initial_heading=initial)(EXT_TEST_MAZE, set())
    assert list(iter_robot(robot, RobotState(*first, initial))) == actions
