"""Utilities for robot testing."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sim.directions import Direction
from sim.robots import Action, RobotState


if TYPE_CHECKING:
    from collections.abc import Iterable
    from sim.robots import Algorithm, Robot


def iter_robot(robot: Robot, begin: RobotState) -> Iterable[Action]:
    """Iterate over a robot's commands.

    Args:
        robot (Robot): The robot to iterate.
        begin (RobotState): The starting position.

    Returns:
        Iterable[Action]: The robot's actions. **Excluding the first READY**
    """
    try:
        first = next(robot)
    except StopIteration:
        return

    assert first == Action.READY, f"first action must always be {Action.READY}"

    pos = begin
    while True:
        try:
            action = robot.send(pos)
        except StopIteration:
            return
        match action:
            case Action.READY:
                pass
            case Action.RESET:
                pos = begin
            case Action.FORWARD:
                match pos.heading:
                    case Direction.NORTH:
                        pos = RobotState(pos.row - 1, pos.col, pos.heading)
                    case Direction.EAST:
                        pos = RobotState(pos.row, pos.col + 1, pos.heading)
                    case Direction.SOUTH:
                        pos = RobotState(pos.row + 1, pos.col, pos.heading)
                    case Direction.WEST:
                        pos = RobotState(pos.row, pos.col - 1, pos.heading)
                    case _:
                        raise NotImplementedError("secondary directions not yet supported")
            case Action.BACKWARDS:
                match pos.heading:
                    case Direction.NORTH:
                        pos = RobotState(pos.row + 1, pos.col, pos.heading)
                    case Direction.EAST:
                        pos = RobotState(pos.row, pos.col - 1, pos.heading)
                    case Direction.SOUTH:
                        pos = RobotState(pos.row - 1, pos.col, pos.heading)
                    case Direction.WEST:
                        pos = RobotState(pos.row, pos.col + 1, pos.heading)
                    case _:
                        raise NotImplementedError("secondary directions not yet supported")
            case Action.TURN_LEFT:
                pos = RobotState(pos.row, pos.col, pos.heading.turn_left())
            case Action.TURN_RIGHT:
                pos = RobotState(pos.row, pos.col, pos.heading.turn_right())
            case _:
                raise AssertionError(f"unexpected action: {action!r}")
        yield action
