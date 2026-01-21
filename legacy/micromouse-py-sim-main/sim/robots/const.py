"""predetermined robots

Move according to a predetermined path.
"""

from __future__ import annotations

from functools import partial
from itertools import chain
from typing import overload, TypedDict, TYPE_CHECKING, Unpack
from .utils import Action, abs_turn_to_actions, cell_to_direction
from ..directions import Direction
from ..maze import ExtendedMaze

if TYPE_CHECKING:
    from collections.abc import Iterable, Set
    from .utils import Algorithm, Robot
    from ..maze import Maze


def _append_unique[T](lst: list[T], val: T) -> None:
    if not lst or lst[-1] != val:
        lst.append(val)


def predetermined_action_robot(
        maze: Maze,
        goals: Set[tuple[int, int]],
        *,
        actions: Iterable[Action],
) -> Robot:
    """A robot that follows a predetermined list of actions.

    Args:
        maze (Maze): The maze (ignored).
        goals (Set[tuple[int, int]]): The goal cells (ignored).
        actions (Iterable[Action]): The actions to perform. This does not need to contain the READY action.

    Returns:
        Robot: The robot's brain.
    """
    _ = maze
    _ = goals

    if isinstance(maze, ExtendedMaze):
        def _update_route():
            _append_unique(maze.route, (pos_row, pos_col))
    else:
        def _update_route():
            pass

    pos_row, pos_col, _ = yield Action.READY
    _update_route()

    for action in actions:
        pos_row, pos_col, _ = yield action
        _update_route()


def _iterate_in_pairs[T](items: Iterable[T]) -> Iterable[tuple[T, T]]:
    it = iter(items)
    try:
        prev = next(it)
    except StopIteration:
        return
    for item in it:
        yield prev, item
        prev = item


def predetermined_directions_robot(
        maze: Maze,
        goals: Set[tuple[int, int]],
        *,
        route: Iterable[Direction],
) -> Robot:
    """A robot that follows a predetermined list of cardinal directions.

    Args:
        maze (Maze): The maze (ignored).
        goals (Set[tuple[int, int]]): The goal cells (ignored).
        route (Iterable[Direction]): The directions to follow. Must have at least 2 elements.
            The first is the starting direction, the rest mean "move 1 cell in X direction".

    Returns:
        Robot: The robot's brain.
    """
    return predetermined_action_robot(
        maze,
        goals,
        actions=chain.from_iterable(abs_turn_to_actions(a, b, allow_reverse=False) for a, b in _iterate_in_pairs(route)),
    )


def predetermined_path_robot(
        maze: Maze,
        goals: Set[tuple[int, int]],
        *,
        path: Iterable[tuple[int, int]],
        initial_heading: Direction,
) -> Robot:
    """A robot that follows a predetermined path through the maze.
    The path is described by cell coordinates and must contain all cells -> every
    two consecutive cells in the path must be adjacent.

    Args:
        maze (Maze): The maze (ignored).
        goals (Set[tuple[int, int]]): The goal cells (ignored).
        path (Iterable[tuple[int, int]]): The path to follow. Must have at least 2 elements.
            The first cell is the starting position.
        initial_heading (Direction): The initial heading of the robot.

    Returns:
        Robot: The robot's brain.
    """
    return predetermined_directions_robot(
        maze,
        goals,
        route=chain(
            [initial_heading],
            (cell_to_direction(a, b) for a, b in _iterate_in_pairs(path)),
        ),
    )


class PathingArgs(TypedDict, total=False):
    """Arguments for the predetermined robot."""
    actions: Iterable[Action]
    route: Iterable[Direction]
    path: Iterable[tuple[int, int]]
    initial_heading: Direction


@overload
def predetermined_robot(*, actions: Iterable[Action]) -> Algorithm: ...
@overload
def predetermined_robot(*, route: Iterable[Direction]) -> Algorithm: ...
@overload
def predetermined_robot(*, path: Iterable[tuple[int, int]], initial_heading: Direction) -> Algorithm: ...


def predetermined_robot(**direction_args: Unpack[PathingArgs]) -> Algorithm:
    """A robot that follows predetermined instructions.

    Args
        actions (Iterable[Action]): Create a ``predetermined_action_robot()``. If provided, no other argument can be passed.
        route (Iterable[Direction]): Create a ``predetermined_directions_robot()``. If provided, no other argument can be passed.
        path (Iterable[tuple[int, int]]): Create a ``predetermined_path_robot()``. If provided, only the ``initial_heading`` argument
            can (and must) be passed.
        initial_heading (Direction): Used with ``path`` to specify the initial heading.

    Raises:
        TypeError: Invalid argument combination.

    Returns:
        Robot: The robot's brain.
    """
    if len(set(direction_args) & {'actions', 'route', 'path'}) != 1:
        raise TypeError
    if 'actions' in direction_args:
        return partial(predetermined_action_robot, actions=direction_args['actions'])
    if 'route' in direction_args:
        return partial(predetermined_directions_robot, route=direction_args['route'])
    if 'path' in direction_args:
        if 'initial_heading' not in direction_args:
            raise TypeError
        return partial(predetermined_path_robot, path=direction_args['path'], initial_heading=direction_args['initial_heading'])
    assert False
