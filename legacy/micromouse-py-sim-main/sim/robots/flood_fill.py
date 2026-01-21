"""flood fill robot

Uses flood-fill to get to the goal and then back to the start,
then uses an "optimal" route (by default, also using flood-fill).
"""

from __future__ import annotations

import math

from functools import partial, reduce
from operator import or_
from typing import Protocol, TypedDict, TYPE_CHECKING

from .utils import (
    Action,
    GiveUpTimer,
    abs_turn_to_actions,
    adjacent_cells,
    build_weighted_graph,
    dijkstra,
    direction_to_cell,
    identity,
    mark_deadends,
    mark_unreachable_groups,
    shuffled,
    walls_to_directions,
)
from .const import predetermined_path_robot
from ..directions import Direction, RelativeDirection
from ..maze import ExtendedMaze, Walls
from ..unionfind import UnionFind

if TYPE_CHECKING:
    from collections.abc import Iterable, Set
    from typing import Callable, Unpack
    from ..maze import ExtraCellInfo
    from .utils import Algorithm, Robot, RobotState, SolverAlgorithm

    type MinorPriority = Callable[[list[Direction]], Iterable[Direction]]


class WeightArgs(TypedDict, total=True):
    """Arguments for weighted calculators."""
    maze: ExtendedMaze
    cell: tuple[int, int]
    info: ExtraCellInfo
    marker: int


class WeightCalc(Protocol):  # pylint: disable=too-few-public-methods
    """A flood-fill weight calculator."""

    def __call__(self, **kwargs: Unpack[WeightArgs]) -> float:
        ...


UNREACHABLE_WEIGHT = math.inf


def simple_flood_weight(**kwargs: Unpack[WeightArgs]) -> float:
    """Simple length-based flood fill weight (aka weight-less flood fill)."""
    return kwargs['marker']


def simple_flood_weight_with_norm_visit_bias(**kwargs: Unpack[WeightArgs]) -> float:
    """
    Simple length-based flood fill weight (aka weight-less flood fill) but with
    a weak bias against seen cells.
    """
    if kwargs['marker'] == 0:
        return 0
    return kwargs['marker'] + bool(kwargs['info'].visited)


def simple_flood_weight_with_strong_visit_bias(**kwargs: Unpack[WeightArgs]) -> float:
    """
    Simple length-based flood fill weight (aka weight-less flood fill) but with
    a weak bias against seen cells.
    """
    if kwargs['marker'] == 0:
        return 0
    return kwargs['marker'] + round(kwargs['info'].visited * 0.5)


def weight_with_avoid_cells(weight: WeightCalc, avoid: Set[tuple[int, int]], avoid_weight: float = UNREACHABLE_WEIGHT) -> WeightCalc:
    """
    Create a weight generator that avoids certain cells by adding a penalty to their weight.

    Args:
        weight (WeightCalc): A weight function for cells that are not avoided.
        avoid (Set[tuple[int, int]]): The cells to avoid.
        avoid_weight (float, optional):
            The weight penalty for avoided cells, this is added to the regular weight.
            Defaults to UNREACHABLE_WEIGHT.

    Returns:
        WeightCalc: A weight calculator based on the provided calculator that avoids the specified cells.
    """
    def weight_and_avoid(**kwargs: Unpack[WeightArgs]) -> float:
        """Calc weights, avoiding some cells.

        Returns:
            float: Weight.
        """
        penalty = avoid_weight if kwargs['cell'] in avoid else 0
        return weight(**kwargs) + penalty

    return weight_and_avoid


def calc_flood_fill(
        maze: ExtendedMaze,
        goals: Set[tuple[int, int]],
        weight: WeightCalc = simple_flood_weight,
        *,
        force: bool = False,
) -> None:
    """Calculates flood-fill weights *in place*.

    Args:
        maze (ExtendedMaze): The maze to flood.
        goals (Set[tuple[int, int]]): The goals to flood to.
        robot_pos (tuple[int, int]): The starting position for the flood.
        robot_direction (Direction): The starting direction for the flood.
        weight (WeightCalc, optional): The weight function for the flood. Defaults to simple_flood_weight.
        force (bool, optional): Force recalculation, even if the maze didn't change. Default False.
    """
    # Check ``force`` later to update the change status of the maze
    # Check for missing cells last because it is the worst complexity
    if not maze.changed() and not force and all(info.weight is not None for _, _, info in maze.iter_info()):
        return

    def _calc_weight(**kwargs: Unpack[WeightArgs]) -> float:
        try:
            return weight(**kwargs)
        except KeyError as kerr:
            if len(kerr.args) == 1 and kerr.args[0] in WeightArgs.__annotations__:
                raise TypeError(f"missing required keyword argument: {kerr.args[0]!r}") from kerr
            raise

    seen = set()
    current = goals
    marker = 0
    while True:
        for cell in current:
            info: ExtraCellInfo = maze.extra_info[cell]
            info.weight = _calc_weight(
                maze=maze,
                cell=cell,
                info=info,
                marker=marker,
            )
        marker += 1
        seen.update(current)
        current = adjacent_cells(maze, current, seen)
        if len(seen) >= maze.cell_count:
            break
        if not current:
            unreachable = {(r, c) for r, c, _ in maze} - seen
            print("unreachable:", unreachable)
            for cell in unreachable:
                maze.extra_info[cell].weight = UNREACHABLE_WEIGHT
            seen.update(unreachable)
            break  # unreachable cells detected
    assert len(seen) == maze.cell_count, f"new cells created ({len(seen)}/{maze.cell_count})"


def _do_nothing() -> None:
    pass


def single_flood_fill(  # pylint: disable=too-many-locals
        maze: ExtendedMaze,
        goals: Set[tuple[int, int]],
        *,
        weight: WeightCalc = simple_flood_weight,
        minor_priority: MinorPriority = shuffled,
        recalculate_flood: bool = True,
) -> Robot:
    """A robot that solves the maze using a simple implementation of the flood-fill algorithm.

    Args:
        maze (Maze): The maze.
        goals (Set[tuple[int, int]]): The goal cells.
        weight (WeightCalc, optional): The weight function. Defaults to simple_flood_weight.
        minor_priority (MinorPriority, optional):
            The last priority in selecting cells. Defaults to shuffled.
        recalculate_flood (bool, optional): Recalculate flood-fill weights every step (if the maze has changed). Defaults to True.

    Returns:
        Robot: The robot's brain.
    """
    def _priority(direction: Direction) -> tuple[float, int]:
        cell: ExtraCellInfo = maze.extra_info[dst := direction_to_cell(pos, direction)]
        assert cell.weight is not None, f"unweighted cell: {dst}"
        return cell.weight, cell.visited

    def _calc_flood_fill(force: bool = False) -> None:
        calc_flood_fill(
            maze=maze,
            goals=goals,
            weight=weight,
            force=force,
        )

    if recalculate_flood:
        _loop_calc_flood_fill = _calc_flood_fill
    else:
        _loop_calc_flood_fill = _do_nothing

    def _assert_turn(r: int, c: int, heading: Direction) -> Direction:
        assert (r, c) == pos, "moved while turning"
        assert maze[r, c] == walls, "walls changed while turning"
        return heading

    _calc_flood_fill(force=True)

    pos_row, pos_col, heading = yield Action.READY
    maze.route.append(pos := (pos_row, pos_col))

    while pos not in goals:
        maze.extra_info[pos_row, pos_col].visit_cell()
        _loop_calc_flood_fill()
        if math.isinf(maze.extra_info[pos_row, pos_col].weight or 0):
            print("floodmouse: cannot reach goals, giving up")
            break
        walls = maze[pos_row, pos_col]
        print(f"floodmouse: at {pos} facing {heading} with {walls}")
        new_direction = min(
            minor_priority(walls_to_directions(walls)),  # regular / reversed / shuffled
            key=_priority,
        )
        print(f"floodmouse: chose to flood {new_direction}")
        *turn_actions, action = abs_turn_to_actions(heading, new_direction, allow_reverse=False)
        assert action is Action.FORWARD, f"actions don't end with a step ({turn_actions=}, {action=})"
        assert set(turn_actions).issubset({Action.TURN_LEFT, Action.TURN_RIGHT}), f"not only turns({turn_actions})"
        for turn_action in turn_actions:
            print(f"floodmouse: turning {action.name.removeprefix('TURN_').lower()}")
            heading = _assert_turn(*(yield turn_action))
        assert heading == new_direction, "turning failed"
        pos_row, pos_col, heading = yield action
        maze.route.append(pos := (pos_row, pos_col))


def flood_fill_explore(
        maze: ExtendedMaze,
        goals: Set[tuple[int, int]],
        *,
        weight: WeightCalc = simple_flood_weight,
        minor_priority: MinorPriority = shuffled,
) -> Robot:
    """Explore the maze using the flood-fill algorithm.

    Args:
        maze (ExtendedMaze): The maze.
        goals (Set[tuple[int, int]]): The goal cells.
        weight (WeightCalc, optional): The weight function. Defaults to simple_flood_weight.
        minor_priority (MinorPriority, optional):
            The last priority in selecting cells. Defaults to shuffled.

    Returns:
        Robot: The robot's exploration brain.
    """
    # Initial flood-fill to the goal
    yield from single_flood_fill(
        maze,
        goals,
        weight=weight,
        minor_priority=minor_priority,
    )

    # Mark the starting point as the new goal and flood-fill to get there
    starting_pos = maze.route[0]
    maze.extra_info[starting_pos].color = "green"
    for goal in goals:
        maze.extra_info[goal].color = "blue"
    maze.mark_changed()
    yield from single_flood_fill(
        maze,
        {starting_pos},
        weight=weight,
        minor_priority=minor_priority,
    )


def flood_fill_robot(  # pylint: disable=too-many-arguments
        *,
        weight: WeightCalc = simple_flood_weight,
        minor_priority: MinorPriority = shuffled,
        final_weight: WeightCalc | None = None,
        final_minor_priority: MinorPriority | None = None,
        final_unknown_penalty: float = UNREACHABLE_WEIGHT,
) -> Algorithm:
    """A robot that solves the maze with only the flood

    Args:
        weight (WeightCalc, optional):
            The weight function for exploration. Defaults to simple_flood_weight.
        minor_priority (MinorPriority, optional):
            The priority tie-breaker for exploration. Defaults to shuffled.
        final_weight (WeightCalc | None, optional):
            The weight function for the final flood fill, if ``None``, ``weight`` is used. Defaults to None.
        final_minor_priority (MinorPriority | None, optional):
            The priority tie-breaker for the final flood fill, if ``None``, ``weight`` is used. Defaults to None.
        final_unknown_penalty (bool, optional):
            Weight penalty for unknown cells in the final flood fill run.
            The final run should be the optimal run so unknown cells should usually be avoided.
            Defaults to UNREACHABLE_WEIGHT which means "avoid at all costs".

    Returns:
        Robot: The robot's brain
    """
    if final_weight is None:
        final_weight = weight
    if final_minor_priority is None:
        final_minor_priority = minor_priority

    def _flood_fill_robot_impl(maze: ExtendedMaze, goals: Set[tuple[int, int]]) -> Robot:
        """A robot that solves the maze using a simple implementation of the flood-fill algorithm.

        Args:
            maze (Maze): The maze.
            goals (Set[tuple[int, int]]): The goal cells.

        Returns:
            Robot: The robot's brain.
        """
        yield from flood_fill_explore(
            maze,
            goals,
            weight=weight,
            minor_priority=minor_priority,
        )

        # Remember unknown cells so we can avoid them later
        unknown_cells = {
            (row, col)
            for row, col, info in maze.iter_info()
            if info.visited == 0
        }
        # Reset colors, route and orientation (but keep walls, we should be at the starting position)
        maze.reset_info()
        del maze.route
        yield Action.RESET
        # Do the actual fast route
        yield from single_flood_fill(
            maze,
            goals,
            weight=weight_with_avoid_cells(final_weight, unknown_cells, final_unknown_penalty),
            minor_priority=final_minor_priority,
            recalculate_flood=not math.isinf(final_unknown_penalty),
        )

    return _flood_fill_robot_impl


def simple_flood_fill(maze: ExtendedMaze, goals: Set[tuple[int, int]]) -> Robot:
    """A robot that solves the maze using a simple implementation of the flood-fill algorithm.

    Args:
        maze (Maze): The maze.
        goals (Set[tuple[int, int]]): The goal cells.

    Returns:
        Robot: The robot's brain.
    """
    return flood_fill_robot(minor_priority=reversed, final_unknown_penalty=0)(maze, goals)


def dijkstra_solver(
        maze: ExtendedMaze,
        goals: Set[tuple[int, int]],
        *,
        pos: RobotState | None = None,
        unknown_cells: Set[tuple[int, int]] = frozenset(),
) -> Robot:
    """A robot that solves the maze using dijkstra.

    THIS IS A SECOND STEP ROBOT!

    Args:
        maze (Maze): The maze.
        goals (Set[tuple[int, int]]): The goal cells.
        pos (RobotState | None, optional): The robot's current position. Defaults to None.
        unknown_cells (Set[tuple[int, int]], optional): The cells to avoid. Defaults to frozenset().

    Returns:
        Robot: The robot's brain.
    """
    forward_weight = 1  # basic unit
    reverse_weight = 2  # reverse is twice as slow as full speed ahead
    turn_weight = 1 + 2 + 1  # deceleration penalty + turn time + acceleration penalty

    if pos is None:
        # Remember unknown cells so we can avoid them later
        unknown_cells = {
            (row, col)
            for row, col, info in maze.iter_info()
            if info.visited == 0
        }

        # Reset colors, route and orientation (but keep walls, we should be at the starting position)
        maze.reset_info()
        del maze.route
        pos = yield Action.RESET

    shortest_routes = dijkstra(
        build_weighted_graph(
            maze,
            {
                RelativeDirection.FRONT: forward_weight,
                RelativeDirection.BACK: reverse_weight,
                RelativeDirection.LEFT: turn_weight,
                RelativeDirection.RIGHT: turn_weight,
            },
            start=pos,
            without=unknown_cells,
        ),
        pos[:-1],
    )

    for row, col, info in maze.iter_info():
        info.weight = shortest_routes.get((row, col), (math.inf, None))[0]
        info.color = 'red' if (row, col) in unknown_cells else None

    weight, best = min(  # there is at least 1 route
        (shortest_routes.get(goal, (math.inf, [])) for goal in goals),
        key=lambda weight_route: (weight_route[0], len(weight_route[1])),
    )

    assert math.isfinite(weight)
    assert best[0] == pos[:-1]
    assert best[-1] in goals

    # Do the actual fast route
    yield from predetermined_path_robot(
        maze,
        goals,
        path=best,
        initial_heading=pos[-1],
    )


def two_step_robot(
        maze: ExtendedMaze,
        goals: Set[tuple[int, int]],
        *,
        explorer: Algorithm = flood_fill_explore,
        solver: SolverAlgorithm = dijkstra_solver,
) -> Robot:
    """Combines 2 robots: one for exploration and another for finding an optimal path.

    Args:
        maze (Maze): The maze.
        goals (Set[tuple[int, int]]): The goal cells.
        explorer (Algorithm, optional): The robot to use for the exploration phase. Defaults to flood_fill_explore.
        solver (SolverAlgorithm, optional): The robot to use for the solution phase. Defaults to dijkstra_solver.

    Returns:
        Robot: The robot's brain.
    """
    yield from explorer(maze, goals)

    # Remember unknown cells so we can avoid them later
    unknown_cells = {
        (row, col)
        for row, col, info in maze.iter_info()
        if info.visited == 0
    }

    # Reset colors, route and orientation (but keep walls, we should be at the starting position)
    maze.reset_info()
    del maze.route
    pos = yield Action.RESET

    yield from solver(maze, goals, pos=pos, unknown_cells=unknown_cells)


def basic_weighted_flood_fill(maze: ExtendedMaze, goals: Set[tuple[int, int]]) -> Robot:
    """A robot that solves the maze using a simple implementation of the flood-fill algorithm.

    Args:
        maze (Maze): The maze.
        goals (Set[tuple[int, int]]): The goal cells.

    Returns:
        Robot: The robot's brain.
    """
    return two_step_robot(
        maze,
        goals,
        explorer=partial(
            flood_fill_explore,
            weight=simple_flood_weight,
            minor_priority=reversed,
        ),
    )


DEFAULT_DIJKSTRA_WEIGHTS: dict[RelativeDirection, float] = {
    RelativeDirection.FRONT: 1,
    RelativeDirection.BACK: 2,
    RelativeDirection.LEFT: 4,
    RelativeDirection.RIGHT: 4,
}


def _mark_deadends_flood(
        maze: ExtendedMaze,
        unknown_groups: Iterable[Set[tuple[int, int]]],
        goals: Set[tuple[int, int]],
        color: tuple[int, int, int] | str | None = 'yellow',
) -> list[set[tuple[int, int]]]:
    """Mark deadends based on flood-fill weights and set their color.

    Args:
        maze (ExtendedMaze): The maze.
        unknown_groups (Iterable[Set[tuple[int, int]]]): Groups of unexplored cells to check.
        goals (Set[tuple[int, int]]): The goals.
        color (tuple[int, int, int] | str | None, optional): The color to mark with. Defaults to 'yellow'.

    Returns:
        list[set[tuple[int, int]]]: A list containing the groups of dead-ends.
    """
    # Clone the maze to avoid polluting the original maze's caches
    tmp_maze = ExtendedMaze.full_from_maze(maze)
    # print(tmp_maze.render_extra(pos=pos + (Direction.NORTH_EAST,), goals=goals, weights=False))

    # Calculate the unbiased flood-fill weights.
    calc_flood_fill(tmp_maze, goals)

    def _check_group(group: Set[tuple[int, int]]) -> bool:
        for cell in group:
            cell_info: ExtraCellInfo = tmp_maze.extra_info[cell]
            assert cell_info.weight is not None
            for adj in adjacent_cells(maze, [cell], group):
                adj_info: ExtraCellInfo = tmp_maze.extra_info[adj]
                assert adj_info.weight is not None
                if cell_info.weight <= adj_info.weight:
                    return False
        return True

    dead_ends_groups: list[set[tuple[int, int]]] = []
    for unknown in unknown_groups:
        dead_end_group: set[tuple[int, int]] = set()
        # Skip the goals
        if unknown & goals:
            continue
        if _check_group(unknown):
            for cell in unknown:
                info: ExtraCellInfo = maze.extra_info[cell]
                info.visited = 1
                info.color = color
                dead_end_group.add(cell)

            dead_ends_groups.append(dead_end_group)

    return dead_ends_groups


def _calc_unknown_groups(  # pylint: disable=too-many-arguments
        maze: ExtendedMaze,
        pos: tuple[int, int],
        start: tuple[int, int],
        goals: Set[tuple[int, int]],
        unknown_color: tuple[int, int, int] | str | None = 'blue',
        deadend_color: tuple[int, int, int] | str | None = 'orange',
) -> tuple[UnionFind[tuple[int, int]], set[tuple[int, int]], list[set[tuple[int, int]]]]:
    def in_goal(*cells: tuple[int, int]) -> bool:
        return all(cell in goals for cell in cells)

    if deadend_color is not None:
        mark_deadends(maze, pos, start, goals, deadend_color)

    groups: UnionFind[tuple[int, int]] = UnionFind()
    for row, col, walls, info in maze.iter_all():
        if info.visited > 0:
            info.reset_color_if(unknown_color)
            continue
        called_union = False
        for missing in ~walls:
            match missing:
                case Walls.NORTH:
                    adjacent = (row - 1, col)
                case Walls.EAST:
                    adjacent = (row, col + 1)
                case Walls.SOUTH:
                    adjacent = (row + 1, col)
                case Walls.WEST:
                    adjacent = (row, col - 1)
                case _:
                    raise AssertionError("unexpected wall")
            if maze.extra_info[adjacent].visited == 0 and not in_goal((row, col), adjacent):
                groups.union((row, col), adjacent)
                called_union = True
        if called_union:
            if info.color is None and unknown_color is not None:
                info.color = unknown_color
        else:
            info.reset_color_if(unknown_color)
            info.visited = 1

    return (
        groups,
        reduce(or_, groups.iter_sets(), set()),
        _mark_deadends_flood(maze, groups.iter_sets(), goals, deadend_color) if deadend_color is not None else [],
    )


def dijkstra_navigator(  # pylint: disable=too-many-arguments
        maze: ExtendedMaze,
        dest: tuple[int, int],
        pos: RobotState,
        *,
        name: str = "dijkstra navigator",
        action: str = "navigating",
        reset_colors: bool = True,
) -> Robot:
    """Navigate to a specific cell with Dijkstra's algorithm.

    This is meant as a tool for other robots and does not yield the READY action.

    Args:
        maze (ExtendedMaze): The maze.
        dest (tuple[int, int]): The destination cell.
        pos (RobotState): The robot's current position + heading.
        name (str, optional): The primary robot's name (for prints). Defaults to "dijkstra navigator".
        action (str, optional): The action's name (for prints). Defaults to "navigating".
        reset_colors (bool, optional): If True, also reset the color of visited cells. Defaults to True.

    Returns:
        Robot: The robot's brain.
    """
    _ = maze.changed()  # Consume the change marker
    while pos[:-1] != dest:
        routes = dijkstra(
            build_weighted_graph(maze, DEFAULT_DIJKSTRA_WEIGHTS, start=pos),
            pos[:-1],
            goals={dest},
        )
        print(f"{name}: @{pos} -> {routes[dest][1]}")
        assert routes[dest][1][0] == pos[:-1], f"robot is at {pos[:-1]} but route starts at {routes[dest][1][0]}"
        return_bot = predetermined_path_robot(
            maze,
            {dest},
            path=routes[dest][1],
            initial_heading=pos[-1],
        )
        assert next(return_bot, None) is Action.READY
        pos = yield Action.READY
        while True:
            if reset_colors:
                maze.extra_info[pos[:-1]].color = None
            try:
                pos = yield return_bot.send(pos)
            except StopIteration:
                assert pos[:-1] == dest
                break
            if maze.changed():
                print(f"{name}: encountered a wall while {action}")
                break  # recalculate route, a new wall was added


def flood_fill_thorough_explorer(  # pylint: disable=too-many-branches,too-many-statements,too-many-locals
        maze: ExtendedMaze,
        goals: Set[tuple[int, int]],
        *,
        percentage: float = 1.0,
        flood_weight: WeightCalc = simple_flood_weight,
        minor_priority: MinorPriority = identity,
) -> Robot:
    """A robot that explores the maze using the flood-fill algorithm.

    THIS IS MEANT AS A FIRST STEP ROBOT!

    Args:
        maze (ExtendedMaze): The maze.
        goals (Set[tuple[int, int]]): The final goals.
        percentage (float, optional): The percentage of the maze to explore before returning home. Defaults to 1.0.
        flood_weight (WeightCalc, optional): Exploration weight. Defaults to simple_flood_weight.
        minor_priority (MinorPriority, optional): Exploration minor priority. Defaults to identity.

    Returns:
        Robot: The robot's brain.
    """
    if not 0 < percentage <= 1.0:
        raise ValueError(f"invalid percentage: {percentage!r}")

    # First, find the goal
    # We cannot use ``yield from`` because we need the final reply from the yield
    print(f"flood hunter: looking for {goals=}")
    flood_bot = single_flood_fill(
        maze,
        goals,
        weight=flood_weight,
        minor_priority=minor_priority,
    )
    assert next(flood_bot, None) is Action.READY
    start_pos = pos = yield Action.READY
    while True:
        try:
            pos = yield flood_bot.send(pos)
        except StopIteration:
            break
        mark_deadends(maze, pos[:-1], start_pos[:-1], goals, 'orange')

    print(f"flood hunter: found goals! {start_pos[:-1]=}, {maze.explored_cells_percentage()=:.02%}/{percentage=:.02%}")
    unknown_color = 'blue'
    dead_ends_groups: list[set[tuple[int, int]]] = []
    while maze.explored_cells_percentage() < percentage:
        print(f"flood hunter: pos={tuple(pos)}, {maze.explored_cells_percentage()=:.02%}/{percentage=:.02%}")
        # print(_render_maze(maze, goals=goals, pos=pos))
        mark_unreachable_groups(maze, pos[:-1])

        unknown, all_unknown, flood_dead_ends = _calc_unknown_groups(maze, pos[:-1], start_pos[:-1], goals, unknown_color=unknown_color)
        # Once marked as a deadend, the cell is explored and won't be rediscovered as a deadend so no duplicates:
        dead_ends_groups += flood_dead_ends

        # Check that the fastest path does not go through a dead-end
        fastest_routes = dijkstra(
            build_weighted_graph(
                maze,
                DEFAULT_DIJKSTRA_WEIGHTS,
                start=start_pos,
            ),
            src=start_pos[:-1],
            goals=goals,
        )

        fastest_path_cells = set(min(  # there is at least 1 route
            (fastest_routes.get(goal, (math.inf, [])) for goal in goals),
            key=lambda weight_route: (weight_route[0], len(weight_route[1])),
        )[1])

        to_remove: list[int] = []
        for i, dead_end_group in enumerate(dead_ends_groups):
            if fastest_path_cells & dead_end_group:
                first_time = True
                for cell in dead_end_group:
                    info: ExtraCellInfo = maze.extra_info[cell]
                    info.visited = 0
                    info.color = unknown_color
                    all_unknown.add(cell)
                    if first_time:
                        dead_end_union_group = unknown.find(cell)
                        first_time = False
                    else:
                        unknown.union(dead_end_union_group, cell)

                to_remove.append(i)

        for i in reversed(to_remove):
            dead_ends_groups.pop(i)

        routes = dijkstra(
            build_weighted_graph(maze, DEFAULT_DIJKSTRA_WEIGHTS, start=pos),
            pos[:-1],
            goals=all_unknown,
        )

        potential_routes = {
            # The start gets 'inf' so that is is chosen last (reaching the start ends the exploration)
            best[1]: (best[0] / len(group)) if start_pos[:-1] not in group else math.inf
            for group in unknown
            if math.isfinite((best := max((routes.get(cell, (math.inf, []))[0], cell) for cell in group))[0])
        }

        # print(f"flood hunter: {potential_routes=}")

        if not potential_routes:
            # Nothing to do
            print(f"flood hunter: {routes=}")
            break

        dest = min(potential_routes, key=potential_routes.__getitem__)
        maze.extra_info[dest].color = 'green'
        print(f"flood hunter: hunting {dest}")

        flood_bot = single_flood_fill(
            maze,
            {dest},
            weight=simple_flood_weight_with_norm_visit_bias,
            minor_priority=minor_priority,
        )
        assert next(flood_bot, None) is Action.READY
        while True:
            # print(maze.render_extra(goals=goals, pos=pos))
            if math.isinf(maze.extra_info[pos[:-1]].weight):
                print("flood hunter: dest is unreachable")
                break
            if dest not in all_unknown:
                print("flood hunter: dest discovered without visiting")
                break
            try:
                pos = yield flood_bot.send(pos)
            except StopIteration:
                print(f"flood hunter: reached dest - {pos=}")
                break
            _, all_unknown, flood_dead_ends = _calc_unknown_groups(
                maze, pos[:-1],
                start_pos[:-1],
                goals,
                unknown_color=unknown_color,
            )
            dead_ends_groups += flood_dead_ends

        maze.extra_info[dest].reset_color_if('green')

    print(f"flood hunter: done exploring - {maze.explored_cells_percentage()=:.02%}/{percentage=:.02%}")
    yield from dijkstra_navigator(
        maze,
        start_pos[:-1],
        pos,
        name="flood hunter",
        action="going home",
    )


def thorough_flood_fill(maze: ExtendedMaze, goals: Set[tuple[int, int]]) -> Robot:
    """A robot that solves the maze using an advanced implementation of the flood-fill algorithm and some dijkstra.

    Returns:
        Robot: The robot's brain.
    """
    return two_step_robot(
        maze,
        goals,
        explorer=partial(
            flood_fill_thorough_explorer,
            flood_weight=simple_flood_weight_with_strong_visit_bias,
        ),
    )


def flood_fill_dijkstra_explorer(  # pylint: disable=too-many-locals,too-many-branches,too-many-statements
        maze: ExtendedMaze,
        goals: Set[tuple[int, int]],
        *,
        route_give_up_limit: int | None = 6,
        flood_weight: WeightCalc = simple_flood_weight,
        minor_priority: MinorPriority = identity,
) -> Robot:
    """A robot that explores the maze using a mix of the flood-fill algorithm and Dijkstra's algorithm.

    THIS IS MEANT AS A FIRST STEP ROBOT!

    Args:
        maze (ExtendedMaze): The maze.
        goals (Set[tuple[int, int]]): The final goals.
        route_give_up_limit (int | None, optional): Specifies after how many actions the robot should recalculate
            the route it explores, ``None`` means "never". Defaults to 10.
        flood_weight (WeightCalc, optional): Exploration weight. Defaults to simple_flood_weight.
        minor_priority (MinorPriority, optional): Exploration minor priority. Defaults to identity.

    Returns:
        Robot: The robot's brain.
    """
    timer = GiveUpTimer(limit=route_give_up_limit)
    explorer = partial(
        single_flood_fill,
        maze,
        weight=flood_weight,
        minor_priority=minor_priority,
    )

    # First, find the goal
    # We cannot use ``yield from`` because we need the final reply from the yield
    print(f"dijkstra hunter: looking for {goals=}")
    flood_bot = explorer(goals)
    assert next(flood_bot, None) is Action.READY
    start_pos = pos = yield Action.READY
    while True:
        try:
            pos = yield flood_bot.send(pos)
        except StopIteration:
            break
        # mark_deadends(maze, pos[:-1], start_pos[:-1], goals, 'orange')

    print(f"dijkstra hunter: found goals! (explored {maze.explored_cells_percentage()=:.02%})")

    while maze.explored_cells_percentage() < 1.0:
        # Find the best routes
        routes = dijkstra(
            build_weighted_graph(
                maze,
                DEFAULT_DIJKSTRA_WEIGHTS,
                start=start_pos,
            ),
            src=start_pos[:-1],
            goals=goals,
        )

        weight, best = min(  # there is at least 1 route
            (routes.get(goal, (math.inf, [])) for goal in goals),
            key=lambda weight_route: (weight_route[0], len(weight_route[1])),
        )

        assert math.isfinite(weight)
        assert best[0] == start_pos[:-1]
        assert best[-1] in goals

        # It's impossible to discover a wall between an unknown cell and a known cell,
        # we can always assume that a newly encountered wall is between an unknown cell
        # and the current cell.
        cells = {cell: i for i, cell in enumerate(best)}
        _, all_unknown, _ = _calc_unknown_groups(maze, pos[:-1], start_pos[:-1], goals, deadend_color=None)
        missing_cells = all_unknown & set(cells)
        if not missing_cells:
            break

        for cell in best:
            maze.extra_info[cell].color = 'green' if cell in missing_cells else 'purple'

        def _bad_wall(cell: tuple[int, int]) -> bool:
            if (cell_idx := cells.get(cell, None)) is None:
                return False

            adj_cells = frozenset(adjacent_cells(maze, (cell,)))
            return (
                (cell_idx > 0 and best[cell_idx - 1] not in adj_cells) or
                (cell_idx < len(best) - 1 and best[cell_idx + 1] not in adj_cells)
            )

        timer.reset()
        while missing_cells and timer:
            flood_bot = explorer(missing_cells)
            assert next(flood_bot, None) is Action.READY

            while timer:
                try:
                    pos = yield flood_bot.send(pos)
                except StopIteration:
                    break

                _, all_unknown, _ = _calc_unknown_groups(maze, pos[:-1], start_pos[:-1], goals, deadend_color=None)
                timer.update()

                if updated_cells := missing_cells - all_unknown:
                    for cell in updated_cells:
                        # print(f"dijkstra hunter: explored {cell}")
                        maze.extra_info[cell].color = 'purple'

                        # Check if we found a wall that cuts the path (no need if the timer started)
                        if timer.stopped and _bad_wall(cell):
                            timer.start()
                            break

                    missing_cells &= all_unknown

                    # Restart the explorer with a new goal
                    flood_bot.close()
                    flood_bot = explorer(missing_cells)
                    assert next(flood_bot, None) is Action.READY
            else:
                # Stop the robot
                flood_bot.close()

        for cell in best:
            maze.extra_info[cell].reset_color_if('purple')
            maze.extra_info[cell].reset_color_if('green')
    else:
        print("dijkstra hunter: explored the entire maze")

    print(f"dijkstra hunter: done exploring - {maze.explored_cells_percentage()=:.02%}")
    yield from dijkstra_navigator(
        maze,
        start_pos[:-1],
        pos,
        name="dijkstra hunter",
        action="going home",
    )


def dijkstra_flood_fill(maze: ExtendedMaze, goals: Set[tuple[int, int]]) -> Robot:
    """A robot that solves the maze using an advanced implementation of the flood-fill algorithm and some dijkstra.

    Returns:
        Robot: The robot's brain.
    """
    return two_step_robot(
        maze,
        goals,
        explorer=flood_fill_dijkstra_explorer,
    )
