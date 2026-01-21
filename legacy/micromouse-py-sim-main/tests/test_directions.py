# pylint: disable=missing-function-docstring,missing-module-docstring
from __future__ import annotations

import math

from typing import Callable

import pytest

from sim.directions import Direction, RelativeDirection


__RELATIVE_DIRECTION_PAIRS: frozenset[tuple[RelativeDirection, RelativeDirection]] = frozenset({
    (RelativeDirection.FRONT, RelativeDirection.BACK),
    (RelativeDirection.LEFT, RelativeDirection.RIGHT),
})

__INVERTED_DIRECTIONS = dict(__RELATIVE_DIRECTION_PAIRS) | {
    b: a for a, b in __RELATIVE_DIRECTION_PAIRS
}


@pytest.mark.parametrize("direction", RelativeDirection)
def test_invert_relative_direction(direction: RelativeDirection):
    inverted = direction.invert()
    assert inverted != direction, "invert() did not change direction"
    assert inverted == __INVERTED_DIRECTIONS[direction], "invert() did not change direction"
    assert inverted.invert() == direction, "invert() twice did not return to the original direction"


@pytest.mark.parametrize("steps,turn", [
    pytest.param(2, Direction.turn_back, id="back"),
    pytest.param(4, Direction.turn_left, id="left"),
    pytest.param(4, Direction.turn_right, id="right"),
    pytest.param(8, Direction.half_turn_left, id="half left"),
    pytest.param(8, Direction.half_turn_right, id="half right"),
])
@pytest.mark.parametrize("direction", Direction)
def test_direction_turn_cycle(steps: int, turn: Callable[[Direction], Direction], direction: Direction):
    seen = set()
    curr = direction
    for i in range(steps):
        assert curr not in seen, f"cycle ended too quickly (after {i} steps, stepped twice in {curr}, {seen=})"
        seen.add(curr)
        curr = turn(curr)
    assert direction == curr, f"cycle didn't end after {steps} steps (started at {direction}, got to {curr}, {seen=})"
    assert len(seen) == steps, f"not enough unique steps: expected {steps}, actual {len(seen)}, {seen=}"


@pytest.mark.parametrize("rel,expected", [
    pytest.param(RelativeDirection.FRONT, Direction, id="front"),
    pytest.param(RelativeDirection.BACK, Direction.turn_back, id="back"),
    pytest.param(RelativeDirection.LEFT, Direction.turn_left, id="left"),
    pytest.param(RelativeDirection.RIGHT, Direction.turn_right, id="right"),
])
@pytest.mark.parametrize("direction", Direction)
def test_direction_turn_relative(rel: RelativeDirection, expected: Callable[[Direction], Direction], direction: Direction):
    assert direction.turn(rel) == expected(direction)


@pytest.mark.parametrize("rel,expected", [
    pytest.param(RelativeDirection.FRONT, Direction, id="front"),
    pytest.param(RelativeDirection.LEFT, Direction.half_turn_left, id="left"),
    pytest.param(RelativeDirection.RIGHT, Direction.half_turn_right, id="right"),
])
@pytest.mark.parametrize("direction", Direction)
def test_direction_half_turn_relative(rel: RelativeDirection, expected: Callable[[Direction], Direction], direction: Direction):
    assert direction.half_turn(rel) == expected(direction)


@pytest.mark.parametrize("direction", Direction)
def test_direction_half_turn_back(direction: Direction):
    with pytest.raises(ValueError):
        direction.half_turn(RelativeDirection.BACK)


@pytest.mark.parametrize("direction,degrees", [
    (Direction.EAST, 0),
    (Direction.SOUTH_EAST, 45),
    (Direction.SOUTH, 90),
    (Direction.SOUTH_WEST, 135),
    (Direction.WEST, 180),
    (Direction.NORTH_WEST, 225),
    (Direction.NORTH, 270),
    (Direction.NORTH_EAST, 315),
])
def test_direction_degrees(direction: Direction, degrees: int):
    assert direction.to_degrees() == degrees
    assert direction.to_radians() == math.radians(degrees)


@pytest.mark.parametrize("direction", Direction)
def test_direction_from_str(direction: Direction):
    assert Direction.from_str(direction.name) == direction
    assert Direction.from_str(direction.name.lower()) == direction
    assert Direction.from_str(direction.name.upper()) == direction
    spaced_name = direction.name.replace('_', ' ')
    assert Direction.from_str(spaced_name.lower()) == direction
    assert Direction.from_str(spaced_name.upper()) == direction
    abbrev = ''.join(part[0] for part in direction.name.split('_'))
    assert Direction.from_str(abbrev.lower()) == direction
    assert Direction.from_str(abbrev.upper()) == direction


@pytest.mark.parametrize("direction", [
    # Completely wrong
    '',
    'x',
    'q',
    # Opposing directions
    'north south', 'ns',
    'south_north', 'sn',
    'east west', 'ew',
    'west_east', 'we',
    'nws', 'nwe',
    # Wrong order
    'wn',
    'east south',
])
def test_direction_from_bad_str(direction: str):
    with pytest.raises(ValueError):
        Direction.from_str(direction)
