"""Absolute and relative directions in the maze

This module contains a classes for representing directions in the maze.
"""

from __future__ import annotations

import math

from enum import auto, Enum, IntEnum
from typing import overload, TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Literal


class RelativeDirection(Enum):
    """Relative directions."""
    FRONT = auto()
    BACK = auto()
    LEFT = auto()
    RIGHT = auto()

    @overload
    def invert(self: Literal[RelativeDirection.FRONT]) -> Literal[RelativeDirection.BACK]: ...
    @overload
    def invert(self: Literal[RelativeDirection.BACK]) -> Literal[RelativeDirection.FRONT]: ...
    @overload
    def invert(self: Literal[RelativeDirection.LEFT]) -> Literal[RelativeDirection.RIGHT]: ...
    @overload
    def invert(self: Literal[RelativeDirection.RIGHT]) -> Literal[RelativeDirection.LEFT]: ...
    @overload
    def invert(self) -> RelativeDirection: ...

    def invert(self) -> RelativeDirection:
        """Invert the direction (left <-> right; front <-> back)

        Returns:
            RelativeDirection: The inverted direction.
        """
        match self:
            case RelativeDirection.FRONT: return RelativeDirection.BACK
            case RelativeDirection.BACK: return RelativeDirection.FRONT
            case RelativeDirection.LEFT: return RelativeDirection.RIGHT
            case RelativeDirection.RIGHT: return RelativeDirection.LEFT


class Direction(IntEnum):
    """Bit masks for the directions."""
    NORTH = 0x1
    EAST = 0x2
    SOUTH = 0x4
    WEST = 0x8
    NORTH_EAST = NORTH | EAST
    NORTH_WEST = NORTH | WEST
    SOUTH_EAST = SOUTH | EAST
    SOUTH_WEST = SOUTH | WEST

    def turn_left(self) -> Direction:  # pylint: disable=too-many-return-statements
        """Return the direction that is the result of turning left (90 degrees counter-clockwise).

        Returns:
            Direction: The result of turning left.
        """
        match self:
            case Direction.NORTH: return Direction.WEST
            case Direction.EAST: return Direction.NORTH
            case Direction.SOUTH: return Direction.EAST
            case Direction.WEST: return Direction.SOUTH
            case Direction.NORTH_EAST: return Direction.NORTH_WEST
            case Direction.NORTH_WEST: return Direction.SOUTH_WEST
            case Direction.SOUTH_EAST: return Direction.NORTH_EAST
            case Direction.SOUTH_WEST: return Direction.SOUTH_EAST

    def turn_right(self) -> Direction:  # pylint: disable=too-many-return-statements
        """Calculate the direction that is the result of turning right (90 degrees clockwise).

        Returns:
            Direction: The result of turning right.
        """
        match self:
            case Direction.NORTH: return Direction.EAST
            case Direction.EAST: return Direction.SOUTH
            case Direction.SOUTH: return Direction.WEST
            case Direction.WEST: return Direction.NORTH
            case Direction.NORTH_EAST: return Direction.SOUTH_EAST
            case Direction.NORTH_WEST: return Direction.NORTH_EAST
            case Direction.SOUTH_EAST: return Direction.SOUTH_WEST
            case Direction.SOUTH_WEST: return Direction.NORTH_WEST

    def turn_back(self) -> Direction:  # pylint: disable=too-many-return-statements
        """Calculate the direction that is the result of turning back (180 degrees).

        Returns:
            Direction: The result of turning back.
        """
        match self:
            case Direction.NORTH: return Direction.SOUTH
            case Direction.EAST: return Direction.WEST
            case Direction.SOUTH: return Direction.NORTH
            case Direction.WEST: return Direction.EAST
            case Direction.NORTH_EAST: return Direction.SOUTH_WEST
            case Direction.NORTH_WEST: return Direction.SOUTH_EAST
            case Direction.SOUTH_EAST: return Direction.NORTH_WEST
            case Direction.SOUTH_WEST: return Direction.NORTH_EAST

    def half_turn_left(self) -> Direction:  # pylint: disable=too-many-return-statements
        """Calculate the direction that is the result of a half turn left (45 degrees counter-clockwise).

        Returns:
            Direction: The result of a half turn left.
        """
        match self:
            case Direction.NORTH: return Direction.NORTH_WEST
            case Direction.EAST: return Direction.NORTH_EAST
            case Direction.SOUTH: return Direction.SOUTH_EAST
            case Direction.WEST: return Direction.SOUTH_WEST
            case Direction.NORTH_EAST: return Direction.NORTH
            case Direction.NORTH_WEST: return Direction.WEST
            case Direction.SOUTH_EAST: return Direction.EAST
            case Direction.SOUTH_WEST: return Direction.SOUTH

    def half_turn_right(self) -> Direction:  # pylint: disable=too-many-return-statements
        """Calculate the direction that is the result of a half turn right (45 degrees clockwise).

        Returns:
            Direction: The result of a half turn right.
        """
        match self:
            case Direction.NORTH: return Direction.NORTH_EAST
            case Direction.EAST: return Direction.SOUTH_EAST
            case Direction.SOUTH: return Direction.SOUTH_WEST
            case Direction.WEST: return Direction.NORTH_WEST
            case Direction.NORTH_EAST: return Direction.EAST
            case Direction.NORTH_WEST: return Direction.NORTH
            case Direction.SOUTH_EAST: return Direction.SOUTH
            case Direction.SOUTH_WEST: return Direction.WEST

    def turn(self, rel: RelativeDirection) -> Direction:
        """Calculate the direction that is the result of turning in the provided relative direction.

        Args:
            rel (RelativeDirection): The direction to turn to.

        Returns:
            Direction: The result of the turn.
        """
        match rel:
            case RelativeDirection.FRONT: return self
            case RelativeDirection.BACK: return self.turn_back()
            case RelativeDirection.LEFT: return self.turn_left()
            case RelativeDirection.RIGHT: return self.turn_right()

    def half_turn(self, rel: RelativeDirection) -> Direction:
        """Calculate the direction that is the result of half a turn in the provided relative direction.

        Args:
            rel (RelativeDirection): The direction to turn to.

        Returns:
            Direction: The result of the turn.
        """
        match rel:
            case RelativeDirection.FRONT: return self
            case RelativeDirection.BACK: raise ValueError("cannot half turn back")
            case RelativeDirection.LEFT: return self.half_turn_left()
            case RelativeDirection.RIGHT: return self.half_turn_right()

    def to_degrees(self) -> Literal[0, 45, 90, 135, 180, 225, 270, 315]:  # pylint: disable=too-many-return-statements
        """Get the rotation degrees. EAST is 0 deg, degrees increase clockwise.

        Returns:
            Literal[0, 45, 90, 135, 180, 225, 270, 315]: The rotation degree.
        """
        match self:
            case Direction.EAST: return 0
            case Direction.SOUTH_EAST: return 45
            case Direction.SOUTH: return 90
            case Direction.SOUTH_WEST: return 135
            case Direction.WEST: return 180
            case Direction.NORTH_WEST: return 225
            case Direction.NORTH: return 270
            case Direction.NORTH_EAST: return 315

    def to_radians(self) -> float:
        """Get the rotation radians. EAST is 0, angles increase clockwise.

        Returns:
            float: The rotation angle in radians.
        """
        return math.radians(self.to_degrees())

    @staticmethod
    def from_str(direction: str) -> Direction:  # pylint: disable=too-many-return-statements
        """Create from a direction name.
        The direction name is case insensitive and can be either a full name or
        an abbreviation.

        Args:
            direction (str): A string representing a cardinal direction.

        Raises:
            ValueError: The ``direction`` string is invalid.

        Returns:
            Direction: The direction represented by the string.
        """
        match direction.strip().casefold():
            case 'north' | 'n':
                return Direction.NORTH
            case 'east' | 'e':
                return Direction.EAST
            case 'south' | 's':
                return Direction.SOUTH
            case 'west' | 'w':
                return Direction.WEST
            case 'north_east' | 'north east' | 'ne':
                return Direction.NORTH_EAST
            case 'north_west' | 'north west' | 'nw':
                return Direction.NORTH_WEST
            case 'south_east' | 'south east' | 'se':
                return Direction.SOUTH_EAST
            case 'south_west' | 'south west' | 'sw':
                return Direction.SOUTH_WEST
        raise ValueError(f"{direction!r} is not a valid Direction")

    def __str__(self) -> str:
        return self.name


PRIMARY_DIRECTIONS = frozenset([
    Direction.NORTH,
    Direction.EAST,
    Direction.SOUTH,
    Direction.WEST,
])

SECONDARY_DIRECTIONS = frozenset([
    Direction.NORTH_EAST,
    Direction.NORTH_WEST,
    Direction.SOUTH_EAST,
    Direction.SOUTH_WEST,
])
