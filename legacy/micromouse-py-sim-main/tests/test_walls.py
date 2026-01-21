# pylint: disable=missing-function-docstring,missing-module-docstring
from __future__ import annotations

from itertools import chain

import pytest

from sim.maze import Walls


@pytest.mark.parametrize("walls,byte", [
    pytest.param(Walls(0), b'\x00', id="empty"),
    pytest.param(Walls.NORTH, b'\x01', id="north"),
    pytest.param(Walls.EAST, b'\x02', id="east"),
    pytest.param(Walls.SOUTH, b'\x04', id="south"),
    pytest.param(Walls.WEST, b'\x08', id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, b'\x03', id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, b'\x05', id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, b'\x09', id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, b'\x06', id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, b'\x0a', id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, b'\x0c', id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, b'\x07', id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, b'\x0b', id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, b'\x0d', id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, b'\x0e', id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, b'\x0f', id="all"),
])
def test_bytes_conversions(walls: Walls, byte: bytes):
    assert bytes(walls) == walls.to_bytes() == byte
    assert Walls.from_bytes(byte) == walls


def test_from_bytes_too_long():
    with pytest.raises(ValueError):
        Walls.from_bytes(b'\x0a\x04')


def test_from_bytes_empty():
    with pytest.raises(ValueError):
        Walls.from_bytes(b'')


def test_none():
    none = Walls.none()
    for wall in Walls:
        assert wall not in none
    assert Walls.none() == Walls(0)


def test_all():
    all_ = Walls.all()
    for wall in Walls:
        assert wall in all_
    assert all_ == Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST


@pytest.mark.parametrize("walls,rotated", [
    pytest.param(Walls(0), Walls.none(), id="empty"),
    pytest.param(Walls.NORTH, Walls.EAST, id="north"),
    pytest.param(Walls.EAST, Walls.SOUTH, id="east"),
    pytest.param(Walls.SOUTH, Walls.WEST, id="south"),
    pytest.param(Walls.WEST, Walls.NORTH, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, Walls.EAST | Walls.SOUTH, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, Walls.EAST | Walls.WEST, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, Walls.NORTH | Walls.EAST, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, Walls.SOUTH | Walls.WEST, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, Walls.NORTH | Walls.SOUTH, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, Walls.NORTH | Walls.WEST, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, ~Walls.NORTH, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, ~Walls.WEST, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, ~Walls.SOUTH, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, ~Walls.EAST, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, Walls.all(), id="all"),
])
def test_rotate_right(walls: Walls, rotated: Walls):
    assert walls.rotate_right() == rotated
    assert walls.rotate_right(n=1) == rotated
    assert walls.rotate_left(n=3) == rotated
    assert walls.rotate_left(n=-1) == rotated


@pytest.mark.parametrize("walls,rotated", [
    pytest.param(Walls(0), Walls.none(), id="empty"),
    pytest.param(Walls.NORTH, Walls.WEST, id="north"),
    pytest.param(Walls.EAST, Walls.NORTH, id="east"),
    pytest.param(Walls.SOUTH, Walls.EAST, id="south"),
    pytest.param(Walls.WEST, Walls.SOUTH, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, Walls.NORTH | Walls.WEST, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, Walls.EAST | Walls.WEST, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, Walls.SOUTH | Walls.WEST, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, Walls.NORTH | Walls.EAST, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, Walls.NORTH | Walls.SOUTH, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, Walls.EAST | Walls.SOUTH, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, ~Walls.SOUTH, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, ~Walls.EAST, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, ~Walls.NORTH, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, ~Walls.WEST, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, Walls.all(), id="all"),
])
def test_rotate_left(walls: Walls, rotated: Walls):
    assert walls.rotate_left() == rotated
    assert walls.rotate_left(n=1) == rotated
    assert walls.rotate_right(n=3) == rotated
    assert walls.rotate_right(n=-1) == rotated


@pytest.mark.parametrize("walls,rotated", [
    pytest.param(Walls(0), Walls.none(), id="empty"),
    pytest.param(Walls.NORTH, Walls.SOUTH, id="north"),
    pytest.param(Walls.EAST, Walls.WEST, id="east"),
    pytest.param(Walls.SOUTH, Walls.NORTH, id="south"),
    pytest.param(Walls.WEST, Walls.EAST, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, Walls.SOUTH | Walls.WEST, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, Walls.NORTH | Walls.SOUTH, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, Walls.EAST | Walls.SOUTH, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, Walls.NORTH | Walls.WEST, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, Walls.EAST | Walls.WEST, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, Walls.NORTH | Walls.EAST, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, ~Walls.EAST, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, ~Walls.NORTH, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, ~Walls.WEST, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, ~Walls.SOUTH, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, Walls.all(), id="all"),
])
def test_rotate_180(walls: Walls, rotated: Walls):
    assert walls.rotate_right(n=2) == rotated
    assert walls.rotate_left(n=2) == rotated


@pytest.mark.parametrize("walls", [
    pytest.param(Walls(0), id="empty"),
    pytest.param(Walls.NORTH, id="north"),
    pytest.param(Walls.EAST, id="east"),
    pytest.param(Walls.SOUTH, id="south"),
    pytest.param(Walls.WEST, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, id="all"),
])
def test_wraparound(walls: Walls):
    right = walls.rotate_right()
    flip = walls.rotate_right(2)
    left = walls.rotate_left()
    for i in chain((-1000,), range(-10, 10), (1000,)):
        assert walls.rotate_right(4 * i) == walls
        assert walls.rotate_right(4 * i + 1) == right
        assert walls.rotate_right(4 * i + 2) == flip
        assert walls.rotate_right(4 * i + 3) == left
        assert walls.rotate_left(4 * i) == walls
        assert walls.rotate_left(4 * i + 1) == left
        assert walls.rotate_left(4 * i + 2) == flip
        assert walls.rotate_left(4 * i + 3) == right


@pytest.mark.parametrize("walls,transposed", [
    pytest.param(Walls(0), Walls.none(), id="empty"),
    pytest.param(Walls.NORTH, Walls.WEST, id="north"),
    pytest.param(Walls.EAST, Walls.SOUTH, id="east"),
    pytest.param(Walls.SOUTH, Walls.EAST, id="south"),
    pytest.param(Walls.WEST, Walls.NORTH, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, Walls.SOUTH | Walls.WEST, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, Walls.EAST | Walls.WEST, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, Walls.NORTH | Walls.WEST, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, Walls.EAST | Walls.SOUTH, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, Walls.NORTH | Walls.SOUTH, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, Walls.NORTH | Walls.EAST, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, ~Walls.NORTH, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, ~Walls.EAST, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, ~Walls.SOUTH, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, ~Walls.WEST, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, Walls.all(), id="all"),
])
def test_transpose(walls: Walls, transposed: Walls):
    assert walls.transpose() == transposed


@pytest.mark.parametrize("walls,transposed", [
    pytest.param(Walls(0), Walls.none(), id="empty"),
    pytest.param(Walls.NORTH, Walls.EAST, id="north"),
    pytest.param(Walls.EAST, Walls.NORTH, id="east"),
    pytest.param(Walls.SOUTH, Walls.WEST, id="south"),
    pytest.param(Walls.WEST, Walls.SOUTH, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, Walls.NORTH | Walls.EAST, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, Walls.EAST | Walls.WEST, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, Walls.EAST | Walls.SOUTH, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, Walls.NORTH | Walls.WEST, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, Walls.NORTH | Walls.SOUTH, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, Walls.SOUTH | Walls.WEST, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, ~Walls.SOUTH, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, ~Walls.WEST, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, ~Walls.NORTH, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, ~Walls.EAST, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, Walls.all(), id="all"),
])
def test_secondary_transpose(walls: Walls, transposed: Walls):
    assert walls.secondary_transpose() == transposed


@pytest.mark.parametrize("walls,transposed", [
    pytest.param(Walls(0), Walls.none(), id="empty"),
    pytest.param(Walls.NORTH, Walls.NORTH, id="north"),
    pytest.param(Walls.EAST, Walls.WEST, id="east"),
    pytest.param(Walls.SOUTH, Walls.SOUTH, id="south"),
    pytest.param(Walls.WEST, Walls.EAST, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, Walls.NORTH | Walls.WEST, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, Walls.NORTH | Walls.SOUTH, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, Walls.NORTH | Walls.EAST, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, Walls.SOUTH | Walls.WEST, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, Walls.EAST | Walls.WEST, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, Walls.EAST | Walls.SOUTH, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, ~Walls.EAST, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, ~Walls.SOUTH, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, ~Walls.WEST, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, ~Walls.NORTH, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, Walls.all(), id="all"),
])
def test_flip_horizontally(walls: Walls, transposed: Walls):
    assert walls.flip_horizontally() == transposed


@pytest.mark.parametrize("walls,transposed", [
    pytest.param(Walls(0), Walls.none(), id="empty"),
    pytest.param(Walls.NORTH, Walls.SOUTH, id="north"),
    pytest.param(Walls.EAST, Walls.EAST, id="east"),
    pytest.param(Walls.SOUTH, Walls.NORTH, id="south"),
    pytest.param(Walls.WEST, Walls.WEST, id="west"),
    pytest.param(Walls.NORTH | Walls.EAST, Walls.EAST | Walls.SOUTH, id="north-east"),
    pytest.param(Walls.NORTH | Walls.SOUTH, Walls.NORTH | Walls.SOUTH, id="north-south"),
    pytest.param(Walls.NORTH | Walls.WEST, Walls.SOUTH | Walls.WEST, id="north-west"),
    pytest.param(Walls.EAST | Walls.SOUTH, Walls.NORTH | Walls.EAST, id="east-south"),
    pytest.param(Walls.EAST | Walls.WEST, Walls.EAST | Walls.WEST, id="east-west"),
    pytest.param(Walls.SOUTH | Walls.WEST, Walls.NORTH | Walls.WEST, id="south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH, ~Walls.WEST, id="north-east-south"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.WEST, ~Walls.NORTH, id="north-east-west"),
    pytest.param(Walls.NORTH | Walls.SOUTH | Walls.WEST, ~Walls.EAST, id="north-south-west"),
    pytest.param(Walls.EAST | Walls.SOUTH | Walls.WEST, ~Walls.SOUTH, id="east-south-west"),
    pytest.param(Walls.NORTH | Walls.EAST | Walls.SOUTH | Walls.WEST, Walls.all(), id="all"),
])
def test_flip_vertically(walls: Walls, transposed: Walls):
    assert walls.flip_vertically() == transposed
