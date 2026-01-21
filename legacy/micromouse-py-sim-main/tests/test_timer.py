# pylint: disable=missing-function-docstring,missing-module-docstring
from __future__ import annotations

# import pytest

from sim.robots.utils import GiveUpTimer


def test_simple():
    timer = GiveUpTimer(limit=3)

    assert timer.limit == 3
    assert timer.stopped
    assert not timer.started
    assert timer.count == 0
    assert not timer.expired
    assert bool(timer)

    timer.start()

    assert timer.limit == 3
    assert not timer.stopped
    assert timer.started
    assert timer.count == 0
    assert not timer.expired
    assert bool(timer)

    timer.update()

    assert timer.limit == 3
    assert not timer.stopped
    assert timer.started
    assert timer.count == 1
    assert not timer.expired
    assert bool(timer)

    timer.update(2)

    assert timer.limit == 3
    assert not timer.stopped
    assert timer.started
    assert timer.count == 3
    assert not timer.expired
    assert bool(timer)

    timer.update()

    assert timer.limit == 3
    assert not timer.stopped
    assert timer.started
    assert timer.count == 4
    assert timer.expired
    assert not timer


def test_update_stopped_timer():
    timer = GiveUpTimer(limit=3)

    assert timer.stopped
    assert timer.count == 0

    timer.update()

    assert timer.stopped
    assert timer.count == 0

    timer.update(45)

    assert timer.stopped
    assert timer.count == 0

    assert not timer.expired


def test_unlimited_timer():
    timer = GiveUpTimer(limit=None)

    assert timer.limit is None

    timer.start()

    timer.update()

    assert not timer.expired

    timer.update(1_000_000)

    assert timer.count == 1_000_001
    assert not timer.expired


def test_start_stop_timer():
    timer = GiveUpTimer(limit=3)

    assert timer.stopped
    assert timer.count == 0

    timer.start()
    timer.update()
    timer.stop()
    timer.update(10)

    assert timer.stopped
    assert timer.count == 1
    assert not timer.expired

    timer.start()
    timer.update(45)

    assert timer.expired


def test_autostart():
    timer = GiveUpTimer(limit=0, autostart=True)

    assert timer.started

    timer.update()

    assert timer.expired


def test_reset():
    timer = GiveUpTimer(limit=3)

    timer.start()
    timer.update(11)

    assert timer.expired

    timer.reset()

    assert timer.stopped
    assert timer.count == 0
    assert not timer.expired

    timer.start()
    timer.update(2)

    assert timer.count == 2
    assert not timer.expired

    timer.reset(stop=False)

    assert timer.started
    assert timer.count == 0
    assert not timer.expired

    timer.update(2)

    assert timer.count == 2
    assert not timer.expired
