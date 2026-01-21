"""Utilities for implementing plugins for this package.

Classes:
* Renderer - base class for "micromouse.gui" entrypoints.
* Tool - base class for "micromouse.tool" entrypoints.
* ParserArgs - Typing utility for ``Tool.PARSER_ARGS``.
* LoadPreset - ``argparse`` action for loading a maze+start+goal preset.

Functions:
* maze - ``argparse`` type for loading a maze from a path or big string argument.
* size - ``argparse`` type for parsing a {height}x{width} size.
* position - ``argparse`` type for parsing a ({row}, {col}) coordinate.
* position_set - ``argparse`` type for parsing semicolon (':') separated ({row}, {col}) coordinates.
* direction - ``argparse`` type for parsing a north/east/south/west direction.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

from abc import ABC, abstractmethod
from collections.abc import Set
from io import StringIO
from typing import TypedDict, TYPE_CHECKING

from .directions import Direction
from .maze import Maze

if TYPE_CHECKING:
    from collections.abc import Sequence
    from .simulator import Simulator


class Renderer(ABC):  # pylint: disable=too-few-public-methods
    """Base renderer class for simulation (GUI) renderers."""

    def __init__(self, sim: Simulator) -> None:
        super().__init__()
        self.sim = sim

    @abstractmethod
    def run(self) -> None:
        """Start the renderer."""
        raise NotImplementedError


class ParserArgs(TypedDict, total=False):
    """Additional extra arguments for creating a parser."""
    aliases: Sequence[str]
    usage: str | None
    description: str | None
    epilog: str | None
    formatter_class: argparse._FormatterClass
    prefix_chars: str
    fromfile_prefix_chars: str | None
    conflict_handler: str
    add_help: bool
    allow_abbrev: bool


class Tool(ABC):
    """Base tool class for tools that use the 'micromouse.tool' entrypoint."""

    PARSER_ARGS: ParserArgs = {}

    @classmethod
    @abstractmethod
    def build_parser(cls, parser: argparse.ArgumentParser) -> None:
        """
        Populate the sub-parser for this tool with arguments.
        The parser's ``help`` is populated from the class's docstring if it is not empty,
        or with a default string if it is.

        Args:
            parser (argparse.ArgumentParser): The parser to populate.
        """
        raise NotImplementedError

    @classmethod
    @abstractmethod
    def main(cls, args: argparse.Namespace) -> None:
        """Main entrypoint for the tool.

        Args:
            args (argparse.Namespace): The parsed commandline arguments.
        """
        raise NotImplementedError


def __maze_from_num(num: str) -> Maze:
    return Maze.from_num_file(StringIO(num))


def __maze_from_maz(maz: str) -> Maze:
    if re.fullmatch(r'(?:\\x[a-fA-F0-9]{2}|\s)+', maz):
        maz_bytes = bytes.fromhex(maz.replace('\\x', ''))
    elif re.fullmatch(r'(?:0x[a-fA-F0-9]{1,2}\s*,?|\s|[\[\](){}])+', maz):
        maz_bytes = bytes(int(byte, 16) for byte in re.sub(r'[\[\](){}\s]', '', maz).split(','))
    elif re.fullmatch(r'(?:[a-fA-F0-9]{2}|\s)+', maz):
        maz_bytes = bytearray.fromhex(maz)
    elif all(0 <= ord(c) <= 0xF for c in maz):
        maz_bytes = maz.encode(encoding="ASCII")
    else:
        raise argparse.ArgumentTypeError("invalid maz format")
    return Maze.from_maz(maz_bytes)


def __maze(arg: str) -> Maze:
    typ = ''
    if ':' in arg:
        typ, sep, arg = arg.partition(':')
        arg = arg.lstrip()
        if not sep and not typ:
            raise argparse.ArgumentTypeError("found ':' but maze type is empty")
    match typ:
        case '':
            file_parser = Maze.from_file
            text_parser = Maze.from_maze_text
        case 'maze':
            file_parser = Maze.from_maze_file
            text_parser = Maze.from_maze_text
        case 'maz':
            file_parser = Maze.from_maz_file
            text_parser = __maze_from_maz
        case 'num':
            file_parser = Maze.from_num_file
            text_parser = __maze_from_num
        case 'csv':
            file_parser = Maze.from_csv_file
            text_parser = Maze.from_csv
        case _:
            raise argparse.ArgumentTypeError(f"unknown maze type: {typ!r}")
    if arg == '-':
        return text_parser(sys.stdin.read())
    if typ and not os.path.exists(arg):
        return text_parser(arg)
    return file_parser(arg)  # type: ignore


def maze(arg: str) -> Maze:
    """Maze type for argparse.

    Format:
        The maze type format is either an optional maze format name followed by a semicolon
        and a file path (if the format is not specified, it is deduced from the file suffix
        and defaults to '.maze') or a maze format followed by a semicolon and then a maze
        literal (the contents of a file with the matching format).

    Args:
        arg (str): The command-line argument.

    Raises:
        argparse.ArgumentTypeError: Invalid format or an error occurred while reading/parsing the maze.

    Returns:
        Maze: The maze parsed from the argument.
    """
    try:
        return __maze(arg)
    except argparse.ArgumentTypeError:
        raise
    except Exception as error:
        raise argparse.ArgumentTypeError(str(error))


def size(arg: str) -> tuple[int, int]:
    """Size type for argparse.

    Format:
        A "size" is a tuple of (height, width) coordinates in a "{height}x{width}" format.

    Args:
        arg (str): The command-line argument.

    Raises:
        ValueError: The argument is not in a {number}x{number} format.

    Returns:
        tuple[int, int]: A (height, width) size.
    """
    if m := re.fullmatch(r'(?P<height>\d+)x(?P<width>\d+)', arg):
        return (int(m['height']), int(m['width']))
    raise ValueError



def position(arg: str) -> tuple[int, int]:
    """Position type for argparse.

    Format:
        A "position" is a tuple of (row, col) coordinates in a "({row}, {col})" format.
        Meaning a "position" is 2 comma-separated integers that can be surrounded by parentheses
        (but don't have to be), and whitespace is ignored.

    Args:
        arg (str): The command-line argument.

    Raises:
        ValueError: The argument is in an incorrect format.

    Returns:
        tuple[int, int]: A (row, col) coordinate.
    """
    if m := re.fullmatch(r'(?P<open>\()?\s*(?P<row>\d+)\s*,\s*(?P<col>\d+)\s*(?(open)\)|)', arg.strip()):
        return (int(m['row']), int(m['col']))
    raise ValueError


def position_set(arg: str) -> Set[tuple[int, int]]:
    """Position set type for argparse.

    Format:
        A list of semicolon-separated positions (see ``position(arg)``).

    Args:
        arg (str): The command-line argument.

    Raises:
        ValueError: The argument is in an incorrect format.

    Returns:
        Set[tuple[int, int]]: A set of (row, col) coordinates.
    """
    return {position(pos) for pos in arg.split(':')}


def direction(arg: str) -> Direction:
    """Direction type for argparse.

    Format:
        A case insensitive name or abbreviation of a cardinal direction.
        (See ``Direction.from_str(s)``).

    Args:
        arg (str): The command-line argument.

    Raises:
        ValueError: The argument is in an incorrect format.

    Returns:
        Direction: A cardinal direction.
    """
    return Direction.from_str(arg)


class Preset(TypedDict, total=True):
    """A maze + starting point + starting direction + goals preset."""
    file: str
    start_pos: tuple[int, int]
    start_direction: Direction
    goals: Set[tuple[int, int]]


_PRESET_KEYS = frozenset(Preset.__annotations__)


def __translate_pos(obj: object, main_name: str, name: str) -> tuple[int, int]:
    if not isinstance(obj, list | tuple):
        raise TypeError(f"in {main_name}: invalid type for '{name}': {type(obj).__name__}")
    if len(obj) != 2:
        raise TypeError(f"in {main_name}: invalid type for '{name}': expected 2 items, found {len(obj)}")
    if not all(isinstance(v, int) for v in obj):
        raise TypeError(
            f"in {main_name}: invalid type for '{name}': "
            f"expected ints, found {', '.join(type(v).__name__ for v in obj)}"
        )
    return tuple(obj)


def __translate(d: dict[str, object], name: str) -> Preset:
    keys = frozenset(d)
    if _PRESET_KEYS != keys:
        if missing := _PRESET_KEYS - keys:
            raise TypeError(f"in {name}: missing keys: {', '.join(missing)}")
        if extra := keys - _PRESET_KEYS:
            raise TypeError(f"in {name}: unexpected keys: {', '.join(extra)}")

    if not isinstance(file := d.get('file'), str):
        raise TypeError(f"invalid type for 'file': {type(file).__name__}")

    start_pos = __translate_pos(d.get('start_pos'), name, 'start_pos')

    if not isinstance(start_direction := d.get('start_direction'), str | Direction):
        raise TypeError(f"invalid type for 'start_direction': {type(start_direction).__name__}")
    if isinstance(start_direction, str):
        start_direction = Direction.from_str(start_direction)

    if not isinstance(goals := d.get('goals'), list | Set):
        raise TypeError(f"invalid type for 'goals': {type(goals).__name__}")
    goals = {__translate_pos(g, name, f'goals[{i}]') for i, g in enumerate(goals)}
    return {'file': file, 'start_pos': start_pos, 'start_direction': start_direction, 'goals': goals}


def load_presets(presets_json_file: str) -> dict[str, Preset]:
    """Load maze presets from a presets JSON file.

    Args:
        presets_json_file (str): The path to the file to load.

    Raises:
        OSError: On errors when opening the file.
        json.decoder.JSONDecodeError: On errors parsing the JSON.
        TypeError: If the JSON is in an incorrect format (bad types).
        ValueError: If a field has an invalid value (i.e.: non-existent start_direction).

    Returns:
        dict[str, Preset]: The parsed presets.
    """
    with open(presets_json_file, encoding="utf-8") as presets_file:
        raw_presets = json.load(presets_file)
    if not isinstance(raw_presets, dict):
        raise TypeError(f"invalid presets main object type: {type(raw_presets).__name__}")

    return {
        name: __translate(info, name)
        for name, info in raw_presets.items()
    }


class LoadPreset(argparse.Action):
    """An argparse action that loads a maze + starting point + goals preset.

    The ``dest`` param should be a comma-separated string with the format of:
    "{maze_dest},{starting_point_dest},{start_direction_dest},{goals}"
    (additional whitespaces are allowed)
    """

    def __init__(  # pylint: disable=too-many-arguments
            self,
            option_strings: Sequence[str],
            dest: str,
            default: str | None = None,
            required: bool = False,
            help: str | None = None,  # pylint: disable=redefined-builtin
            metavar: str | tuple[str, ...] | None = None,
            presets_file: str = 'presets.json',
    ) -> None:
        self.presets: dict[str, Preset] | None = None
        self.presets_file = presets_file
        self.maze_dest, self.start_pos_dest, self.start_direction_dest, self.goal_dest = map(str.strip, dest.split(','))
        super().__init__(
            option_strings=option_strings,
            dest=self.maze_dest,
            default=default,
            required=required,
            help=help,
            metavar=metavar,
        )

    def _load_presets(self, force: bool = False):
        if self.presets and not force:
            return

        self.presets = load_presets(self.presets_file)

    def __call__(self, parser, namespace, values, option_string=None):
        if option_string not in self.option_strings:
            return

        try:
            self._load_presets()
        except Exception as err:
            raise argparse.ArgumentError(self, f"failed to load presets: {err!s}")
        assert self.presets is not None

        if values not in self.presets:
            raise argparse.ArgumentError(self, f"unknown preset: {values}")

        preset = self.presets[values]
        try:
            setattr(namespace, self.maze_dest, maze(preset['file']))
        except Exception as err:
            raise argparse.ArgumentError(self, str(err))
        setattr(namespace, self.start_pos_dest, preset['start_pos'])
        setattr(namespace, self.start_direction_dest, preset['start_direction'])
        setattr(namespace, self.goal_dest, preset['goals'])
