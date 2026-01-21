"""Example tools.
"""
from __future__ import annotations

import argparse
import sys

from typing import Literal, TextIO

from .front import (
    maze as maze_type,
    size as size_type,
    Tool,
)
from .maze import (
    ascii_charset,
    Charset,
    Maze,
    utf8_charset,
    Walls,
)


class MazeEditor(Tool):
    """Utilities for viewing and manipulating a maze."""

    @classmethod
    def build_parser(cls, parser: argparse.ArgumentParser) -> None:
        subcommands = parser.add_subparsers(
            title='subcommands',
            dest='maze_command',
            required=True,
        )

        generate = subcommands.add_parser(
            'generate',
            help="Generate a new maze.",
            description="Generate a new maze.",
        )
        generate.add_argument(
            'output_file',
            type=argparse.FileType("wt", encoding="utf-8"),
            help="The path to save the new maze at ('-' for stdout).",
        )
        generate.add_argument(
            '-s', '--size',
            type=size_type,
            default="16x16",
            help="The size of the maze to create. (default: 16x16)",
        )
        gen_methods = generate.add_mutually_exclusive_group()
        gen_methods.add_argument(
            '--empty',
            action='store_const',
            const='empty',
            dest='gen_method',
            help="Generate an empty maze (no walls).",
        )
        gen_methods.add_argument(
            '--full',
            action='store_const',
            const='full',
            dest='gen_method',
            help="Generate a full maze (all walls). (default)",
        )
        generate.set_defaults(gen_method='full')

        render = subcommands.add_parser(
            'render',
            help="Render a maze (can be used to convert other formats into .maze format).",
            description="Render a maze.",
        )
        render.add_argument(
            'maze',
            type=maze_type,
            help="The maze to load. May be a file or a maze input.",
        )
        render.add_argument(
            '-A', '--ascii',
            action='store_const',
            const=ascii_charset,
            dest='charset',
            help="Use ASCII characters to draw the maze. (default)",
        )
        render.add_argument(
            '-U', '--unicode',
            action='store_const',
            const=utf8_charset,
            dest='charset',
            help="Use Unicode (UTF-8) characters to draw the maze.",
        )
        render.add_argument(
            '--cell-width',
            type=int,
            default=3,
            help="The amount of characters between cell corners horizontally. (default: 3).",
        )
        render.add_argument(
            '--cell-height',
            type=int,
            default=1,
            help="The amount of characters between cell corners vertically. (default: 1).",
        )
        render.add_argument(
            '--no-force-corners',
            action='store_false',
            dest='force_corners',
            help="Don't draw corners between cells where no walls are attached.",
        )
        render.add_argument(
            '-o', '--output-file',
            type=argparse.FileType("wt", encoding="utf-8"),
            default='-',
            help="Render the maze into a file. Use '-' for stdout. (default: stdout)",
        )
        render.set_defaults(charset=ascii_charset)

        rotate = subcommands.add_parser(
            'rotate',
            help="Rotate a maze (can be used to convert other formats into .maze format).",
            description="Rotate a maze.",
        )
        rotate.add_argument(
            'maze',
            type=maze_type,
            help="The maze to load. May be a file or a maze input.",
        )
        rotate.add_argument(
            '-o', '--output-file',
            type=argparse.FileType("wt", encoding="utf-8"),
            default='-',
            help="Rotate the maze into a file. Use '-' for stdout. (default: stdout)",
        )
        direction = rotate.add_mutually_exclusive_group()
        direction.add_argument(
            '-l', '--left',
            action='store_const',
            const='left',
            dest='direction',
            help="Rotate the maze to the left (counter-clockwise).",
        )
        direction.add_argument(
            '-r', '--right',
            action='store_const',
            const='right',
            dest='direction',
            help="Rotate the maze to the right (clockwise). (default)",
        )
        rotate.set_defaults(direction='right')
        rotate.add_argument(
            '-n', '--rotations',
            type=int,
            default=1,
            help="The number of 90-degree rotations to apply. (default: 1)",
        )

        transpose = subcommands.add_parser(
            'transpose',
            help="Transpose a maze.",
            description="Transpose a maze.",
        )
        transpose.add_argument(
            'maze',
            type=maze_type,
            help="The maze to load. May be a file or a maze input.",
        )
        transpose.add_argument(
            '-o', '--output-file',
            type=argparse.FileType("wt", encoding="utf-8"),
            default='-',
            help="Transpose the maze into a file. Use '-' for stdout. (default: stdout)",
        )
        transpose.add_argument(
            '-s', '--secondary-diagonal',
            action='store_const',
            const='secondary',
            default='primary',
            dest='diagonal',
            help="Transpose the maze along the secondary diagonal. (default: primary diagonal)",
        )

        flip = subcommands.add_parser(
            'flip',
            help="Flip a maze.",
            description="Flip a maze.",
        )
        flip.add_argument(
            'maze',
            type=maze_type,
            help="The maze to load. May be a file or a maze input.",
        )
        flip.add_argument(
            '-o', '--output-file',
            type=argparse.FileType("wt", encoding="utf-8"),
            default='-',
            help="Flip the maze into a file. Use '-' for stdout. (default: stdout)",
        )
        flip.add_argument(
            '-a', '--axis',
            choices=['horizontal', 'vertical'],
            help="The axis to flip.",
        )

    @classmethod
    def main(cls, args: argparse.Namespace) -> None:
        match args.maze_command:
            case 'generate':
                cls.generate(
                    output=args.output_file,
                    size=args.size,
                    method=args.gen_method,
                )
            case 'render':
                cls.render(
                    maze=args.maze,
                    charset=args.charset,
                    cell_size=(args.cell_height, args.cell_width),
                    force_corners=args.force_corners,
                    output=args.output_file,
                )
            case 'rotate':
                cls.rotate(
                    maze=args.maze,
                    direction=args.direction,
                    n=args.rotations,
                    output=args.output_file,
                )
            case 'transpose':
                cls.transpose(
                    maze=args.maze,
                    diagonal=args.diagonal,
                    output=args.output_file,
                )
            case 'flip':
                cls.flip(
                    maze=args.maze,
                    axis=args.axis,
                    output=args.output_file,
                )
            case _:
                print(f"unknown command: {args.maze_command}", file=sys.stderr)
                sys.exit(1)

    @staticmethod
    def generate(
            output: TextIO,
            size: tuple[int, int] = (16, 16),
            method: Literal['empty', 'full'] = 'full',
    ) -> None:
        """Generate a new maze.

        Args:
            output (TextIO): The file to render the maze into. Defaults to stdout.
            size (tuple[int, int], optional): _description_. Defaults to (16, 16).
            method (Literal['empty', 'full']): Generation method. Defaults to 'full'.
            charset (Charset): Character set to use. Defaults to ASCII.
        """
        match method:
            case 'empty':
                func = Maze.empty
            case 'full':
                func = Maze.full
            case _:
                raise ValueError(f"unexpected method: {method!r}")

        output.write(func(*size).render())

    @staticmethod
    def render(
            maze: Maze,
            charset: Charset = ascii_charset,
            cell_size: tuple[int, int] = (1, 3),
            force_corners: bool = True,
            output: TextIO = sys.stdout,
    ) -> None:
        """Render a maze.

        Args:
            maze (Maze): The maze to render.
            charset (Charset): Character set to use. Defaults to ASCII.
            cell_size (tuple[int, int]): A (height, width) tuple of the amount of characters between 2 corners. Defaults to (1, 3).
            force_corners (bool): Draw corners, even if no walls ar attached. Defaults to True.
            output (TextIO): The file to render the maze into. Defaults to stdout.
        """
        output.write(maze.render(
            charset=charset,
            cell_height=cell_size[0],
            cell_width=cell_size[1],
            force_corners=force_corners,
        ))

    @staticmethod
    def rotate(
            maze: Maze,
            direction: Literal['left', 'right'],
            n: int = 1,
            output: TextIO = sys.stdout,
    ) -> None:
        """Rotate a maze.

        Args:
            maze (Maze): The maze to render.
            direction (Literal['left', 'right']): Direction to rotate.
            n (int): Amount of 90-degree rotations to apply. Defaults to 1.
            output (TextIO): The file to render the maze into. Defaults to stdout.
        """
        match direction:
            case 'left':
                rotate_cell = Walls.rotate_left
                reduced_n = (4 - (n & 0x3)) & 0x3  # (4 - n) % 4
            case 'right':
                rotate_cell = Walls.rotate_right
                reduced_n = n & 0x3  # n % 4
            case _:
                raise ValueError(f"unknown direction: {direction!r}")

        match reduced_n:
            case 0:  # No rotation
                def _rotate_pos(row: int, col: int) -> tuple[int, int]:
                    return (row, col)
            case 1:  # Rotate right
                def _rotate_pos(row: int, col: int) -> tuple[int, int]:
                    return (col, maze.height - 1 - row)
            case 2:  # Rotate 180
                def _rotate_pos(row: int, col: int) -> tuple[int, int]:
                    return (maze.height - 1 - row, maze.width - 1 - col)
            case 3:  # Rotate left
                def _rotate_pos(row: int, col: int) -> tuple[int, int]:
                    return (maze.width - 1 - col, row)
            case _:
                raise AssertionError(f"bad modulus calculation (got {n!r})")

        # For odd rotations, maze size swaps the dimensions
        result = Maze.empty(*(maze.size[::-1] if n & 1 else maze.size))
        for (row, col, cell) in maze:
            result[_rotate_pos(row, col)] = rotate_cell(cell, n)

        output.write(result.render())

    @staticmethod
    def transpose(
            maze: Maze,
            diagonal: Literal['primary', 'secondary'] = 'primary',
            output: TextIO = sys.stdout,
    ) -> None:
        """Transpose a maze.

        Args:
            maze (Maze): The maze to transpose.
            diagonal (Literal['primary', 'secondary']): Diagonal to transpose. Defaults to 'primary'.
            output (TextIO): The file to save the maze at. Defaults to stdout.
        """
        match diagonal:
            case 'primary':
                transpose_cell = Walls.transpose

                def _transpose_pos(row: int, col: int) -> tuple[int, int]:
                    return col, row
            case 'secondary':
                transpose_cell = Walls.secondary_transpose

                def _transpose_pos(row: int, col: int) -> tuple[int, int]:
                    return maze.width - 1 - col, maze.height - 1 - row
            case _:
                raise ValueError(f"unexpected diagonal: {diagonal!r}")

        # For odd rotations, maze size swaps the dimensions
        result = Maze.empty(*maze.size[::-1])
        for (row, col, cell) in maze:
            result[_transpose_pos(row, col)] = transpose_cell(cell)

        output.write(result.render())

    @staticmethod
    def flip(
            maze: Maze,
            axis: Literal['horizontal', 'vertical'],
            output: TextIO = sys.stdout,
    ) -> None:
        """Flip a maze.

        Args:
            maze (Maze): The maze to flip.
            direction (Literal['horizontal', 'vertical']): Direction to flip.
            output (TextIO): The file to save the maze at. Defaults to stdout.
        """
        match axis:
            case 'horizontal':
                flip_cell = Walls.flip_horizontally

                def _flip_pos(row: int, col: int) -> tuple[int, int]:
                    return row, maze.width - 1 - col
            case 'vertical':
                flip_cell = Walls.flip_vertically

                def _flip_pos(row: int, col: int) -> tuple[int, int]:
                    return maze.height - 1 - row, col
            case _:
                raise ValueError(f"unexpected axis: {axis!r}")

        result = Maze.empty(*maze.size)
        for (row, col, cell) in maze:
            result[_flip_pos(row, col)] = flip_cell(cell)

        output.write(result.render())
