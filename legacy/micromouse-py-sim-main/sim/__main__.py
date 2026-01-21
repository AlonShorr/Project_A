"""main entrypoint for the simulator"""
from __future__ import annotations

import argparse
import sys

from importlib.metadata import entry_points

from .directions import Direction
from .front import (
    direction,
    LoadPreset,
    maze,
    position_set,
    position,
    Renderer,
    Tool,
)
from .maze import ExtendedMaze, Maze
from .robots import idle_robot, load_robots
from .robots.utils import walls_to_directions
from .simulator import Simulator

__ENGINE_ENTRYPOINT = 'micromouse.gui'


def __load_gui_engine(name: str) -> type[Renderer]:
    engine: type[Renderer] | None = None
    for renderer in entry_points(group=__ENGINE_ENTRYPOINT, name=name):
        try:
            renderer_class = renderer.load()
        except (AttributeError, ImportError) as err:
            print(f"warning: failed to load renderer {renderer.name!r}: {err}", file=sys.stderr)
            continue
        if not issubclass(renderer_class, Renderer):
            print(
                f"warning: failed to load renderer {renderer.name!r}: must inherit from {Renderer.__module__}.{Renderer.__qualname__}",
                file=sys.stderr,
            )
            continue
        if engine is not None:
            print(
                f"warning: {renderer.name!r} from {renderer.module}:{renderer.attr} overrides "
                f"{engine.__module__}:{engine.__qualname__}",
                file=sys.stderr,
            )
        engine = renderer_class
    if engine is None:
        print(f"error: no valid renderer for {name!r}", file=sys.stderr)
        sys.exit(1)
    return engine


def __build_tool_parser(main_parser: argparse.ArgumentParser) -> None:
    tools = main_parser.add_subparsers(
        title='tools',
        description="All registered tools.",
        required=True,
    )

    def __priority(module: str) -> tuple[int, str]:
        return (int(not (bool(__package__) and module.startswith(f"{__package__}."))), module)

    taken_names: dict[str, type[Tool]] = {}
    for tool in sorted(entry_points(group='micromouse.tool'), key=lambda ep: __priority(ep.module)):
        if tool.name in taken_names:
            print(
                f"warning: not loading {tool.name} from {tool.module}:{tool.attr}: would override "
                f"{taken_names[tool.name].__module__}:{taken_names[tool.name].__qualname__}",
                file=sys.stderr,
            )
            continue
        try:
            tool_class = tool.load()
        except (AttributeError, ImportError) as err:
            print(f"warning: failed to load tool {tool.name}: {err}", file=sys.stderr)
            continue
        if not issubclass(tool_class, Tool):
            print(
                f"warning: failed to load tool {tool.name}: must inherit from {Tool.__module__}.{Tool.__qualname__}",
                file=sys.stderr,
            )
            continue

        parser_args = tool_class.PARSER_ARGS.copy()

        # Add load information to epilog
        origin = f"Loaded from {tool.value}"
        if epilog := parser_args.pop('epilog', None):
            epilog += f"\n{origin}"
        else:
            epilog = origin
        parser_args['epilog'] = epilog

        # Fill description if needed
        if not parser_args.get('description') and tool_class.__doc__:
            parser_args['description'] = tool_class.__doc__

        parser = tools.add_parser(
            tool.name,
            help=tool_class.__doc__ or f'{tool.module}:{tool.attr}',
            **parser_args,
        )
        tool_class.build_parser(parser)
        parser.set_defaults(tool_main=tool_class.main)

        taken_names[tool.name] = tool_class


def main():
    """
    The main entrypoint for the simulation, loads a simulation based on
    commandline arguments and triggers the selected renderer entrypoint.
    """
    parser = argparse.ArgumentParser(
        description="Micromouse simulator.",
    )
    subparsers = parser.add_subparsers(
        title='subcommands',
        dest='main_action',
        required=True,
    )
    sim = subparsers.add_parser(
        'sim',
        help="Run the simulator.",
        description="Run the simulator.",
    )
    sim.add_argument(
        '-m', '--maze',
        type=maze,
        default=Maze.from_maze_text("+---+\n|   |\n+---+\n"),
        help="The maze to load. May be a file or a maze input. (default: 1x1 maze)",
    )
    sim.add_argument(
        '-s', '--start-pos',
        type=position,
        default=(0, 0),
        help="The starting position. (default: the top-left corner - (0, 0))",
    )
    sim.add_argument(
        '-d', '--start-direction',
        type=direction,
        help="The starting direction. (default: a valid direction for the starting position)",
    )
    sim.add_argument(
        '-g', '--goals',
        type=position_set,
        # default={(M - 1, N - 1)},
        help="The goal positions. (default: the bottom-right corner - (M-1, N-1) in a MxN maze)",
    )
    sim.add_argument(
        '-p', '--preset',
        action=LoadPreset,
        dest='maze,start_pos,start_direction,goals',
        metavar='PRESET',
        presets_file='mazes/presets.json',
        help="A maze+start+goals preset to load.",
    )
    assert any(ep.name == 'default' for ep in entry_points(group=__ENGINE_ENTRYPOINT)), "missing default entrypoint"
    sim.add_argument(
        '-e', '--engine',
        choices={ep.name for ep in entry_points(group=__ENGINE_ENTRYPOINT)},
        default='default',  # 'default' comes from this package
        help="The render engine to use. (default: a simple pygame GUI renderer, requires the 'gui' feature)",
    )

    tool = subparsers.add_parser(
        'tool',
        help="Run a tool.",
        description="Run a tool.",
    )
    __build_tool_parser(tool)

    args = parser.parse_args()

    load_robots()

    # print(args)

    match args.main_action:
        case 'sim':
            if args.goals is None:
                args.goals = {(args.maze.height - 1, args.maze.width - 1)}
            if args.start_direction is None:
                args.start_direction = (walls_to_directions(args.maze[args.start_pos]) or [Direction.NORTH])[0]

            if args.maze:
                print(ExtendedMaze.full_from_maze(args.maze).render_extra(
                    pos=args.start_pos + (args.start_direction,),
                    goals=args.goals,
                    weights=False,
                ))
            else:
                print("no maze")
                sys.exit(1)

            __load_gui_engine(args.engine)(Simulator(
                alg=idle_robot,
                maze=args.maze,
                begin=args.start_pos + (args.start_direction,),
                end=args.goals,
            )).run()
        case 'tool':
            args.tool_main(args)


if __name__ == '__main__':
    main()
