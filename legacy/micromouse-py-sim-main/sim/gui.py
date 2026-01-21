"""GUI elements and running the GUI
"""
# pylint gives too many false-positive "no member" for pygame attributes
# pylint: disable=no-member

from __future__ import annotations

import math

from datetime import datetime, timedelta
from os import environ
from typing import Iterable, NamedTuple, Self

import rich

from .directions import Direction, RelativeDirection
from .front import Renderer
from .maze import ExtraCellInfo, ExtendedMaze, Maze, Walls
from .robots import (
    ROBOTS,
    basic_weighted_flood_fill,
    dijkstra_flood_fill,
    idle_robot,
    random_robot,
    simple_flood_fill,
    thorough_flood_fill,
    wall_follower_robot,
)
from .simulator import SimulationStatus, Simulator

# Disable the prompt triggered by importing `pygame`.
# autopep8: off
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame as pg  # pylint: disable=wrong-import-order,wrong-import-position
import pygame_gui    # pylint: disable=wrong-import-order,wrong-import-position
# autopep8: on


class Position(NamedTuple):
    """Represents a position on screen."""
    row: int
    col: int

    def __add__(self, other: object) -> Self:
        if not isinstance(other, type(self)):
            return NotImplemented
        return type(self)(self.row + other.row, self.col + other.col)


class GUIRenderer(Renderer):  # pylint: disable=too-many-instance-attributes
    """Class for rendering the GUI on screen"""

    def __init__(self, sim: Simulator):
        """Initialize GUI

        Args:
            sim (Simulator): The simulation to show.
        """
        super().__init__(sim)

        pg.init()
        initial_screen_size = (1280, 720)
        screen = pg.display.set_mode(initial_screen_size, pg.RESIZABLE)
        pg.display.set_caption('Micromouse')
        self.step_delay = timedelta(seconds=0.5)
        self.step_min_delay = timedelta(milliseconds=1)
        self.screen = screen
        self.wall_thickness = 5
        self.wall_color = 'red'
        self.robot_main_color = 'blue'
        self.robot_second_color = 'yellow'
        self.robot_highlight_color = 'white'
        self.goal_color = 'green'
        self.route_color = 'magenta'
        self.heatmap_colors = ['blue', 'cyan', 'green', 'yellow', 'orange', 'red', 'brown']
        self.ui_manager = pygame_gui.UIManager((screen.get_width(), screen.get_height()))
        self.start_button = pygame_gui.elements.UIButton(
            relative_rect=pg.Rect((0, 0), (0, 0)),
            text='Start',
            manager=self.ui_manager,
        )
        self.start_button.tool_tip_text = "Shortcut: 's'"
        self.step_button = pygame_gui.elements.UIButton(
            relative_rect=pg.Rect((0, 0), (0, 0)),
            text='Step',
            manager=self.ui_manager,
        )
        self.step_button.tool_tip_text = "Shortcut: 'n'"
        # self.browse_button = pygame_gui.elements.UIButton(
        #    relative_rect=pg.Rect((0, 0), (0, 0)),
        #    text='Browse maze file',
        #    manager=self.ui_manager,
        # )
        # maze_list = ["Custom", "Maze 2"]
        # self.maze_dropdown = pygame_gui.elements.UIDropDownMenu(
        #    maze_list,
        #    maze_list[0],
        #    pg.Rect((0, 0), (0, 0)),
        #    self.ui_manager,
        # )
        self.robot_dropdown = pygame_gui.elements.UIDropDownMenu(
            list(ROBOTS),
            'Idle',
            pg.Rect((0, 0), (0, 0)),
            self.ui_manager,
        )

        # self.browse_maze_file_dialog = None
        # self.maze_path = ''

        self.sim_speed_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pg.Rect((0, 0), (0, 0)),
            start_value=self.step_delay.total_seconds(),
            value_range=(0.0, 1.0),
            manager=self.ui_manager,
        )

        self.sim_auto_step = True

        self.screen_width, self.screen_height = initial_screen_size
        self.text_size = 4 * min(self.screen_width, self.screen_height) // 100
        self.tile_size = min(self.screen_width // (2 * self.sim.maze.width + 3), self.screen_height // (self.sim.maze.height + 4))
        self.half_tile = self.tile_size // 2
        self.full_maze_offset = Position(self.tile_size, 3 * self.tile_size)
        self.full_maze_center = self.full_maze_offset + Position(self.sim.maze.width * self.half_tile, 0)
        self.robot_maze_offset = self.full_maze_offset + Position((self.sim.maze.width + 1) * self.tile_size, 0)
        self.robot_maze_center = self.robot_maze_offset + Position(self.sim.maze.width * self.half_tile, 0)

    def run(self):
        """Run the GUI
        """
        self.configure_simulator()
        self.scale()
        self.main_loop()
        pg.quit()

    def get_selected_robot(self):
        """Fetch the selected robot.

        Returns:
            Robot: The chosen robot.
        """
        return ROBOTS[self.robot_dropdown.selected_option]

    def get_selected_maze(self):
        """Fetch the selected preset for maze.

        Returns:
            Maze: The selected maze.
        """
        return ('mazes/semifinal.maze', (15, 0, Direction.EAST), {(8, 8), (7, 8), (8, 7), (7, 7)})
        # return ('mazes/simple.maze', (0, 0, Direction.SOUTH), {(1, 2)})

    def configure_simulator(self):
        """Configure the simulator with the chosen maze preset and robot."""
        # self.sim_auto_step = True
        # maze_file, begin, end = self.get_selected_maze()
        # self.sim = Simulator(
        #    alg=self.get_selected_robot(),
        #    maze=Maze.from_file(maze_file),
        #    begin=begin,
        #    end=end,
        # )
        self.scale()

    def process_event(self, event: pg.Event):
        """Process pygame event.

        Args:
            event (pg.Event): The event to process.
        """
        self.ui_manager.process_events(event)
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == self.start_button:
                self.sim_auto_step = not self.sim_auto_step
            elif event.ui_element == self.step_button:
                self.sim_auto_step = False
                self.sim.step()
            # elif event.ui_element == self.browse_button:
            #    self.browse_maze_file_dialog = pygame_gui.windows.UIFileDialog(
            #        rect=pg.Rect(
            #            0,
            #            0,
            #            500,
            #            500),
            #        manager=self.ui_manager,
            #        window_title="Select maze",
            #        initial_file_path="./mazes/",
            #        allow_existing_files_only=True,
            #    )
            #    self.browse_maze_file_dialog.set_blocking(True)
            # if self.browse_maze_file_dialog is not None and event.ui_element == self.browse_maze_file_dialog.ok_button:
            #    self.maze_path = self.browse_maze_file_dialog.current_file_path
            #    print(self.maze_path)  # TODO: use path to create maze...
        elif event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
            # if event.ui_element == self.maze_dropdown:
            #    self.configure_simulator()
            if event.ui_element == self.robot_dropdown:
                self.sim_auto_step = False
                self.sim.restart(self.get_selected_robot())
        elif event.type == pg.KEYDOWN:
            match (event.key, event.mod & pg.KMOD_SHIFT != 0):
                case (pg.K_d, True):
                    self.sim.restart(dijkstra_flood_fill)
                case (pg.K_d, _):
                    self.sim.restart(thorough_flood_fill)
                case (pg.K_f, True):
                    self.sim.restart(simple_flood_fill)
                case (pg.K_f, _):
                    self.sim.restart(basic_weighted_flood_fill)
                case (pg.K_l, _):
                    self.sim.restart(wall_follower_robot(RelativeDirection.LEFT))
                case (pg.K_n, _):
                    self.sim_auto_step = False
                    self.sim.step()
                case (pg.K_r, True):
                    self.sim.restart(random_robot)
                case (pg.K_r, _):
                    self.sim.restart(wall_follower_robot(RelativeDirection.RIGHT))
                case (pg.K_s, _):
                    self.sim_auto_step = not self.sim_auto_step

    def update(self, time_delta: float):
        """Update GUI elements and draw them on screen.

        Args:
            time_delta (float): The time passed since the last call to update, in seconds.
        """
        self.start_button.set_text('Stop' if self.sim_auto_step else 'Start')
        self.step_delay = timedelta(seconds=self.sim_speed_slider.get_current_value())

        self.ui_manager.update(time_delta)
        self.ui_manager.draw_ui(self.screen)
        pg.display.update()

    def draw_maze(
            self,
            maze: ExtendedMaze,
            tile_size: int,
            offset: tuple[int, int] = (0, 0),
            goal_cells: Iterable[tuple[int, int]] = (),
            robot_pos: tuple[int, int] = (0, 0),
            robot_direction: Direction = Direction.NORTH,
            *,
            heatmap: bool = False,
    ):  # pylint: disable=too-many-arguments,too-many-locals,too-many-branches
        """Draw the maze on screen.

        Args:
            maze (Maze): Maze to draw.
            tile_size (int): The size of a maze's single tile.
            offset (tuple[int, int], optional): Offset for maze in pixels from (0, 0). Defaults to (0, 0).
            robot_pos (tuple[int, int], optional): Position of robot in maze (row, col). Defaults to (0, 0).
            robot_direction (Direction): Direction robot is facing. Defaults to north.
            heatmap (bool): Draw heatmap on maze. Defaults to False.
        """
        wall_color = pg.Color(self.wall_color)
        screen_robot_pos: Position | None = None
        for row, col, walls, extra_info in maze.iter_all():
            x = col * tile_size + offset[0]
            y = row * tile_size + offset[1]
            current_pos = Position(x, y)

            if heatmap:
                self.draw_heatmap(extra_info, current_pos)

            for cell in goal_cells:
                if (row, col) == cell:
                    pg.draw.rect(self.screen, self.goal_color, pg.Rect(x, y, tile_size, tile_size))

            self.fill_cell_color(extra_info.color, current_pos, 128)

            if (col, row) == robot_pos:
                screen_robot_pos = current_pos

            line_end = self.wall_thickness // 2
            if Walls.NORTH in walls:
                pg.draw.line(self.screen, wall_color, (x - line_end, y), (x + line_end + tile_size, y), self.wall_thickness)
            if Walls.EAST in walls:
                pg.draw.line(
                    self.screen,
                    wall_color,
                    (x + tile_size, y - line_end),
                    (x + tile_size, y + line_end + tile_size),
                    self.wall_thickness,
                )
            if Walls.SOUTH in walls:
                pg.draw.line(
                    self.screen,
                    wall_color,
                    (x - line_end, y + tile_size),
                    (x + line_end + tile_size, y + tile_size),
                    self.wall_thickness,
                )
            if Walls.WEST in walls:
                pg.draw.line(self.screen, wall_color, (x, y - line_end), (x, y + line_end + tile_size), self.wall_thickness)

        # Draw robot's route after all other maze elements:
        for i in range(len(maze.route) - 1):
            self.draw_route_cell(
                Position(maze.route[i][1] * tile_size + offset[0], maze.route[i][0] * tile_size + offset[1]),
                Position(maze.route[i + 1][1] * tile_size + offset[0], maze.route[i + 1][0] * tile_size + offset[1]),
                self.route_color,
            )

        assert screen_robot_pos is not None, "Robot not in maze"
        self.draw_robot(screen_robot_pos, robot_direction)

        for row, col, extra_info in maze.iter_info():
            current_pos = Position(col * tile_size + offset[0], row * tile_size + offset[1])
            if extra_info.weight is not None:
                self.draw_text_by_center(
                    str(extra_info.weight),
                    self.half_tile,
                    current_pos + Position(self.half_tile + 1, self.half_tile + 1),
                )

    def fill_cell_color(self, color: tuple[int, int, int] | str | None, pos: Position, alpha: int = 255):
        """Fills a cell with color

        Args:
            color (tuple[int, int, int] | str | None): Cell color to draw.
            pos (Position): Cell position in pixels.
            alpha (int, optional): Color's transparent value. Defaults to 255.
        """
        if color is not None:
            pg_color = pg.Color(color)
            alpha_surface = pg.Surface((self.tile_size, self.tile_size), pg.SRCALPHA)
            alpha_surface.fill((pg_color.r, pg_color.g, pg_color.b, alpha))
            self.screen.blit(alpha_surface, pos)

    def draw_round_corners_line(self, start: tuple[int, int], end: tuple[int, int], color: pg.Color, width: int):
        """Draw a line with round edges.

        Args:
            start (tuple[int, int]): Start point on surface.
            end (tuple[int, int]): End point on surface.
            color (pg.Color): Line's color.
            width (int): Line's width.
        """
        pg.draw.line(self.screen, color, start, end, width)
        pg.draw.circle(self.screen, color, start, width // 2)
        pg.draw.circle(self.screen, color, end, width // 2)

    def draw_route_cell(self, current_pos: Position, next_pos: Position, color: tuple[int, int, int] | str | None, alpha: int = 255):
        """Draw a cell in route

        Args:
            current_pos (Position): The current position of the route.
            next_pos (Position): The next position of the route.
            color (tuple[int, int, int] | str | None): The color to draw with.
            alpha (int, optional): Color's transparent value. Defaults to 255.
        """
        if color is not None:
            pg_color = pg.Color(color)
            pg_color.a = alpha
            current_pos += Position(self.half_tile + 1, self.half_tile + 1)
            next_pos += Position(self.half_tile + 1, self.half_tile + 1)
            self.draw_round_corners_line(current_pos, next_pos, pg_color, 3)

    def draw_heatmap(self, info: ExtraCellInfo, cell_pos: Position):
        """Fill cell color by visit count to produce a heatmap.

        Args:
            info (ExtraCellInfo): Cell info to draw.
            cell_pos (Position): Cell position in pixels.
        """
        color = None
        if info.color is not None:
            color = info.color
        elif info.visited > 0:
            color = self.heatmap_colors[min(info.visited, len(self.heatmap_colors)) - 1]

        self.fill_cell_color(color, cell_pos, 200)

    def draw_robot(self, robot_pos: Position, robot_direction: Direction):
        """Draw robot on screen.

        Args:
            robot_pos (Position): Position of robot in maze (in pixels).
            robot_direction (Direction): The direction in which robot is facing.
        """
        robot_radius = self.half_tile * 0.7
        robot_pos = robot_pos + Position(self.half_tile + 1, self.half_tile + 1)
        pg.draw.circle(self.screen, self.robot_highlight_color, robot_pos, robot_radius + 2)
        pg.draw.circle(self.screen, self.robot_main_color, robot_pos, robot_radius)
        heading_point = robot_pos + Position(round(robot_radius * math.cos(robot_direction.to_radians())),
                                             round(robot_radius * math.sin(robot_direction.to_radians())))
        pg.draw.line(self.screen, self.robot_second_color, robot_pos, heading_point, 3)

    def draw_text_by_center(self, text: str, size: int, center: Position, color='white'):
        """Draw text on screen by the position of the center of the text rectangle.

        Args:
            text (str): Text to draw.
            size (int): Size of text.
            center (Position): Position of center of text on screen (x, y).
            color (str, optional): Text's color. Defaults to 'white'.
        """
        font = pg.font.Font(pg.font.get_default_font(), size)
        render_text = font.render(text, True, color)
        text_rect = render_text.get_rect()
        text_rect.center = center
        self.screen.blit(render_text, text_rect)

    def draw_text(self, text: str, size: int, top_left: Position, color='white'):
        """Draw text on screen by the position of the top left corner of the text rectangle.

        Args:
            text (str): Text to draw.
            size (int): Size of text.
            top_left (Position): Position of top left of text on screen (x, y).
            color (str, optional): Text's color. Defaults to 'white'.
        """
        font = pg.font.Font(pg.font.get_default_font(), size)
        render_text = font.render(text, True, color)
        text_rect = render_text.get_rect()
        text_rect.topleft = top_left
        self.screen.blit(render_text, text_rect)

    def scale(self):
        """Calculate sizes and offsets of GUI elements according to screen size."""
        self.screen_width, self.screen_height = pg.display.get_surface().get_size()
        self.ui_manager.set_window_resolution((self.screen_width, self.screen_height))
        self.text_size = 24
        self.tile_size = min(self.screen_width // (2 * self.sim.maze.width + 3), self.screen_height // (self.sim.maze.height + 7))
        self.half_tile = self.tile_size // 2
        self.full_maze_offset = Position(self.tile_size, 6 * self.tile_size)
        self.full_maze_center = self.full_maze_offset + Position(self.sim.maze.width * self.half_tile, 0)
        self.robot_maze_offset = self.full_maze_offset + Position((self.sim.maze.width + 1) * self.tile_size, 0)
        self.robot_maze_center = self.robot_maze_offset + Position(self.sim.maze.width * self.half_tile, 0)
        middle_width = self.screen_width // 2
        self.start_button.set_position((middle_width - 2 * self.tile_size, 0.5 * self.text_size))
        self.start_button.set_dimensions((3 * self.tile_size, 1.5 * self.text_size))
        self.step_button.set_position((middle_width + 2 * self.tile_size, 0.5 * self.text_size))
        self.step_button.set_dimensions((3 * self.tile_size, 1.5 * self.text_size))
        self.sim_speed_slider.set_position((middle_width + 8 * self.tile_size, 0.5 * self.text_size))
        self.sim_speed_slider.set_dimensions((10 * self.text_size, 1.5 * self.text_size))
        # self.browse_button.set_position((8 * self.tile_size, 0))
        # self.browse_button.set_dimensions((5 * self.tile_size, self.tile_size))
        # self.maze_dropdown.set_position((5 * self.text_size, 0))
        # self.maze_dropdown.set_dimensions((4 * self.tile_size, self.tile_size))
        self.robot_dropdown.set_position((6 * self.text_size, 0.5 * self.text_size))
        self.robot_dropdown.set_dimensions((12 * self.text_size, 1.5 * self.text_size))

    def main_loop(self):  # pylint: disable=too-many-locals
        """Main GUI loop. This function will stay in a loop until exit."""
        last_step = datetime.now()
        clock = pg.time.Clock()
        while True:
            time_delta = clock.tick(60) / 1000.0
            step = False
            now = datetime.now()

            if pg.key.get_pressed()[pg.K_SPACE]:
                step = now - last_step >= self.step_min_delay

            for event in pg.event.get():
                if event.type == pg.QUIT:
                    return
                if event.type == pg.VIDEORESIZE:
                    self.scale()
                elif event.type == pg.KEYDOWN:
                    if event.key in (pg.K_q, pg.K_ESCAPE):
                        return

                self.process_event(event)

            if now - last_step >= self.step_delay:
                step = True

            if self.sim.status in (SimulationStatus.ERROR, SimulationStatus.FINISHED):
                step = False

            if step and self.sim_auto_step:
                try:
                    self.sim.step()
                except RuntimeError:
                    rich.get_console().print_exception(show_locals=True)

                last_step = now
                print(f"GUI: after step - {self.sim.maze[self.sim.robot_pos[:-1]]=} {self.sim.robot_maze[self.sim.robot_pos[:-1]]=}")

            self.screen.fill("black")
            self.draw_text_by_center(
                'Full Maze',
                self.text_size,
                self.full_maze_center + Position(0, -self.half_tile),
            )
            self.draw_text_by_center(
                'Robot View',
                self.text_size,
                self.robot_maze_center + Position(0, -self.half_tile),
            )
            robot_y, robot_x, robot_heading = self.sim.robot_pos
            self.draw_maze(
                self.sim.maze,
                self.tile_size,
                self.full_maze_offset,
                self.sim.end,
                (robot_x, robot_y),
                robot_heading,
                heatmap=True,
            )
            self.draw_maze(
                self.sim.robot_maze,
                self.tile_size,
                self.robot_maze_offset,
                self.sim.end,
                (robot_x, robot_y),
                robot_heading,
            )

            middle_width = self.screen_width // 2

            # texts for menus:
            # self.draw_text(
            #    'Preset:',
            #    self.text_size,
            #    Position(0, 0),
            # )
            self.draw_text(
                'Algorithm:',
                self.text_size,
                Position(0, int(0.6 * self.text_size)),
            )
            # self.draw_text(
            #    f'Maze file: {self.maze_path}',
            #    self.text_size,
            #    Position(0, self.tile_size + self.half_tile),
            # )
            self.draw_text(
                f'Entrance: {self.sim.begin[:2]} Facing: {self.sim.begin[2]}',
                self.text_size,
                Position(0, 2 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Steps: {self.sim.counter.current_step}',
                self.text_size,
                Position(middle_width - 6 * self.tile_size, 2 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Weight: {self.sim.counter.current_weight}',
                self.text_size,
                Position(middle_width, 2 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Cells: {self.sim.counter.current_cell}',
                self.text_size,
                Position(middle_width + 7 * self.tile_size, 2 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Goal(s): {', '.join(map(str, self.sim.end))}',
                self.text_size,
                Position(0, 3 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Total Steps: {self.sim.counter.total_step}',
                self.text_size,
                Position(middle_width - 6 * self.tile_size, 3 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Total Weight: {self.sim.counter.total_weight}',
                self.text_size,
                Position(middle_width, 3 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Total Cells: {self.sim.counter.total_cell}',
                self.text_size,
                Position(middle_width + 7 * self.tile_size, 3 * self.tile_size + self.half_tile),
            )
            self.draw_text(
                f'Simulation Delay [s]: {self.sim_speed_slider.get_current_value():.3f}',
                self.text_size,
                Position(middle_width + 7 * self.tile_size, 2 * self.text_size),
            )
            self.draw_text(
                f'Explored Cells: {self.sim.maze.explored_cells_count()}',
                self.text_size,
                Position(self.tile_size, self.full_maze_offset.col + self.sim.maze.height * (self.tile_size + 1)),
            )
            self.draw_text(
                f'Explored Cells Percentage: {self.sim.maze.explored_cells_percentage():.3%}',
                self.text_size,
                Position(self.robot_maze_offset.row, self.full_maze_offset.col + self.sim.maze.height * (self.tile_size + 1)),
            )

            self.update(time_delta)


def _main():
    sim = Simulator(
        alg=idle_robot,
        maze=Maze.from_file('mazes/semifinal_2010.maze'),
        begin=(15, 0, Direction.EAST),
        end={(7, 7), (7, 8), (8, 7), (8, 8)},
    )

    GUIRenderer(sim).run()


if __name__ == '__main__':
    _main()
