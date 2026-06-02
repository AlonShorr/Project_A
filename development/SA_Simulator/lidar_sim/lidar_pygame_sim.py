import pygame
import numpy as np
import math

from config import (
    MAP_WIDTH, MAP_HEIGHT, WINDOW_SIZE,
    GRID_SIZE, MAP_ROWS, MAP_COLS,
    SENSOR_ANGLES, DIR_TO_ANGLE,
    WALL_UP, WALL_DOWN, WALL_LEFT, WALL_RIGHT,
    LOCALIZATION_THRESHOLD, IS_LOST_THRESHOLD, GOAL, AUTO_STEP_DELAY,
    BLACK, WHITE, BLUE, GREEN, RED, GRAY, CYAN, YELLOW, ORANGE, SIDEBAR_BG,
)
from sim_robot import Robot
from localization import (
    precompute_all_orientations,
    update_probability,
    predict_motion,
    get_best_estimate,
    do_localization_step,
)
from motion_planning import plan_path
from map_builder import generate_wall_map, toggle_wall_click, dump_selected_cells


def main():
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Lidar Sim: WASD=Move, QE=Rotate, R=Auto, B=Builder, P=Analysis")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)
    small_font = pygame.font.SysFont("Arial", 14)

    wall_map = generate_wall_map()
    expected_data = precompute_all_orientations(wall_map)

    prob_matrix = np.ones((MAP_ROWS, MAP_COLS))
    prob_matrix /= MAP_ROWS * MAP_COLS

    robot = Robot(3, 3)

    analysis_mode = False
    selected_cells = set()
    building_mode = False

    AUTO_IDLE, AUTO_LOCALIZING, AUTO_PLANNING, AUTO_MOVING, AUTO_DONE = 0, 1, 2, 3, 4
    AUTO_STATE_LABELS = {0: "IDLE", 1: "LOCALIZING", 2: "PLANNING", 3: "MOVING", 4: "DONE"}
    auto_state = AUTO_IDLE
    planned_path = []
    loc_spin_count = 0
    loc_move_count = 0
    auto_step_delay = AUTO_STEP_DELAY
    last_auto_step_ms = 0
    goal = GOAL

    running = True
    while running:
        moved = False
        dr, dc = 0, 0

        # --- Event handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                if mx < MAP_WIDTH:
                    if analysis_mode:
                        c, r = mx // GRID_SIZE, my // GRID_SIZE
                        if 0 <= c < MAP_COLS and 0 <= r < MAP_ROWS:
                            coord = (r, c)
                            if coord in selected_cells:
                                selected_cells.remove(coord)
                            else:
                                selected_cells.add(coord)

                    elif building_mode:
                        toggle_wall_click(wall_map, mx, my)

                    else:
                        c, r = mx // GRID_SIZE, my // GRID_SIZE
                        if 0 <= c < MAP_COLS and 0 <= r < MAP_ROWS:
                            if event.button == 3:  # Right-click: set goal
                                goal = (r, c)
                                planned_path = []
                                if auto_state == AUTO_MOVING:
                                    auto_state = AUTO_PLANNING
                                print(f"Goal set to {goal}")
                            else:  # Left-click: teleport robot
                                robot.move_to(r, c)
                                prob_matrix.fill(1.0 / (MAP_ROWS * MAP_COLS))

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_b:
                    building_mode = not building_mode
                    if not building_mode:
                        screen.fill(BLACK)
                        txt = font.render("Recomputing Map Data... Please Wait...", True, WHITE)
                        screen.blit(txt, (MAP_WIDTH // 2 - 150, MAP_HEIGHT // 2))
                        pygame.display.flip()
                        expected_data = precompute_all_orientations(wall_map)
                        prob_matrix.fill(1.0 / (MAP_ROWS * MAP_COLS))

                if event.key == pygame.K_p and not building_mode:
                    if analysis_mode:
                        print("\n--- EXITING ANALYSIS MODE ---")
                        if selected_cells:
                            dump_selected_cells(expected_data, selected_cells)
                        else:
                            print("No cells selected.")
                        selected_cells.clear()
                        analysis_mode = False
                    else:
                        analysis_mode = True
                        print("\n--- ANALYSIS MODE STARTED ---")
                        print("1. Click cells on the map to mark them (Cyan).")
                        print("2. Press 'P' again to dump their distance data to console.")

                if event.key == pygame.K_r and not building_mode and not analysis_mode:
                    if auto_state in (AUTO_IDLE, AUTO_DONE):
                        auto_state = AUTO_LOCALIZING
                        loc_spin_count, loc_move_count = 0, 0
                        planned_path = []
                        print("--- AUTO MODE STARTED: Localizing... ---")
                    else:
                        auto_state = AUTO_IDLE
                        planned_path = []
                        print("--- AUTO MODE STOPPED ---")

                if event.key == pygame.K_LEFTBRACKET:
                    auto_step_delay = max(50, auto_step_delay - 50)
                elif event.key == pygame.K_RIGHTBRACKET:
                    auto_step_delay = min(2000, auto_step_delay + 50)

                if not building_mode and not analysis_mode and auto_state == AUTO_IDLE:
                    if event.key == pygame.K_w:   dr, dc = -1, 0
                    elif event.key == pygame.K_s: dr, dc = 1, 0
                    elif event.key == pygame.K_a: dr, dc = 0, -1
                    elif event.key == pygame.K_d: dr, dc = 0, 1
                    elif event.key == pygame.K_q: robot.rotate(-1)
                    elif event.key == pygame.K_e: robot.rotate(1)
                    elif event.key == pygame.K_1: robot.toggle_sensor(0)
                    elif event.key == pygame.K_2: robot.toggle_sensor(1)
                    elif event.key == pygame.K_3: robot.toggle_sensor(2)
                    elif event.key == pygame.K_4: robot.toggle_sensor(3)
                    elif event.key == pygame.K_5: robot.toggle_sensor(4)

                    if dr != 0 or dc != 0:
                        if robot.move_rel(dr, dc, wall_map):
                            moved = True

        # --- Simulation logic ---
        if not building_mode:
            eff_dr, eff_dc, eff_moved = 0, 0, False
            now = pygame.time.get_ticks()
            auto_step_ready = (now - last_auto_step_ms) >= auto_step_delay

            if auto_state == AUTO_LOCALIZING and auto_step_ready:
                loc_spin_count, loc_move_count, eff_dr, eff_dc = \
                    do_localization_step(robot, wall_map, loc_spin_count, loc_move_count)
                eff_moved = (eff_dr != 0 or eff_dc != 0)
                last_auto_step_ms = now

            elif auto_state == AUTO_PLANNING:
                est_r, est_c, _ = get_best_estimate(prob_matrix)
                path = plan_path((est_r, est_c, robot.angle_index), goal, wall_map)
                if path and len(path) > 1:
                    planned_path = list(path[1:])
                    auto_state = AUTO_MOVING
                    print(f"Path found: {len(planned_path)} steps to goal {goal}")
                elif auto_state != AUTO_IDLE:
                    print("A* returned no path. Stopping auto mode.")
                    auto_state = AUTO_IDLE

            elif auto_state == AUTO_MOVING and auto_step_ready:
                if not planned_path:
                    if (robot.r, robot.c) == goal:
                        auto_state = AUTO_DONE
                        print("--- GOAL REACHED! ---")
                    else:
                        auto_state = AUTO_PLANNING
                else:
                    target_r, target_c = planned_path[0]
                    step_dr = target_r - robot.r
                    step_dc = target_c - robot.c
                    required_angle = DIR_TO_ANGLE.get((step_dr, step_dc))
                    if required_angle is not None and robot.angle_index != required_angle:
                        delta = (required_angle - robot.angle_index) % 8
                        robot.rotate(1 if delta <= 4 else -1)
                        last_auto_step_ms = now
                    else:
                        if robot.move_rel(step_dr, step_dc, wall_map):
                            eff_dr, eff_dc = step_dr, step_dc
                            eff_moved = True
                            last_auto_step_ms = now
                        if robot.r == target_r and robot.c == target_c:
                            planned_path.pop(0)

            else:  # AUTO_IDLE or AUTO_DONE — manual control
                eff_dr, eff_dc = dr, dc
                eff_moved = moved

            if eff_moved:
                prob_matrix = predict_motion(prob_matrix, eff_dr, eff_dc, wall_map)

            dists, hit_points = robot.measure(wall_map)
            prob_matrix = update_probability(prob_matrix, dists, expected_data, robot.angle_index)

            if auto_state == AUTO_LOCALIZING:
                _, _, peak = get_best_estimate(prob_matrix)
                if peak >= LOCALIZATION_THRESHOLD:
                    print(f"Localized! peak={peak:.3f}. Planning path to {goal}...")
                    auto_state = AUTO_PLANNING
                    loc_spin_count, loc_move_count = 0, 0
            elif auto_state == AUTO_MOVING:
                _, _, peak = get_best_estimate(prob_matrix)
                if peak < IS_LOST_THRESHOLD:
                    print(f"Lost (peak={peak:.3f}). Re-localizing...")
                    auto_state = AUTO_LOCALIZING
                    loc_spin_count, loc_move_count = 0, 0
                    planned_path = []
        else:
            dists, hit_points = robot.measure(wall_map)

        # --- Drawing ---
        screen.fill(BLACK)

        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                rect = (c * GRID_SIZE, r * GRID_SIZE, GRID_SIZE, GRID_SIZE)

                if not building_mode and not analysis_mode:
                    b = min(int(prob_matrix[r, c] * 5000), 255)
                    if b > 10:
                        s = pygame.Surface((GRID_SIZE, GRID_SIZE))
                        s.set_alpha(150)
                        s.fill((b, 0, 0))
                        screen.blit(s, rect[:2])

                if (r, c) == goal:
                    goal_s = pygame.Surface((GRID_SIZE, GRID_SIZE))
                    goal_s.set_alpha(180)
                    goal_s.fill(YELLOW)
                    screen.blit(goal_s, rect[:2])

                grid_col = (60, 60, 0) if building_mode else (30, 30, 30)
                pygame.draw.rect(screen, grid_col, rect, 1)

                if analysis_mode and (r, c) in selected_cells:
                    pygame.draw.rect(screen, CYAN, rect)

                w_thick = 4 if building_mode else 2
                if wall_map[r, c, WALL_UP] == 0:
                    pygame.draw.line(screen, RED, (c * GRID_SIZE, r * GRID_SIZE), ((c + 1) * GRID_SIZE, r * GRID_SIZE), w_thick)
                if wall_map[r, c, WALL_DOWN] == 0:
                    pygame.draw.line(screen, RED, (c * GRID_SIZE, (r + 1) * GRID_SIZE), ((c + 1) * GRID_SIZE, (r + 1) * GRID_SIZE), w_thick)
                if wall_map[r, c, WALL_LEFT] == 0:
                    pygame.draw.line(screen, RED, (c * GRID_SIZE, r * GRID_SIZE), (c * GRID_SIZE, (r + 1) * GRID_SIZE), w_thick)
                if wall_map[r, c, WALL_RIGHT] == 0:
                    pygame.draw.line(screen, RED, ((c + 1) * GRID_SIZE, r * GRID_SIZE), ((c + 1) * GRID_SIZE, (r + 1) * GRID_SIZE), w_thick)

        if analysis_mode:
            pygame.draw.rect(screen, BLUE, (0, 0, MAP_WIDTH, MAP_HEIGHT), 5)

        pygame.draw.circle(screen, BLUE, (int(robot.true_x), int(robot.true_y)), 10)
        head_rad = math.radians(robot.angle_deg)
        head_x = robot.true_x + math.cos(head_rad) * 15
        head_y = robot.true_y + math.sin(head_rad) * 15
        pygame.draw.line(screen, WHITE, (robot.true_x, robot.true_y), (head_x, head_y), 2)

        for p in hit_points:
            pygame.draw.line(screen, GREEN, (robot.true_x, robot.true_y), p, 1)
            pygame.draw.circle(screen, GREEN, (int(p[0]), int(p[1])), 2)

        if planned_path:
            prev_px, prev_py = int(robot.true_x), int(robot.true_y)
            for wp_r, wp_c in planned_path:
                wp_x = int(wp_c * GRID_SIZE + GRID_SIZE // 2)
                wp_y = int(wp_r * GRID_SIZE + GRID_SIZE // 2)
                pygame.draw.line(screen, ORANGE, (prev_px, prev_py), (wp_x, wp_y), 2)
                pygame.draw.circle(screen, ORANGE, (wp_x, wp_y), 3)
                prev_px, prev_py = wp_x, wp_y

        # --- Sidebar ---
        pygame.draw.rect(screen, SIDEBAR_BG, pygame.Rect(MAP_WIDTH, 0, MAP_WIDTH, MAP_HEIGHT))
        pygame.draw.line(screen, WHITE, (MAP_WIDTH, 0), (MAP_WIDTH, MAP_HEIGHT), 2)

        x_start = MAP_WIDTH + 10
        y_off = 10

        if analysis_mode:
            mode_txt, mode_col = "ANALYSIS MODE", CYAN
        elif building_mode:
            mode_txt, mode_col = "BUILDER MODE", YELLOW
        elif auto_state == AUTO_DONE:
            mode_txt, mode_col = "GOAL REACHED!", GREEN
        elif auto_state != AUTO_IDLE:
            mode_txt, mode_col = f"AUTO: {AUTO_STATE_LABELS[auto_state]}", ORANGE
        else:
            mode_txt, mode_col = "SIMULATION MODE", GREEN

        screen.blit(font.render(mode_txt, True, mode_col), (x_start, y_off)); y_off += 30
        screen.blit(font.render("CONTROLS", True, GREEN), (x_start, y_off)); y_off += 25
        screen.blit(small_font.render("B: Toggle Builder", True, WHITE), (x_start, y_off)); y_off += 20
        screen.blit(small_font.render("P: Toggle Analysis", True, WHITE), (x_start, y_off)); y_off += 20

        if building_mode:
            screen.blit(small_font.render("Click Edges: Toggle Wall", True, YELLOW), (x_start, y_off)); y_off += 30
        elif analysis_mode:
            screen.blit(small_font.render("Click Cells: Select/Deselect", True, CYAN), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render("Press P again to Dump Data", True, CYAN), (x_start, y_off)); y_off += 30
        elif auto_state == AUTO_DONE:
            screen.blit(small_font.render(f"Goal {goal} reached!", True, GREEN), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render("R: Run Auto Again", True, WHITE), (x_start, y_off)); y_off += 30
        elif auto_state != AUTO_IDLE:
            screen.blit(small_font.render(f"Goal: row={goal[0]} col={goal[1]}", True, YELLOW), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render(f"Path: {len(planned_path)} steps left", True, WHITE), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render(f"Step delay: {auto_step_delay}ms  [ / ]", True, WHITE), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render("R: Stop Auto", True, ORANGE), (x_start, y_off)); y_off += 30
        else:
            screen.blit(small_font.render("WASD: Move  |  Q/E: Rotate", True, WHITE), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render("LClick: Teleport", True, WHITE), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render(f"RClick: Set Goal {goal}", True, YELLOW), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render(f"Step delay: {auto_step_delay}ms  [ / ]", True, WHITE), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render("R: Run Auto", True, WHITE), (x_start, y_off)); y_off += 30

        screen.blit(font.render("SENSORS (Keys 1-5)", True, GREEN), (x_start, y_off)); y_off += 25
        for i, active in enumerate(robot.active_sensors):
            color = GREEN if active else RED
            screen.blit(small_font.render(f"#{i+1} ({SENSOR_ANGLES[i]}°): {'ON' if active else 'OFF'}", True, color), (x_start, y_off))
            y_off += 20
        y_off += 10

        screen.blit(font.render("DIAGNOSTICS", True, GREEN), (x_start, y_off)); y_off += 25
        screen.blit(font.render(f"Heading: {robot.angle_deg}°", True, WHITE), (x_start, y_off)); y_off += 25
        screen.blit(font.render("Top Probable Locs:", True, GREEN), (x_start, y_off)); y_off += 25

        if not building_mode and not analysis_mode:
            count = 0
            for idx in np.argsort(prob_matrix.ravel())[::-1]:
                r_idx, c_idx = divmod(idx, MAP_COLS)
                prob = prob_matrix[r_idx, c_idx]
                if prob < 0.005 or count >= 8:
                    break
                text_str = f"({r_idx}, {c_idx}): {prob:.3f}"
                color = WHITE
                if r_idx == robot.r and c_idx == robot.c:
                    color = GREEN
                    text_str += " <--"
                screen.blit(small_font.render(text_str, True, color), (x_start, y_off))
                y_off += 20
                count += 1
        elif analysis_mode:
            screen.blit(small_font.render(f"Selected: {len(selected_cells)}", True, CYAN), (x_start, y_off))
        else:
            screen.blit(small_font.render("Paused...", True, GRAY), (x_start, y_off))

        y_off = MAP_HEIGHT - 60
        screen.blit(small_font.render("Map Legend:", True, GREEN), (x_start, y_off)); y_off += 20
        screen.blit(small_font.render("Red Intensity = Probability", True, RED), (x_start, y_off))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
