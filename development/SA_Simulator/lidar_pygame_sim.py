import pygame
import numpy as np
import math

# --- CONFIGURATION ---
MAP_WIDTH = 800
MAP_HEIGHT = 600
SIDEBAR_WIDTH = 250
WINDOW_SIZE = (MAP_WIDTH + SIDEBAR_WIDTH, MAP_HEIGHT)

GRID_SIZE = 40  # pixels
MAP_COLS = MAP_WIDTH // GRID_SIZE
MAP_ROWS = MAP_HEIGHT // GRID_SIZE

# Physics / Sensor Config
CELL_SIZE_MM = 180.0
PIXELS_PER_MM = GRID_SIZE / CELL_SIZE_MM
MAX_RANGE_MM = 1300.0
MAX_RANGE_PX = MAX_RANGE_MM * PIXELS_PER_MM

# Angles
# Shifted so 0 is the center/forward sensor (Old 90 is now 0)
SENSOR_ANGLES = [-90, -40.33, 0, 37.22, 90] # Relative to robot facing
ROTATION_STEP = 45 # Degrees per Q/E press

# Map Wall Indices
WALL_UP = 0
WALL_DOWN = 1
WALL_LEFT = 2
WALL_RIGHT = 3

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 100, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY = (50, 50, 50)
DARK_RED = (100, 0, 0)
YELLOW = (255, 255, 0)
SIDEBAR_BG = (30, 30, 30)

class Robot:
    def __init__(self, r, c):
        self.r = r # position in grid cells
        self.c = c
        self.true_x = (c * GRID_SIZE) + (GRID_SIZE / 2) # position in pixels 
        self.true_y = (r * GRID_SIZE) + (GRID_SIZE / 2)
        
        # 0=0deg, 1=45deg, 2=90deg ... 7=315deg
        # Start at 90deg (Up) so the robot faces the direction of 'W' movement
        self.angle_index = 2 
        
        # Active state for sensors (Default all ON)
        self.active_sensors = [True] * len(SENSOR_ANGLES)
        
    @property
    def angle_deg(self):
        return self.angle_index * ROTATION_STEP

    def rotate(self, direction):
        """ direction: +1 (Left/Q), -1 (Right/E) """
        self.angle_index = (self.angle_index + direction) % 8

    def toggle_sensor(self, idx):
        if 0 <= idx < len(self.active_sensors):
            self.active_sensors[idx] = not self.active_sensors[idx]

    def move_to(self, r, c):
        self.r = r
        self.c = c
        self.true_x = (c * GRID_SIZE) + (GRID_SIZE / 2)
        self.true_y = (r * GRID_SIZE) + (GRID_SIZE / 2)

    def move_rel(self, dr, dc, wall_map):
        """Move checking Thin Walls"""
        # 1. Determine which wall to check based on direction
        # Walls: 0=Up, 1=Down, 2=Left, 3=Right. Values: 0=Wall, 1=Open
        
        # Moving Up (dr = -1)
        if dr == -1:
            if wall_map[self.r, self.c, WALL_UP] == 0: return False
            if self.r - 1 < 0: return False
            
        # Moving Down (dr = 1)
        if dr == 1:
            if wall_map[self.r, self.c, WALL_DOWN] == 0: return False
            if self.r + 1 >= MAP_ROWS: return False
            
        # Moving Left (dc = -1)
        if dc == -1:
            if wall_map[self.r, self.c, WALL_LEFT] == 0: return False
            if self.c - 1 < 0: return False
            
        # Moving Right (dc = 1)
        if dc == 1:
            if wall_map[self.r, self.c, WALL_RIGHT] == 0: return False
            if self.c + 1 >= MAP_COLS: return False

        # If safe, update position
        self.move_to(self.r + dr, self.c + dc)
        return True

    def measure(self, wall_map):
        """
        Raycasting for Thin Walls.
        Returns distances in PIXELS.
        If a sensor is disabled, returns None for that index.
        """
        measurements = []
        hit_points = []
        
        robot_angle = self.angle_deg

        for i, sensor_angle in enumerate(SENSOR_ANGLES):
            # Check if sensor is active
            if not self.active_sensors[i]:
                measurements.append(None)
                continue

            # Combined angle
            total_angle = (robot_angle + sensor_angle) % 360
            rad = math.radians(total_angle)
            
            dx = math.cos(rad)
            dy = -math.sin(rad) # Y is inverted in PyGame
            
            curr_x, curr_y = self.true_x, self.true_y
            dist_travelled = 0
            hit = False
            
            # Ray Marching Steps
            step_size = 2.0 
            
            while dist_travelled < MAX_RANGE_PX:
                next_x = curr_x + dx * step_size
                next_y = curr_y + dy * step_size
                
                # Calculate Grid Coords
                curr_c, curr_r = int(curr_x // GRID_SIZE), int(curr_y // GRID_SIZE)
                next_c, next_r = int(next_x // GRID_SIZE), int(next_y // GRID_SIZE)
                
                # 1. Check Map Bounds
                if not (0 <= next_c < MAP_COLS and 0 <= next_r < MAP_ROWS):
                    hit = True
                    break
                
                # 2. Check Boundary Crossing (Thin Walls)
                # Did we cross a vertical grid line?
                if next_c != curr_c:
                    # Moving Right
                    if next_c > curr_c:
                        if wall_map[curr_r, curr_c, WALL_RIGHT] == 0:
                            hit = True; break
                    # Moving Left
                    else:
                        if wall_map[curr_r, curr_c, WALL_LEFT] == 0:
                            hit = True; break
                            
                # Did we cross a horizontal grid line?
                if next_r != curr_r:
                    # Moving Down
                    if next_r > curr_r:
                        if wall_map[curr_r, curr_c, WALL_DOWN] == 0:
                            hit = True; break
                    # Moving Up
                    else:
                        if wall_map[curr_r, curr_c, WALL_UP] == 0:
                            hit = True; break
                
                # Update
                curr_x, curr_y = next_x, next_y
                dist_travelled += step_size
                
            measurements.append(dist_travelled)
            hit_points.append((curr_x, curr_y))
            
        return measurements, hit_points

def generate_wall_map():
    """
    Generates map with Thin Walls.
    Shape: (Rows, Cols, 4). 0=Wall, 1=Open.
    Indices: 0=Up, 1=Down, 2=Left, 3=Right.
    """
    maze = np.ones((MAP_ROWS, MAP_COLS, 4), dtype=int)
    
    # Helpers
    def set_v_wall(r, c):
        """Wall to the Right of (r,c)"""
        maze[r, c, WALL_RIGHT] = 0
        if c + 1 < MAP_COLS: maze[r, c+1, WALL_LEFT] = 0
        
    def set_h_wall(r, c):
        """Wall Below (r,c)"""
        maze[r, c, WALL_DOWN] = 0
        if r + 1 < MAP_ROWS: maze[r+1, c, WALL_UP] = 0
        
    # Borders
    maze[0, :, WALL_UP] = 0
    maze[-1, :, WALL_DOWN] = 0
    maze[:, 0, WALL_LEFT] = 0
    maze[:, -1, WALL_RIGHT] = 0
    
    # Custom Walls (Example Layout)
    # A box in the top left
    set_v_wall(2, 2); set_v_wall(3, 2)
    set_h_wall(3, 2); set_h_wall(3, 3)
    
    # A long wall in the middle
    for i in range(5, 15):
        set_h_wall(8, i)
        
    # Some random partitions
    set_v_wall(5, 10); set_v_wall(6, 10)
    
    return maze

def precompute_all_orientations(wall_map):
    """
    Pre-computes expected sensor readings for:
    Every Cell (Rows x Cols) AND Every Rotation (8 angles).
    Structure: [Row, Col, AngleIdx, SensorIdx]
    """
    print("Pre-computing for 8 orientations... (Wait for it)")
    rows, cols, _ = wall_map.shape
    num_angles = 8
    
    # Shape: (Rows, Cols, 8, 5)
    data = np.zeros((rows, cols, num_angles, len(SENSOR_ANGLES)))
    
    v_bot = Robot(0, 0)
    
    for r in range(rows):
        for c in range(cols):
            v_bot.move_to(r, c)
            
            for ang_idx in range(num_angles):
                v_bot.angle_index = ang_idx
                # Ensure virtual bot has all sensors on for precomputation
                v_bot.active_sensors = [True] * len(SENSOR_ANGLES)
                dists, _ = v_bot.measure(wall_map)
                data[r, c, ang_idx] = dists
                
    print("Pre-computation Complete.")
    return data

def update_probability(prob_matrix, measured_dists, expected_data, current_angle_idx):
    """
    Bayes Update using the slice of expected data corresponding 
    to the robot's current rotation.
    """
    rows, cols = prob_matrix.shape
    new_prob = np.zeros_like(prob_matrix)
    sigma = 30.0 # Pixels
    
    for r in range(rows):
        for c in range(cols):
            if prob_matrix[r, c] < 0.00001: continue
            
            # Fetch expectations for the CURRENT angle
            expected_dists = expected_data[r, c, current_angle_idx]
            
            likelihood = 1.0
            
            for i in range(len(SENSOR_ANGLES)):
                z = measured_dists[i]
                
                # Skip disabled sensors (z is None)
                if z is None:
                    continue
                
                mu = expected_dists[i]
                
                # If we expect to hit a wall instantly (0 dist), avoid bad math
                if mu < 1: mu = 1
                
                # Gaussian
                p = np.exp(-((z - mu) ** 2) / (2 * sigma ** 2))
                likelihood *= p
            
            new_prob[r, c] = prob_matrix[r, c] * likelihood
            
    total = np.sum(new_prob)
    if total > 0:
        new_prob /= total
    else:
        new_prob.fill(1.0 / np.count_nonzero(prob_matrix))
        
    return new_prob

def predict_motion(prob_matrix, dr, dc, wall_map):
    """Shift probabilities"""
    rows, cols = prob_matrix.shape
    new_prob = np.zeros_like(prob_matrix)
    
    for r in range(rows):
        for c in range(cols):
            if prob_matrix[r, c] > 0.0001:
                # Check walls for particle movement
                blocked = False
                
                # Logic mirroring Robot.move_rel
                if dr == -1 and (wall_map[r,c,WALL_UP]==0 or r-1<0): blocked=True
                elif dr == 1 and (wall_map[r,c,WALL_DOWN]==0 or r+1>=rows): blocked=True
                elif dc == -1 and (wall_map[r,c,WALL_LEFT]==0 or c-1<0): blocked=True
                elif dc == 1 and (wall_map[r,c,WALL_RIGHT]==0 or c+1>=cols): blocked=True
                
                if not blocked:
                    new_prob[r+dr, c+dc] += prob_matrix[r, c]
                else:
                    new_prob[r, c] += prob_matrix[r, c]

    # Blur
    kernel = np.array([[0.05, 0.1, 0.05], [0.1, 0.4, 0.1], [0.05, 0.1, 0.05]])
    blurred = np.zeros_like(new_prob)
    for r in range(1, rows-1):
        for c in range(1, cols-1):
            patch = new_prob[r-1:r+2, c-1:c+2]
            blurred[r,c] = np.sum(patch * kernel)
            
    s = np.sum(blurred)
    if s > 0: blurred /= s
    return blurred

def toggle_wall_click(wall_map, mx, my):
    """
    Toggles the wall nearest to the mouse click (mx, my).
    Syncs the wall change between adjacent cells.
    """
    if not (0 <= mx < MAP_WIDTH and 0 <= my < MAP_HEIGHT):
        return

    c = mx // GRID_SIZE
    r = my // GRID_SIZE
    
    # Calculate offset within the cell
    lx = mx % GRID_SIZE
    ly = my % GRID_SIZE
    
    # Distances to each edge
    d_left = lx
    d_right = GRID_SIZE - lx
    d_up = ly
    d_down = GRID_SIZE - ly
    
    min_dist = min(d_left, d_right, d_up, d_down)
    
    # Threshold in pixels to click a wall
    CLICK_THRESHOLD = 15
    if min_dist > CLICK_THRESHOLD:
        return
        
    # Toggle the specific wall (0->1 or 1->0)
    # And ensure the neighbor's corresponding wall is updated
    if min_dist == d_up:
        val = 1 - wall_map[r, c, WALL_UP]
        wall_map[r, c, WALL_UP] = val
        if r > 0: 
            wall_map[r-1, c, WALL_DOWN] = val
            
    elif min_dist == d_down:
        val = 1 - wall_map[r, c, WALL_DOWN]
        wall_map[r, c, WALL_DOWN] = val
        if r < MAP_ROWS - 1: 
            wall_map[r+1, c, WALL_UP] = val
            
    elif min_dist == d_left:
        val = 1 - wall_map[r, c, WALL_LEFT]
        wall_map[r, c, WALL_LEFT] = val
        if c > 0: 
            wall_map[r, c-1, WALL_RIGHT] = val
            
    elif min_dist == d_right:
        val = 1 - wall_map[r, c, WALL_RIGHT]
        wall_map[r, c, WALL_RIGHT] = val
        if c < MAP_COLS - 1: 
            wall_map[r, c+1, WALL_LEFT] = val

def main():
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Lidar Sim: WASD=Move, QE=Rotate, B=Builder Mode")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)
    small_font = pygame.font.SysFont("Arial", 14)

    # 1. Setup
    wall_map = generate_wall_map()
    expected_data = precompute_all_orientations(wall_map)
    
    # Init Prob
    prob_matrix = np.ones((MAP_ROWS, MAP_COLS))
    prob_matrix /= (MAP_ROWS * MAP_COLS)
    
    robot = Robot(3, 3)
    
    building_mode = False

    running = True
    while running:
        moved = False
        dr, dc = 0, 0
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                
                # Only handle clicks within the map area
                if mx < MAP_WIDTH:
                    if building_mode:
                        toggle_wall_click(wall_map, mx, my)
                    else:
                        # Teleport
                        c, r = mx // GRID_SIZE, my // GRID_SIZE
                        if 0 <= c < MAP_COLS and 0 <= r < MAP_ROWS:
                            robot.move_to(r, c)
                            prob_matrix.fill(1.0 / (MAP_ROWS*MAP_COLS))
            
            elif event.type == pygame.KEYDOWN:
                # Toggle Builder Mode
                if event.key == pygame.K_b:
                    building_mode = not building_mode
                    if not building_mode:
                        # Exit Builder: Recompute everything
                        screen.fill(BLACK)
                        txt = font.render("Recomputing Map Data... Please Wait...", True, WHITE)
                        screen.blit(txt, (MAP_WIDTH//2 - 150, MAP_HEIGHT//2))
                        pygame.display.flip()
                        
                        expected_data = precompute_all_orientations(wall_map)
                        prob_matrix.fill(1.0 / (MAP_ROWS*MAP_COLS))
                
                if not building_mode:
                    if event.key == pygame.K_w:   dr, dc = -1, 0
                    elif event.key == pygame.K_s: dr, dc = 1, 0
                    elif event.key == pygame.K_a: dr, dc = 0, -1
                    elif event.key == pygame.K_d: dr, dc = 0, 1
                    elif event.key == pygame.K_q: 
                        robot.rotate(1) # Left
                    elif event.key == pygame.K_e: 
                        robot.rotate(-1) # Right
                    
                    # Sensor Toggling (1-5)
                    elif event.key == pygame.K_1: robot.toggle_sensor(0)
                    elif event.key == pygame.K_2: robot.toggle_sensor(1)
                    elif event.key == pygame.K_3: robot.toggle_sensor(2)
                    elif event.key == pygame.K_4: robot.toggle_sensor(3)
                    elif event.key == pygame.K_5: robot.toggle_sensor(4)

                    if dr != 0 or dc != 0:
                        if robot.move_rel(dr, dc, wall_map):
                            moved = True

        # Logic
        if not building_mode:
            if moved:
                prob_matrix = predict_motion(prob_matrix, dr, dc, wall_map)
                
            dists, hit_points = robot.measure(wall_map)
            prob_matrix = update_probability(prob_matrix, dists, expected_data, robot.angle_index)
        else:
            # In builder mode, we can still show robot/rays, but no physics updates
            dists, hit_points = robot.measure(wall_map)

        # Draw
        screen.fill(BLACK)
        
        # --- Draw Map Area ---
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                rect = (c*GRID_SIZE, r*GRID_SIZE, GRID_SIZE, GRID_SIZE)
                
                # Draw Prob (Only in Sim Mode)
                if not building_mode:
                    p = prob_matrix[r, c]
                    b = min(int(p * 5000), 255)
                    if b > 10:
                        s = pygame.Surface((GRID_SIZE, GRID_SIZE))
                        s.set_alpha(150)
                        s.fill((b, 0, 0))
                        screen.blit(s, rect[:2])
                
                # Draw Grid
                grid_col = (60, 60, 0) if building_mode else (30, 30, 30)
                pygame.draw.rect(screen, grid_col, rect, 1)
                
                # Draw Walls (Red Lines)
                # Make walls thicker in builder mode to see easier
                w_thick = 4 if building_mode else 2
                
                if wall_map[r, c, WALL_UP] == 0:
                    pygame.draw.line(screen, RED, (c*GRID_SIZE, r*GRID_SIZE), ((c+1)*GRID_SIZE, r*GRID_SIZE), w_thick)
                if wall_map[r, c, WALL_DOWN] == 0:
                    pygame.draw.line(screen, RED, (c*GRID_SIZE, (r+1)*GRID_SIZE), ((c+1)*GRID_SIZE, (r+1)*GRID_SIZE), w_thick)
                if wall_map[r, c, WALL_LEFT] == 0:
                    pygame.draw.line(screen, RED, (c*GRID_SIZE, r*GRID_SIZE), (c*GRID_SIZE, (r+1)*GRID_SIZE), w_thick)
                if wall_map[r, c, WALL_RIGHT] == 0:
                    pygame.draw.line(screen, RED, ((c+1)*GRID_SIZE, r*GRID_SIZE), ((c+1)*GRID_SIZE, (r+1)*GRID_SIZE), w_thick)

        # Robot & Rays
        pygame.draw.circle(screen, BLUE, (int(robot.true_x), int(robot.true_y)), 10)
        head_rad = math.radians(robot.angle_deg)
        head_x = robot.true_x + math.cos(head_rad) * 15
        head_y = robot.true_y - math.sin(head_rad) * 15
        pygame.draw.line(screen, WHITE, (robot.true_x, robot.true_y), (head_x, head_y), 2)
        
        for p in hit_points:
            pygame.draw.line(screen, GREEN, (robot.true_x, robot.true_y), p, 1)
            pygame.draw.circle(screen, GREEN, (int(p[0]), int(p[1])), 2)

        # --- Draw Sidebar (Diagnostics) ---
        sidebar_rect = pygame.Rect(MAP_WIDTH, 0, SIDEBAR_WIDTH, MAP_HEIGHT)
        pygame.draw.rect(screen, SIDEBAR_BG, sidebar_rect)
        pygame.draw.line(screen, WHITE, (MAP_WIDTH, 0), (MAP_WIDTH, MAP_HEIGHT), 2)
        
        # Info Text
        x_start = MAP_WIDTH + 10
        y_off = 10
        
        # Mode Status
        mode_txt = "BUILDER MODE" if building_mode else "SIMULATION MODE"
        mode_col = YELLOW if building_mode else GREEN
        screen.blit(font.render(mode_txt, True, mode_col), (x_start, y_off)); y_off += 30
        
        screen.blit(font.render("CONTROLS", True, GREEN), (x_start, y_off)); y_off += 25
        screen.blit(small_font.render("B: Toggle Builder/Sim", True, WHITE), (x_start, y_off)); y_off += 20
        if building_mode:
            screen.blit(small_font.render("Click Edges: Toggle Wall", True, YELLOW), (x_start, y_off)); y_off += 30
        else:
            screen.blit(small_font.render("WASD: Move  |  Q/E: Rotate", True, WHITE), (x_start, y_off)); y_off += 20
            screen.blit(small_font.render("Mouse: Teleport", True, WHITE), (x_start, y_off)); y_off += 30
        
        # Sensor Status
        screen.blit(font.render("SENSORS (Keys 1-5)", True, GREEN), (x_start, y_off)); y_off += 25
        for i, active in enumerate(robot.active_sensors):
            status = "ON" if active else "OFF"
            color = GREEN if active else RED
            txt = f"#{i+1} ({SENSOR_ANGLES[i]}°): {status}"
            screen.blit(small_font.render(txt, True, color), (x_start, y_off))
            y_off += 20
        y_off += 10

        screen.blit(font.render("DIAGNOSTICS", True, GREEN), (x_start, y_off)); y_off += 25
        screen.blit(font.render(f"Heading: {robot.angle_deg}°", True, WHITE), (x_start, y_off)); y_off += 25
        
        # Top Estimates
        screen.blit(font.render("Top Probable Locs:", True, GREEN), (x_start, y_off)); y_off += 25
        
        if not building_mode:
            # Flatten and sort probabilities
            flat_indices = np.argsort(prob_matrix.ravel())[::-1]
            
            count = 0
            for idx in flat_indices:
                r_idx, c_idx = divmod(idx, MAP_COLS)
                prob = prob_matrix[r_idx, c_idx]
                
                if prob < 0.005 or count >= 8: # Limit list size
                    break
                
                # Format text
                text_str = f"({r_idx}, {c_idx}): {prob:.3f}"
                color = WHITE
                
                # Highlight correct position (Cheat/Debug view)
                if r_idx == robot.r and c_idx == robot.c:
                    color = GREEN
                    text_str += " <--"
                
                screen.blit(small_font.render(text_str, True, color), (x_start, y_off))
                y_off += 20
                count += 1
        else:
             screen.blit(small_font.render("Paused...", True, GRAY), (x_start, y_off))
            
        # Legend
        y_off = MAP_HEIGHT - 60
        screen.blit(small_font.render("Map Legend:", True, GREEN), (x_start, y_off)); y_off += 20
        screen.blit(small_font.render("Red Intensity = Probability", True, RED), (x_start, y_off)); y_off += 20
        
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    main()