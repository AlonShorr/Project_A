import numpy as np
import matplotlib.pyplot as plt

# --- 1. Robot Class ---
class RealRobot:
    def __init__(self, start_r, start_c, cell_size_mm=180):
        self.r = start_r
        self.c = start_c
        self.cell_size = cell_size_mm
        
        # VL53L4CD Sensor Specs
        self.max_range_mm = 1300.0
        self.min_range_mm = 1.0
        
    def measure(self, dist_matrix):
        """Simulates VL53L4CD ToF Sensor readings with noise."""#
        # Get grid distance (Cells)
        grid_dist = dist_matrix[self.r, self.c]
        
        # Convert to mm: (cells * 180) + (90mm to center of cell)
        true_mm = (grid_dist * self.cell_size) + (self.cell_size / 2.0)
        
        # Add 5% Noise (min 10mm sigma)
        sigma = np.maximum(0.05 * true_mm, 10.0)
        noise = np.random.normal(0, sigma, size=4)
        measured_mm = true_mm + noise
        
        # Clip to sensor limits
        measured_mm = np.clip(measured_mm, self.min_range_mm, self.max_range_mm)
        
        return true_mm, measured_mm.astype(int)

    def move(self, direction_idx, maze):
        """Moves the robot if no wall exists."""
        dirs = {0: (-1, 0), 1: (1, 0), 2: (0, -1), 3: (0, 1)}
        dr, dc = dirs[direction_idx]
        
        # Check wall in the direction of movement
        if maze[self.r, self.c, direction_idx] == 0:
            print(f"CRASH! Wall at direction {direction_idx}")
            return False
            
        self.r += dr
        self.c += dc
        return True

# --- 2. Map Generation & Pre-computation ---

def generate_wall_maze():
    """
    Creates a 5x5 maze where walls are defined on the edges of cells.
    Data Structure: (Rows, Cols, 4)
    Indices: 0=Up, 1=Down, 2=Left, 3=Right
    Values: 0=Wall, 1=Open
    """
    H, W = 5, 5
    maze = np.ones((H, W, 4), dtype=int)
    
    def add_v_wall(r, c):
        maze[r, c, 3] = 0 
        if c+1 < W: maze[r, c+1, 2] = 0 

    def add_h_wall(r, c):
        maze[r, c, 1] = 0 
        if r+1 < H: maze[r+1, c, 0] = 0 

    maze[0, :, 0] = 0   # Top
    maze[H-1, :, 1] = 0 # Bottom
    maze[:, 0, 2] = 0   # Left
    maze[:, W-1, 3] = 0 # Right
    
    # # Internal Walls
    # add_v_wall(0, 0); add_v_wall(0, 2); add_h_wall(0, 1)
    # add_v_wall(1, 0); add_v_wall(1, 3); add_h_wall(1, 2)
    # add_h_wall(2, 0); add_h_wall(2, 4); add_v_wall(2, 2)
    # add_v_wall(3, 1); add_h_wall(3, 3)
    # add_v_wall(4, 2)
    
    # mapa Adira
    add_v_wall(1,1); add_v_wall(1,2);  add_v_wall(3,1);  add_v_wall(3,2); 
    add_h_wall(1,1); add_h_wall(2,1); add_h_wall(1,3); add_h_wall(2,3)
    
    return maze

def calculate_4way_distances(maze):
    """Computes distance to nearest wall in 4 directions (in Cells)."""
    height, width, _ = maze.shape
    dist_matrix = np.zeros((height, width, 4), dtype=int)
    directions = {0: (-1, 0), 1: (1, 0), 2: (0, -1), 3: (0, 1)}
    
    for r in range(height):
        for c in range(width):
            for dir_idx, (dr, dc) in directions.items():
                curr_r, curr_c = r, c
                distance = 0
                while True:
                    if maze[curr_r, curr_c, dir_idx] == 0:
                        break
                    curr_r += dr
                    curr_c += dc
                    if not (0 <= curr_r < height and 0 <= curr_c < width):
                        break
                    distance += 1
                dist_matrix[r, c, dir_idx] = distance
    return dist_matrix

# --- 3. Localization Logic ---

def initialize_probabilities(rows, cols):
    """Start with uniform distribution (Robot could be anywhere)."""
    p = 1.0 / (rows * cols)
    return np.full((rows, cols), p)

def update_probabilities(prob_matrix, measured_mm, dist_matrix_cells, cell_size_mm=180):
    """
    Correction Step (Bayes Update).
    Calculates P(Measurement | Location) for every cell.
    """
    rows, cols = prob_matrix.shape
    new_prob = np.zeros_like(prob_matrix)
    
    for r in range(rows):
        for c in range(cols):
            # 1. Prediction: What SHOULD we see at this cell?
            expected_cells = dist_matrix_cells[r, c]
            expected_mm = (expected_cells * cell_size_mm) + (cell_size_mm / 2.0)
            
            # 2. Likelihood: How close is measured_mm to expected_mm?
            likelihood = 1.0
            for i in range(4): # For Up, Down, Left, Right
                mu = expected_mm[i]
                sigma = max(mu * 0.05, 20.0) # 5% error, min 20mm
                x = measured_mm[i]
                
                # Gaussian PDF
                prob_direction = np.exp(-((x - mu) ** 2) / (2 * sigma ** 2))
                likelihood *= prob_direction
            
            # 3. Update
            new_prob[r, c] = prob_matrix[r, c] * likelihood
            
    # 4. Normalize
    total_prob = np.sum(new_prob)
    if total_prob > 0:
        new_prob /= total_prob
    else:
        new_prob = initialize_probabilities(rows, cols)
        
    return new_prob

def predict_motion(prob_matrix, direction_idx):
    """
    Prediction Step (Motion Update).
    Shifts the probability cloud in the direction of movement.
    0=Up, 1=Down, 2=Left, 3=Right
    """
    rows, cols = prob_matrix.shape
    new_prob = np.zeros_like(prob_matrix)
    
    # Mapping direction index to delta row/col
    dr, dc = {0: (-1, 0), 1: (1, 0), 2: (0, -1), 3: (0, 1)}[direction_idx]
    
    for r in range(rows):
        for c in range(cols):
            if prob_matrix[r, c] > 0.0001: # Optimization: only move meaningful prob
                # Calculate where this particle moves
                nr, nc = r + dr, c + dc
                
                # Check if new position is within bounds
                # (Ideally we also check wall collisions here, but for simple shifting:)
                if 0 <= nr < rows and 0 <= nc < cols:
                    new_prob[nr, nc] += prob_matrix[r, c]
                    
    # Normalize (handle mass lost to walls/boundaries)
    s = np.sum(new_prob)
    if s > 0:
        new_prob /= s
    else:
        # If we hit a wall perfectly, maybe we are lost, or stay put?
        # For this sim, reset if empty, or just return existing if blocked (simpler)
        pass 
        
    return new_prob

# --- 4. Visualization Functions ---

def visualize_simulation(maze, prob_matrix, true_robot, measurement):
    """
    Main visualizer: Shows the Real World (Walls + Robot) and the Robot's Belief (Heatmap).
    """
    plt.clf() # Clear the figure to allow dynamic updates
    
    H, W, _ = maze.shape
    
    # Create subplots manually to control layout on the existing figure
    ax1 = plt.subplot(1, 2, 1)
    ax2 = plt.subplot(1, 2, 2)
    
    # --- Plot 1: Reality ---
    ax1.set_title("Reality: Robot Position")
    ax1.set_xlim(-0.5, W-0.5)
    ax1.set_ylim(H-0.5, -0.5)
    ax1.set_aspect('equal')
    ax1.grid(True, color='lightgray', linestyle='--')
    
    # Draw Walls
    for r in range(H):
        for c in range(W):
            if maze[r, c, 0] == 0: ax1.plot([c-0.5, c+0.5], [r-0.5, r-0.5], 'r-', lw=2)
            if maze[r, c, 1] == 0: ax1.plot([c-0.5, c+0.5], [r+0.5, r+0.5], 'r-', lw=2)
            if maze[r, c, 2] == 0: ax1.plot([c-0.5, c-0.5], [r-0.5, r+0.5], 'r-', lw=2)
            if maze[r, c, 3] == 0: ax1.plot([c+0.5, c+0.5], [r-0.5, r+0.5], 'r-', lw=2)
            
    # Draw Robot
    ax1.plot(true_robot.c, true_robot.r, 'bo', markersize=12, label='Robot')
    
    # --- Plot 2: Probability ---
    ax2.set_title("Robot's Brain: Probability Map")
    im = ax2.imshow(prob_matrix, cmap='Reds', vmin=0, vmax=1)
    
    # Show wall outlines faintly on probability map for reference
    for r in range(H):
        for c in range(W):
            if maze[r, c, 0] == 0: ax2.plot([c-0.5, c+0.5], [r-0.5, r-0.5], 'k-', lw=0.5, alpha=0.3)
            if maze[r, c, 1] == 0: ax2.plot([c-0.5, c+0.5], [r+0.5, r+0.5], 'k-', lw=0.5, alpha=0.3)
            if maze[r, c, 2] == 0: ax2.plot([c-0.5, c-0.5], [r-0.5, r+0.5], 'k-', lw=0.5, alpha=0.3)
            if maze[r, c, 3] == 0: ax2.plot([c+0.5, c+0.5], [r-0.5, r+0.5], 'k-', lw=0.5, alpha=0.3)

    # Info Text
    info_text = (f"Measurement (mm):\n"
                 f"Up: {measurement[0]}\n"
                 f"Down: {measurement[1]}\n"
                 f"Left: {measurement[2]}\n"
                 f"Right: {measurement[3]}")
    plt.figtext(0.02, 0.02, info_text, fontsize=10, bbox=dict(facecolor='white', alpha=0.8))

    plt.draw()
    plt.pause(0.1) # Pause to let the GUI update

if __name__ == "__main__":
    # 1. Init Map
    my_maze = generate_wall_maze()
    dist_matrix = calculate_4way_distances(my_maze)
    probabilities = initialize_probabilities(5, 5)
    
    # 2. User Setup
    print("--- LIDAR SIMULATION ---")
    try:
        user_in = input("Enter start position as 'row,col' (e.g., 2,2): ")
        r_in, c_in = map(int, user_in.split(','))
        start_r, start_c = r_in, c_in
    except ValueError:
        print("Invalid input. Defaulting to 2,2")
        start_r, start_c = 2, 2

    robot = RealRobot(start_r, start_c)
    print(f"Robot placed at ({start_r}, {start_c})")
    print("Controls: 'w'=Up, 's'=Down, 'a'=Left, 'd'=Right, 'q'=Quit")

    # 3. Setup Interactive Plot
    plt.ion() # Interactive mode on
    plt.figure(figsize=(12, 5))

    while True:
        # --- A. Measurement Step ---
        true_dists, measured_dists = robot.measure(dist_matrix)
        
        # --- B. Correction Step (Update Belief based on Measurement) ---
        probabilities = update_probabilities(probabilities, measured_dists, dist_matrix)
        
        # --- C. Visualize ---
        visualize_simulation(my_maze, probabilities, robot, measured_dists)
        
        # --- D. User Input ---
        cmd = input("\nAction [w/a/s/d] or q: ").strip().lower()
        
        if cmd == 'q':
            break
            
        move_map = {'w': 0, 's': 1, 'a': 2, 'd': 3}
        
        if cmd in move_map:
            dir_idx = move_map[cmd]
            
            # --- E. Move Robot ---
            success = robot.move(dir_idx, my_maze)
            
            if success:
                # --- F. Prediction Step (Shift Belief based on Move) ---
                probabilities = predict_motion(probabilities, dir_idx)
                print("Moved.")
            else:
                print("Blocked by wall!")
        else:
            print("Unknown command.")

    print("Simulation ended.")
    plt.close()