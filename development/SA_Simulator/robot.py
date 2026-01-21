import numpy as np

class RealRobot:
    def __init__(self, start_r, start_c, cell_size_mm=180):
        self.r = start_r
        self.c = start_c
        self.cell_size = cell_size_mm
        
        # VL53L4CD Sensor Specs
        self.max_range_mm = 1300.0
        self.min_range_mm = 1.0
        
    def measure(self, dist_matrix):
        """Simulates VL53L4CD ToF Sensor readings with noise."""
        # Get grid distance
        grid_dist = dist_matrix[self.r, self.c]
        
        # Convert to mm: (cells * 180) + (90mm to center of cell)
        true_mm = (grid_dist * self.cell_size) + (self.cell_size / 2.0)
        
        # Add 5% Noise
        sigma = 0.05 * true_mm 
        noise = np.random.normal(0, sigma, size=4)
        measured_mm = true_mm + noise
        
        # Clip to sensor limits
        measured_mm = np.clip(measured_mm, self.min_range_mm, self.max_range_mm)#
        
        return true_mm, measured_mm.astype(int)

    def move(self, direction_idx, maze):
        """Moves the robot if no wall exists."""
        dirs = {0: (-1, 0), 1: (1, 0), 2: (0, -1), 3: (0, 1)}
        dr, dc = dirs[direction_idx]
        
        if maze[self.r, self.c, direction_idx] == 0:
            print(f"CRASH! Wall at direction {direction_idx}")
            return False
            
        self.r += dr
        self.c += dc
        return True

