import numpy as np
import random

from config import (
    MAP_COLS, SENSOR_ANGLES, MIN_RANGE_MM,
    WALL_UP, WALL_DOWN, WALL_LEFT, WALL_RIGHT,
    LOCALIZATION_MOVES,
)
from sim_robot import Robot


def precompute_all_orientations(wall_map):
    """
    Pre-compute noiseless sensor readings for every (row, col, angle) combination.
    Returns ndarray of shape (Rows, Cols, 8 angles, N sensors).
    """
    print("Pre-computing for 8 orientations... (Wait for it)")
    rows, cols, _ = wall_map.shape
    data = np.zeros((rows, cols, 8, len(SENSOR_ANGLES)))

    v_bot = Robot(0, 0)
    for r in range(rows):
        for c in range(cols):
            v_bot.move_to(r, c)
            for ang_idx in range(8):
                v_bot.angle_index = ang_idx
                v_bot.active_sensors = [True] * len(SENSOR_ANGLES)
                dists, _ = v_bot.measure(wall_map)
                data[r, c, ang_idx] = dists

    print("Pre-computation Complete.")
    return data


def update_probability(prob_matrix, measured_dists, expected_data, current_angle_idx):
    """
    Bayes correction step: weight each cell by how well the current measurements
    match the pre-computed expectations for that cell and heading.

    sigma=20mm accounts for map discretization error (cells are 180mm wide) on
    top of the sensor's 1mm accuracy.
    """
    rows, cols = prob_matrix.shape
    new_prob = np.zeros_like(prob_matrix)
    sigma = 20.0  # mm

    for r in range(rows):
        for c in range(cols):
            if prob_matrix[r, c] < 0.00001:
                continue

            expected_dists = expected_data[r, c, current_angle_idx]
            likelihood = 1.0

            for i in range(len(SENSOR_ANGLES)):
                z = measured_dists[i]
                if z is None:
                    continue
                mu = max(expected_dists[i], MIN_RANGE_MM)
                likelihood *= np.exp(-((z - mu) ** 2) / (2 * sigma ** 2))

            new_prob[r, c] = prob_matrix[r, c] * likelihood

    total = np.sum(new_prob)
    if total > 0:
        new_prob /= total
    else:
        new_prob.fill(1.0 / np.count_nonzero(prob_matrix))

    return new_prob


def predict_motion(prob_matrix, dr, dc, wall_map):
    """
    Motion model: shift the probability cloud by (dr, dc), then apply a
    3×3 blur to represent positional uncertainty after each move.
    """
    rows, cols = prob_matrix.shape
    shifted = np.zeros_like(prob_matrix)

    for r in range(rows):
        for c in range(cols):
            if prob_matrix[r, c] <= 0.0001:
                continue

            blocked = (
                (dr == -1 and (wall_map[r, c, WALL_UP] == 0 or r - 1 < 0)) or
                (dr ==  1 and (wall_map[r, c, WALL_DOWN] == 0 or r + 1 >= rows)) or
                (dc == -1 and (wall_map[r, c, WALL_LEFT] == 0 or c - 1 < 0)) or
                (dc ==  1 and (wall_map[r, c, WALL_RIGHT] == 0 or c + 1 >= cols))
            )

            if blocked:
                shifted[r, c] += prob_matrix[r, c]
            else:
                shifted[r + dr, c + dc] += prob_matrix[r, c]

    # Blur kernel:
    # 0.05  0.10  0.05
    # 0.10  0.40  0.10
    # 0.05  0.10  0.05
    blurred = np.zeros_like(shifted)
    for r in range(rows):
        for c in range(cols):
            val = shifted[r, c]
            if val < 1e-9:
                continue
            for dr_k in (-1, 0, 1):
                for dc_k in (-1, 0, 1):
                    nr, nc = r + dr_k, c + dc_k
                    if not (0 <= nr < rows and 0 <= nc < cols):
                        continue
                    if dr_k == 0 and dc_k == 0:
                        weight = 0.40
                    elif dr_k == 0 or dc_k == 0:
                        weight = 0.10
                    else:
                        weight = 0.05
                    blurred[nr, nc] += val * weight

    total = np.sum(blurred)
    if total > 0:
        blurred /= total

    return blurred


def get_best_estimate(prob_matrix):
    """Return (r, c, peak_probability) of the highest-probability cell."""
    idx = np.argmax(prob_matrix)
    r, c = divmod(idx, MAP_COLS)
    return r, c, prob_matrix[r, c]


def do_localization_step(robot, wall_map, spin_count, move_count):
    """
    Advance the localization routine by one action frame:
    first complete a full 360° spin (8 steps), then try random moves.
    Returns (spin_count, move_count, moved_dr, moved_dc).
    """
    RANDOM_DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    moved_dr, moved_dc = 0, 0

    if spin_count < 8:
        robot.rotate(1)
        spin_count += 1
    elif move_count < LOCALIZATION_MOVES:
        dr, dc = random.choice(RANDOM_DIRS)
        if robot.move_rel(dr, dc, wall_map):
            moved_dr, moved_dc = dr, dc
        move_count += 1
    else:
        spin_count = 0
        move_count = 0

    return spin_count, move_count, moved_dr, moved_dc
