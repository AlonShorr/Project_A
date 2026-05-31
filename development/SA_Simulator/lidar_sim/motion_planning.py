from rrt_planner import plan_rrt


def rrt(start, goal, wall_map):
    """
    Plan a path from start to goal using RRT.

    start: (r, c) or (r, c, theta_idx)
    goal:  (r, c) target grid cell

    Returns a list of (r, c) cells from start to goal inclusive,
    or an empty list if no path is found.
    """
    result = plan_rrt(wall_map=wall_map, start=start, goal=goal)
    if result["success"]:
        start_cell = (int(start[0]), int(start[1]))
        return [start_cell] + result["path"]

    print("RRT failed:", result["debug"]["reason"])
    return []
