from rrt_planner import plan_rrt, bfs_shortest_path_cells


def rrt(start, goal, wall_map):
    """
    Plan a path from start to goal using RRT.

    start: (r, c) or (r, c, theta_idx)
    goal:  (r, c) target grid cell

    Returns a list of (r, c) cells from start to goal inclusive,
    or an empty list if no path is found.
    """
    start_cell = (int(start[0]), int(start[1]))
    goal_cell = (int(goal[0]), int(goal[1]))

    bfs_path = bfs_shortest_path_cells(wall_map, start_cell, goal_cell)
    if bfs_path is None:
        print("BFS reachability: unreachable")
    else:
        print(f"BFS reachability: reachable, shortest path length = {len(bfs_path) - 1}")

    result = plan_rrt(wall_map=wall_map, start=start, goal=goal, max_iter=5000, goal_sample_rate=0.25)

    if result["success"]:
        print(
            f"RRT result: success after {result['debug'].get('iterations')} iterations, "
            f"nodes={result['debug'].get('num_nodes')}, source={result['debug'].get('path_source')}"
        )
        return [start_cell] + result["path"]

    print("RRT failed:", result["debug"]["reason"])
    print(
        "RRT diagnostics: "
        f"map_shape={result['debug'].get('map_shape')}, "
        f"start={result['debug'].get('start_state')}, "
        f"goal={result['debug'].get('goal_state')}, "
        f"unique_states={result['debug'].get('unique_states_in_tree')}/"
        f"{result['debug'].get('total_possible_states')}, "
        f"duplicates={result['debug'].get('duplicate_states_rejected')}, "
        f"invalid_forward={result['debug'].get('invalid_forward_moves')}, "
        f"wall_collisions={result['debug'].get('invalid_wall_collisions')}, "
        f"out_of_bounds={result['debug'].get('invalid_out_of_bounds')}, "
        f"diagonal_blocks={result['debug'].get('diagonal_forward_blocks')}, "
        f"goal_samples={result['debug'].get('goal_samples')}, "
        f"expansions={result['debug'].get('successful_expansions')}, "
        f"closest={result['debug'].get('closest_node_to_goal')}, "
        f"closest_dist={result['debug'].get('closest_distance_to_goal')}"
    )
    return []
