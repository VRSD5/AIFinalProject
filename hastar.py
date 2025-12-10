# hastar.py

import collections
import random
import sys
from queue import PriorityQueue
import util
import search
import area
from problem import ContinuousNavigation, Problem, Node
import copy


class HAGridNode:
    # ... (HAGridNode class content is unchanged)
    def __init__(self, pos: tuple[float, float], cost: float, heuristic: float, node_id: int):
        self.pos = pos
        self.cost = cost
        self.heuristic = heuristic
        self.f = self.cost + self.heuristic
        self.id = node_id

    def __lt__(self, other):
        if self.f != other.f:
            return self.f < other.f
        return self.id < other.id

    def __hash__(self):
        return hash(self.pos)

    def __eq__(self, other):
        return self.pos == other.pos


def get_grid_center(pos: tuple[float, float], density: float) -> tuple[float, float]:
    # ... (get_grid_center content is unchanged)
    grid_x = int(pos[0] // density) * density + density / 2
    grid_y = int(pos[1] // density) * density + density / 2
    return (grid_x, grid_y)


def get_neighbors(center: tuple[float, float], density: float) -> list[tuple[float, float]]:
    # ... (get_neighbors content is unchanged)
    neighbors = []
    offsets = [(-density, 0), (density, 0), (0, density), (0, -density),
               (-density, -density), (-density, density), (density, -density), (density, density)]
    for dx, dy in offsets:
        neighbors.append((center[0] + dx, center[1] + dy))
    return neighbors


def high_level_search(problem: ContinuousNavigation, density: float) -> list[tuple[float, float]]:
    # FIXED: Use problem.goal_state instead of problem.goal
    goal_center = problem.goal_state.get_center()
    start_grid = get_grid_center(problem.initial_state, density)
    goal_grid = get_grid_center(goal_center, density)

    if start_grid == goal_grid:
        return [goal_center]

    def h(pos: tuple[float, float]) -> float:
        return abs(pos[0] - goal_grid[0]) + abs(pos[1] - goal_grid[1])

    frontier = PriorityQueue()
    node_id_counter = 0
    start_node = HAGridNode(start_grid, 0, h(start_grid), node_id_counter)
    frontier.put(start_node)

    came_from = {start_grid: None}
    g_score = {start_grid: 0}
    node_id_counter += 1

    while not frontier.empty():
        current_node: HAGridNode = frontier.get()
        current_pos = current_node.pos

        if current_pos == goal_grid:
            path = []
            while current_pos is not None:
                path.append(current_pos)
                current_pos = came_from[current_pos]

            waypoints = list(reversed(path))[1:]

            if waypoints and waypoints[-1] == goal_grid:
                waypoints[-1] = goal_center

            return waypoints

        for neighbor_pos in get_neighbors(current_pos, density):
            # Collision check is against the obstacle list 'self.maze' in ContinuousNavigation
            if any(obstacle.check_collision(neighbor_pos) for obstacle in problem.maze):
                continue

            cost = util.length((neighbor_pos[0] - current_pos[0], neighbor_pos[1] - current_pos[1]))
            new_g_score = g_score[current_pos] + cost

            if neighbor_pos not in g_score or new_g_score < g_score[neighbor_pos]:
                g_score[neighbor_pos] = new_g_score
                came_from[neighbor_pos] = current_pos

                neighbor_node = HAGridNode(neighbor_pos, new_g_score, h(neighbor_pos), node_id_counter)
                frontier.put(neighbor_node)
                node_id_counter += 1
    return []


def hierarchical_astar_search(problem: ContinuousNavigation) -> list[tuple[float, float]]:
    DENSITY = 10

    # Use a copy of the problem object to ensure the original instance's state is not corrupted
    # Use a shallow copy since problem.maze (the environment) and goal_state are immutable references
    seg_problem = copy.copy(problem)

    # 1. High-Level Search
    high_level_waypoints = high_level_search(seg_problem, DENSITY)

    if not high_level_waypoints:
        return []

    # 2. Piece together Low-Level Paths
    final_path_moves = []
    current_start_pos = seg_problem.initial_state

    # Store the original goal area for restoration
    original_goal_state = seg_problem.goal_state

    for waypoint in high_level_waypoints:
        # --- Low-Level A* from Current Position to Waypoint ---

        # Create a tiny CircleArea at the waypoint to act as the segment goal
        segment_goal_area = area.CircleArea(waypoint, 0.5, "temp")

        # FIXED: Reconfigure the copied problem instance for the segment by changing goal_state
        seg_problem.goal_state = segment_goal_area
        seg_problem.initial_state = current_start_pos

        # The low-level search uses standard A* from search.py
        segment_path_moves = search.astar_search(seg_problem)

        if not segment_path_moves:
            # Low-level segment failed, restore and exit
            seg_problem.goal_state = original_goal_state
            return []

        final_path_moves.extend(segment_path_moves)

        # Update starting position for the next segment
        current_start_pos = waypoint

        # 3. Restore the original goal object to the copied problem instance (good practice)
    seg_problem.goal_state = original_goal_state

    return final_path_moves