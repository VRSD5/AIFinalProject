from queue import PriorityQueue
import util
import search
import area
from problem import ContinuousNavigation
import copy

GRID_CACHE = {}         # Maps (start_grid, goal_grid, density) → waypoint list
SEGMENT_CACHE = {}      # Maps (start_pos, waypoint_pos) → low-level A* path


class HAGridNode:
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
    grid_x = int(pos[0] // density) * density + density / 2
    grid_y = int(pos[1] // density) * density + density / 2
    return (grid_x, grid_y)


def get_neighbors(center: tuple[float, float], density: float) -> list[tuple[float, float]]:
    neighbors = []
    offsets = [(-density, 0), (density, 0), (0, density), (0, -density),
               (-density, -density), (-density, density),
               (density, -density), (density, density)]
    for dx, dy in offsets:
        neighbors.append((center[0] + dx, center[1] + dy))
    return neighbors

def high_level_search(problem: ContinuousNavigation, density: float) -> list[tuple[float, float]]:
    goal_center = problem.goal_state.get_center()
    start_grid = get_grid_center(problem.initial_state, density)
    goal_grid = get_grid_center(goal_center, density)

    cache_key = (start_grid, goal_grid, density)
    if cache_key in GRID_CACHE:
        return GRID_CACHE[cache_key]

    if start_grid == goal_grid:
        GRID_CACHE[cache_key] = [goal_center]
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
            # Reconstruct path
            path = []
            while current_pos is not None:
                path.append(current_pos)
                current_pos = came_from[current_pos]

            waypoints = list(reversed(path))[1:]  # Remove start grid

            if waypoints and waypoints[-1] == goal_grid:
                waypoints[-1] = goal_center  # Snap last waypoint to exact goal

            GRID_CACHE[cache_key] = waypoints
            return waypoints

        # Expand neighbors
        for neighbor_pos in get_neighbors(current_pos, density):

            # OBSTACLE COLLISION ONLY — this is correct
            if any(obstacle.check_collision(neighbor_pos) for obstacle in problem.maze):
                continue

            cost = util.length(
                (neighbor_pos[0] - current_pos[0],
                 neighbor_pos[1] - current_pos[1])
            )
            new_g_score = g_score[current_pos] + cost

            if neighbor_pos not in g_score or new_g_score < g_score[neighbor_pos]:
                g_score[neighbor_pos] = new_g_score
                came_from[neighbor_pos] = current_pos

                neighbor_node = HAGridNode(
                    neighbor_pos, new_g_score, h(neighbor_pos), node_id_counter
                )
                frontier.put(neighbor_node)
                node_id_counter += 1

    GRID_CACHE[cache_key] = []
    return []


def hierarchical_astar_search(problem: ContinuousNavigation) -> list[tuple[float, float]]:
    DENSITY = 25   # <<--- MUCH COARSER GRID = BIG PERFORMANCE BOOST

    seg_problem = copy.copy(problem)

    # High-level coarse plan
    waypoints = high_level_search(seg_problem, DENSITY)
    if not waypoints:
        return []

    final_path_moves = []
    current_start_pos = seg_problem.initial_state
    original_goal_state = seg_problem.goal_state

    for waypoint in waypoints:

        seg_key = (current_start_pos, waypoint)
        if seg_key in SEGMENT_CACHE:
            final_path_moves.extend(SEGMENT_CACHE[seg_key])
            current_start_pos = waypoint
            continue

        segment_goal_area = area.CircleArea(waypoint, 0.5, "temp")

        seg_problem.goal_state = segment_goal_area
        seg_problem.initial_state = current_start_pos

        segment_path = search.astar_search(seg_problem)

        if not segment_path:
            seg_problem.goal_state = original_goal_state
            return []

        SEGMENT_CACHE[seg_key] = segment_path  # Save for future agents
        final_path_moves.extend(segment_path)

        current_start_pos = waypoint

    seg_problem.goal_state = original_goal_state
    return final_path_moves
