import collections
import math
import random
import sys
from queue import PriorityQueue

import area
from problem import ContinuousNavigation, Problem
from problem import Node

def random_search(problem: Problem) -> list[str]:
    initial_state = problem.initial_state
    num_steps = 10
    curr_node = Node(initial_state)
    for i in range(num_steps):
        # Generate nodes that hold the neighboring states.
        neighbors = Node.expand(curr_node, problem)
        if len(neighbors) == 0:
            break
        # Pick a neighbor at random to become the new current node.
        curr_node = random.choice(neighbors)
    # See get_path() documentation for how it recreates the path
    # from the current node and expand for what information is
    # baked into each node.
    path = Node.path_actions(curr_node)
    return path

def breadth_first_search(problem: Problem) -> list[str]:
    """Implements early-goal test BFS"""
    #TODO Implement this method.
    node : Node
    node = Node(problem.initial_state)
    if problem.is_goal(node.state):
        return Node.path_actions(node)
    frontier : list[Node] = [node]
    reached  = {problem.initial_state}
    while frontier:
        #print(node)
        node : Node = frontier.pop(0)


        for child in Node.expand(node, problem):
            s = child.state
            if problem.is_goal(s):
                return Node.path_actions(child)
            if not (s in reached):
                reached.add(s)
                frontier.append(child)
    return []

def depth_limited_search(problem: Problem, limit:int, fail = False) -> list[str]:
    """Implements depth-limited DFS"""
    #TODO Implement this method.
    frontier = [Node(problem.initial_state)]
    result = []
    if fail:
        result = ["fail"]
    while frontier:
        node : Node = frontier.pop(-1)
        if problem.is_goal(node.state):
            return Node.path_actions(node)
        if node.depth > limit:
            result = ["cutoff"]
        elif not Node.is_cycle(node):
            for child in Node.expand(node, problem):
                frontier.append(child)

    return result


def depth_first_search(problem: Problem) -> list[str]:
    """Implements DFS by using depth-limited DFS with
    a very large (i.e., infinite) limit."""
    #TODO Implement this method. Use sys.maxsizve for the very large value.
    return depth_limited_search(problem, sys.maxsize)

def iterative_deepening(problem: Problem) -> list[str]:
    """Implements Iterative Deepening by using depth-limited DFS."""
    #TODO Implement this method.
    for depth in range(0, sys.maxsize):
        result = depth_limited_search(problem, depth, fail=True)
        if result == ["fail"]:
            return []
        if not (result == ["cutoff"]):
            return result
    return random_search(problem)

def uniform_search(problem: Problem) -> list[str]:
    """Implements Uniform Search by hardcoding the frontier behavior of best-first search as
    a priority queue that uses the path_cost of a node as its priority.
    """
    #TODO Implement this method. Look at pq_example.py for how to use a priority queue.
    node : Node
    node = Node(problem.initial_state)
    if problem.is_goal(node.state):
        return Node.path_actions(node)
    frontier = PriorityQueue()
    frontier.put((node.path_cost, node.id, node))
    reached  = {problem.initial_state:node}
    while frontier.qsize() > 0:
        node : Node = frontier.get()[2]
        for child in Node.expand(node, problem):
            s = child.state
            if problem.is_goal(s):
                return Node.path_actions(child)
            if not (s in reached) or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.put((child.path_cost, child.id, child))
    return []
    return random_search(problem)


def astar_search(problem: Problem) -> list[str]:
    """Implements A* Search."""

    node : Node
    node = Node(problem.initial_state)
    #print(node)
    if problem.is_goal(node.state):
        return Node.path_actions(node)
    frontier = PriorityQueue()
    frontier.put((node.path_cost + problem.h(node.state), node.id, node))
    reached  = {problem.initial_state:node}
    while frontier.qsize() > 0:
        node : Node = frontier.get()[2]
        for child in Node.expand(node, problem):
            s = child.state
            if problem.is_goal(s):
                return Node.path_actions(child)
            if not (s in reached) or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.put((child.path_cost + problem.h(child.state), child.id, child))
    return []
    return random_search(problem)


def hierarchical_astar(problem: ContinuousNavigation) -> list[str]:
    """Hierarchical A* using waypoints in continuous space"""
    start = problem.initial_state
    goal = problem.goal_state.get_center()

    # If close enough, just use regular A*
    dist = math.sqrt((goal[0] - start[0]) ** 2 + (goal[1] - start[1]) ** 2)
    if dist < 30:
        return astar_search(problem)

    # Create intermediate waypoints along straight line
    num_waypoints = max(3, int(dist / 20))
    waypoints = []

    for i in range(1, num_waypoints):
        t = i / num_waypoints
        wx = start[0] * (1 - t) + goal[0] * t
        wy = start[1] * (1 - t) + goal[1] * t
        waypoints.append((wx, wy))

    # Add final goal
    waypoints.append(goal)

    # Pathfind to each waypoint sequentially
    full_path = []
    current_pos = start

    for waypoint in waypoints:
        # Create subproblem to this waypoint
        from area import CircleArea
        subgoal = CircleArea(waypoint, 3.0, "green")
        subproblem = ContinuousNavigation(
            current_pos, problem.maze, subgoal,
            problem.width, problem.height
        )

        subpath = astar_search(subproblem)
        if not subpath:  # Can't reach waypoint, fallback
            return astar_search(problem)

        full_path.extend(subpath)

        # Update current position
        for action in subpath:
            current_pos = problem.result(current_pos, action)

    return full_path
def greedy_search(problem: Problem) -> list[str]:
    """Implements Greedy Search."""
    #TODO Implement this method. Make sure you implement the heuristic methods h() in
    # DustMaze before running the dust maze tests.
    node : Node
    node = Node(problem.initial_state)
    if problem.is_goal(node.state):
        return Node.path_actions(node)
    frontier = PriorityQueue()
    frontier.put((problem.h(node.state), node.id, node))
    reached  = {problem.initial_state:node}
    while frontier.qsize() > 0:
        node : Node = frontier.get()[2]
        for child in Node.expand(node, problem):
            s = child.state
            if problem.is_goal(s):
                return Node.path_actions(child)
            if not (s in reached) or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.put((problem.h(child.state), child.id, child))
    return []
    return random_search(problem)

