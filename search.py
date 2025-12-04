import collections
import random
import sys
from queue import PriorityQueue


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

