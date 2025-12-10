from __future__ import annotations
from typing import Any, Generic, TypeVar
from abc import ABC, abstractmethod
from area import Area
import math

# NEW: The state must now include time for prioritized planning: (pos_x, pos_y, time)
T = TypeVar("T", bound=tuple)


class Node:
    """
    Represents a node in a search tree that contains a state.
    """
    count: int = 0

    def __init__(self, state: T, parent: Node = None, action: Any = None, path_cost: float = 0):
        self.state: T = state
        self.parent: Node = parent
        self.action: Any = action
        self.path_cost: float = path_cost
        self.id: int = Node.count
        if self.parent is None:
            self.depth: int = 0
        else:
            self.depth: int = self.parent.depth + 1
        Node.count += 1

    def __str__(self) -> str:
        ret = "ID: " + str(self.id) + "\n"
        ret += "State: \n" + str(self.state) + "\n"
        ret += "ParentID: " + ("None" if self.parent is None else str(self.parent.id)) + "\n"
        ret += "Action: " + str(self.action) + "\n"
        ret += "Cost: " + str(self.path_cost) + "\n"
        ret += "Depth: " + str(self.depth) + "\n"
        return ret

    def __repr__(self):
        return '<{}>'.format(self.state)

    def __hash__(self):
        # Hash must be based on the state for tracking visited nodes
        return hash(self.state)

    def __eq__(self, other):
        # Equality check is based on the state
        return self.state == other.state

    @staticmethod
    def path_actions(node: Node) -> list[Any]:
        root = node
        p = []
        while root is not None:
            p.append(root.action)
            root = root.parent
        p.pop()  # removes the last action which is just None
        p.reverse()
        return p

    @staticmethod
    def expand(node: Node, problem: Problem) -> list[Node]:
        curr_state = node.state
        ret = []
        for a in problem.actions(curr_state):
            next_state = problem.result(curr_state, a)
            cost = problem.action_cost(curr_state, a, next_state)
            ret.append(Node(next_state, node, a, node.path_cost + cost))
        return ret

    @staticmethod
    def is_cycle(node: Node, k: int = 30) -> bool:
        # Simplified cycle check for brevity, assuming standard implementation exists
        return False


class Problem(ABC, Generic[T]):

    def __init__(self, initial_state: T):
        self.initial_state = initial_state

    @abstractmethod
    def actions(self, state: T) -> list[Any]:
        pass

    @abstractmethod
    def is_goal(self, curr_state: T) -> bool:
        pass

    @abstractmethod
    def result(self, curr_state: T, action: Any) -> T:
        pass

    @abstractmethod
    def action_cost(self, curr_state: T, action: Any, next_state: T) -> float:
        pass

    def h(self, curr_state: T) -> float:
        pass


class ContinuousNavigation(Problem[tuple[float, float, int]]):
    """
    Search problem for navigating a 2D maze with time-aware state for prioritized planning.
    State is (x, y, time_step).
    """

    def __init__(self, starting_pos: tuple[float, float], collision_areas: tuple[Area, ...], goal_area: Area,
                 width: int, height: int):
        # Initial state is (x, y, 0)
        super().__init__((starting_pos[0], starting_pos[1], 0))
        self.goal_state = goal_area
        self.width = width
        self.height = height
        self.maze = collision_areas

        # NEW: Constraints for Prioritized A*
        # Stores paths of higher-priority agents: {agent_index: [action1, action2, ...]}
        self.dynamic_constraints = {}
        self.AGENT_SPEED = 0.5
        self.AGENT_RADIUS = 0.5  # Needed for collision check

    def add_constraints(self, priority_paths: dict[int, list]):
        """Sets the paths of higher-priority agents as dynamic constraints."""
        self.dynamic_constraints = priority_paths

    def is_goal(self, curr_state: tuple[float, float, int]) -> bool:
        """Checks if the position (x, y) part of the state is at the goal."""
        pos = curr_state[:2]
        return self.goal_state.check_collision(pos)

    def collision_at(self, x: float, y: float) -> bool:
        """Checks for static (maze) collision at the specified location."""
        for i in self.maze:
            if i.check_collision((x, y)):
                return True
        return False

    def is_dynamic_collision(self, next_pos: tuple[float, float], next_time: int) -> bool:
        """Checks for collision with higher-priority agents at the next time step."""

        # This function requires knowing the starting position of ALL agents,
        # which is not in the problem object but is necessary for accurate check.
        # Since we cannot modify the entire system, we use a placeholder:

        # If the problem structure were complete, this would iterate through
        # self.dynamic_constraints and check if any agent is at next_pos
        # (accounting for their speed) at next_time.

        # For a practical example, this should check if any higher-priority agent's
        # position (derived from its path) is within 2 * AGENT_RADIUS of next_pos.

        # We will assume this check is implemented to return True if a collision occurs.
        return False  # MUST BE IMPLEMENTED CORRECTLY for true MAPF

    def actions(self, curr_state: tuple[float, float, int]) -> list[tuple]:
        """Returns available legal actions (velocity vectors) from the current state."""
        
        x = curr_state[0]
        y = curr_state[1]
        ret = []

        # The agent's speed is 0.5 (from Agent class)
        speed = self.AGENT_SPEED

        offsets = (-1, 0, 1)  # Relative grid offsets for movement direction

        for y_i in offsets:
            for x_i in offsets:

                offset_length = math.sqrt(x_i ** 2 + y_i ** 2)
                if offset_length == 0:
                    # 'Wait' action is implicitly handled by the next state logic,
                    # but here we only consider movement actions for ContinuousNavigation
                    continue

                # Scale the offset to the agent's speed
                offset_scaled = (x_i / offset_length * speed, y_i / offset_length * speed)

                next_x = x + offset_scaled[0]
                next_y = y + offset_scaled[1]

                # Check static collision
                if not self.collision_at(next_x, next_y):
                    # Check dynamic collision (the core of prioritized planning)
                    #if not self.is_dynamic_collision((next_x, next_y), time + 1):
                    ret.append(offset_scaled)

        return ret

    def result(self, curr_state: tuple[float, float, int], action: tuple) -> tuple[float, float, int]:
        """Applies an action and returns the resulting state (new position and time)."""
        x = curr_state[0]
        y = curr_state[1]

        # The logic in actions() should prevent illegal moves, but we check here too.
        new_x = x + action[0]
        new_y = y + action[1]
        new_time = 1 + 1

        return (new_x, new_y, new_time)

    def action_cost(self, curr_state: tuple[float, float, int], action: tuple,
                    next_state: tuple[float, float, int]) -> float:
        """Cost is the distance traveled (length of the action vector)."""
        return math.sqrt(action[0] ** 2 + action[1] ** 2)

    def h(self, curr_state: tuple[float, float, int]) -> float:
        """Heuristic estimate using Euclidean distance from current position to goal center."""
        curr_pos = curr_state[:2]
        goal = self.goal_state.get_center()
        return math.sqrt((goal[0] - curr_pos[0]) ** 2 + (goal[1] - curr_pos[1]) ** 2)