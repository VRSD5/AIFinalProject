from __future__ import annotations  # needed in order to reference a Class within itself

from typing import Any, Generic, TypeVar
from abc import ABC, abstractmethod
from area import Area
import math

# makes the state both immutable and hashable
T = TypeVar("T", bound=tuple)

class Node:
    """
    Represents a node in a search tree that contains a state.

    Attributes:
        count (int): Class-level counter tracking created nodes.
        state (T): Current state of the problem.
        parent (Node | None): Parent node containing the prior state.
        action (str | None): Action taken to reach this state from the parent.
        path_cost (float): Cumulative cost of reaching this node.
        id (int): Unique identifier for the node instance.
        depth (int): Depth of the node in the search tree.
    """
    count: int = 0
    """Class-level counter tracking created nodes."""

    def __init__(self, state: T, parent: Node = None, action: str = None, path_cost: float = 0):
        """
        Initializes a Node.

        Args:
            state (T): Current state.
            parent (Node | None): Parent node. Defaults to None.
            action (str | None): Action taken from the parent. Defaults to None.
            path_cost (float): Path cost from the parent. Defaults to 0.
        """
        self.state:T = state
        self.parent:Node = parent
        self.action:str = action
        self.path_cost:float = path_cost
        self.id:int = Node.count
        if self.parent is None:
            self.depth:int = 0
        else:
            self.depth:int = self.parent.depth + 1
        Node.count += 1

    def __str__(self) -> str:
        """String representation of the node"""
        ret = "ID: " + str(self.id) + "\n"
        ret += "State: \n" + str(self.state) + "\n"
        ret += "ParentID: " + ("None" if self.parent is None else str(self.parent.id)) + "\n"
        ret += "Action: " + str(self.action) + "\n"
        ret += "Cost: " + str(self.path_cost) + "\n"
        ret += "Depth: " + str(self.depth) + "\n"
        return ret

    def __repr__(self): return '<{}>'.format(self.state)

    def __hash__(self):
        """Stubbing this out so if you try to use it, it will throw an error."""
        raise NotImplementedError

    def __eq__(self, other):
        """Stubbing this out so if you try to use it, it will throw an error."""
        raise NotImplementedError

    def __copy__(self):
        """Stubbing this out so if you try to use it, it will throw an error."""
        raise NotImplementedError

    @staticmethod
    def path_actions(node: Node) -> list[str]:
        """
        Returns the sequence of actions from the root to a node.

        Args:
            node (Node): Target node.

        Returns:
            list[str]: Actions leading to the node.
        """

        root = node
        p = []

        while root is not None:
            p.append(root.action)
            root = root.parent

        # removes the last action which is just None
        p.pop()
        p.reverse()
        return p

    @staticmethod
    def expand(node: Node, problem: Problem) -> list[Node]:
        """
        Expands a node into nodes with neighboring states.

        Args:
            node (Node): Current node.
            problem (Problem): Search problem.

        Returns:
            list[Node]: Nodes containing neighboring states.
        """
        curr_state = node.state
        ret = []
        for a in problem.actions(curr_state):
            next_state = problem.result(curr_state, a)
            cost = problem.action_cost(curr_state, a, next_state)
            ret.append(Node(next_state, node, a, node.path_cost + cost))
        return ret

    @staticmethod
    def is_cycle(node: Node, k: int = 30) -> bool:
        """
        Checks if the node forms a cycle within k ancestors.

        Args:
            node (Node): Node to check.
            k (int): Max number of ancestors. Defaults to 30.

        Returns:
            bool: True if a cycle exists, False otherwise.
        """
        def find_cycle(ancestor:Node, _k:int) -> bool:
            if ancestor is None: # you're at root and done
                return False
            elif _k > 0:
                # seen the current state before
                if ancestor.state == node.state:
                    return True
                else:
                    return find_cycle(ancestor.parent, _k - 1)
            else:
                return False
        return find_cycle(node.parent, k)

class Problem(ABC, Generic[T]):
    """
    Interface representing a generic problem formulation.

    This class defines the operations needed for search algorithms.
    """

    def __init__(self, initial_state: T):
        """
        Initializes a problem with an initial state.

        Args:
            initial_state (T): The starting state of the problem.
        """
        self.initial_state = initial_state

    @abstractmethod
    def actions(self, state: T) -> list[str]:
        """
        Returns a list of legal actions available from the given state.

        Args:
            state (T): Current state.

        Returns:
            List[str]: List of actions available from the state.
        """
        pass

    @abstractmethod
    def is_goal(self, curr_state: T) -> bool:
        """
        Determines whether the given state is a goal state.

        Args:
            curr_state (T): State to check.

        Returns:
            bool: True if curr_state is a goal state, False otherwise.
        """
        pass

    @abstractmethod
    def result(self, curr_state: T, action: str) -> T:
        """
        Returns the state that results from applying an action to the current state.

        Args:
            curr_state (T): Current state.
            action (str): Action to apply.

        Returns:
            T: The resulting state after applying the action.
        """
        pass

    @abstractmethod
    def action_cost(self, curr_state: T, action: str, next_state: T) -> float:
        """
        Returns the cost of transitioning from curr_state to next_state via the given action.

        Args:
            curr_state (T): Current state.
            action (str): Action applied.
            next_state (T): Resulting state after applying the action.

        Returns:
            float: Cost associated with the transition.
        """
        pass

    def h(self, curr_state: T) -> float:
        """
        Heuristic estimate of the cost to reach a goal from the current state.

        Args:
            curr_state (T): Current state.

        Returns:
            float: Estimated cost to reach a goal state.
        """
        pass

    def value(self, curr_state: T) -> float:
        """
        Performance measure of the current state, used for local search.

        Args:
            curr_state (T): Current state.

        Returns:
            float: Fitness or score of the current state.
        """
        pass

    def verify(self, actions: list[str]) -> bool:
        """
        Verifies that a sequence of actions leads from the initial state to a goal state.

        Args:
            actions (List[str]): Sequence of actions to test.

        Returns:
            bool: True if the actions reach a goal state, False otherwise.
        """
        new_state = self.initial_state
        for action in actions:
            new_state = self.result(new_state, action)
        return self.is_goal(new_state)


class ContinuousNavigation(Problem[tuple[Area]]):
    """
    Search problem for navigating a 2D maze.

    A state is a tuple of locations of things in the maze,
    with the first tuple being the agent location.

    Attributes:
        DIRECTIONS (list[str]): List of possible movement directions.
        goal_state (tuple[int, int]): Target goal location.
        width (int): Maze width.
        height (int): Maze height.
        maze (list[Thing]): Flat list representation of the maze layout.
    """

 

    def __init__(self, starting_pos, collision_areas: tuple[Area, ...], goal_area: Area, width: int, height: int):
        """
        Initializes a MazeNavigation problem.

        Args:
            thing_locations (tuple[tuple[int, int], ...]): Starting locations of things.
            The first tuple is the agent.
            goal_state (tuple[int, int]): Goal location.
            maze (list[Thing]): Flat list representation of the maze.
            width (int): Maze width.
            height (int): Maze height.
        """
        super().__init__(starting_pos)
        self.goal_state = goal_area
        self.width = width
        self.height = height
        self.maze = collision_areas

    def is_goal(self, curr_pos: tuple[int, int]) -> bool:
        """
        Checks if the agent is at the goal.

        Args:
            curr_state (tuple[tuple[int, int], ...]): Locations of things.
            The first tuple is the agent.
        Returns:
            bool: True if the agent is at the goal, False otherwise.
        """
        return self.goal_state.check_collision(curr_pos)



    def collision_at(self, x: int, y: int) -> bool:
        """
        Checks if a Thing of a given type is at the specified location.

        Args:
            x (int): X-coordinate.
            y (int): Y-coordinate.
            t_class (type[Thing]): Class of Thing to check.

        Returns:
            bool: True if a Thing of the specified type is present, False otherwise.
        """
        for i in self.maze:
            if i.check_collision((x,y)):
                return True
        return False


    def actions(self, curr_pos: tuple[int, int]) -> list[str]:
        """
        Returns available actions from the current state.

        Args:
            curr_state (tuple[tuple[int, int], ...]): Locations of things.
            The first tuple is the agent.
        Returns:
            list[str]: List of legal actions ("Up", "Right", "Down", "Left").
        """
        x, y = curr_pos
        ret = []

        speed = 0.5

        offsets = (-1, 0, 1)

        for y_i in offsets:
            for x_i in offsets:
                
                offset_length = math.sqrt(x_i ** 2 + y_i ** 2)
                if offset_length == 0:
                    continue

                offset_scaled = (x_i / offset_length * speed, y_i / offset_length * speed)

                if not self.collision_at(x + offset_scaled[0], y + offset_scaled[1]):
                    ret.append(offset_scaled)


        #ret.sort(key=lambda d: self.DIRECTIONS.index(d))
        return ret

    def result(self, curr_pos: tuple[int, int], action: tuple) -> tuple[int, int]:
        """
        Applies an action and returns the resulting state.

        Args:
            curr_state (tuple[tuple[int, int], ...]): Locations of things. T
            he first tuple is the agent.
            action (str): Action to apply.

        Returns:
            tuple[tuple[int, int], ...]: New state with updated agent location
            and removed locations that match the agent's new position.
        """
        x, y = curr_pos

        if not self.collision_at(x + action[0], y + action[1]):
            new_agent = (x + action[0], y + action[1])
        else:
            print(f"Illegal Move {action}")
            new_agent = (x, y)

        # Keep all other things except those at the new agent location
        #remaining_things = tuple(loc for loc in curr_state[1:] if loc != new_agent)
        return new_agent# + remaining_things

    def action_cost(self, curr_state: tuple[tuple[int, int], ...],
                    action: str, next_state: tuple[tuple[int, int], ...]) -> float:
        """
        Returns the cost of performing an action.

        Args:
            curr_state (tuple[tuple[int, int], ...]): Current locations of things.
            action (str): Action applied.
            next_state (tuple[tuple[int, int], ...]): Next locations of things.

        Returns:
            float: Cost associated with the transition to next_state.
        """
        return 1 #self.maze[self.to_index(next_state[0][0], next_state[0][1])].cost

    def h(self, curr_pos: tuple[int, int]) -> float:
        """
        Estimates the cost to reach the goal using Manhattan distance.

        Args:
            curr_state (tuple[tuple[int, int], ...]): Locations of things.
            First tuple corresponds to the agent.
        Returns:
            float: Manhattan distance from agent to goal.
        """

        goal = self.goal_state.get_center()

        return float(abs(goal[0] - curr_pos[0]) ** 2 +
                     abs(goal[1] - curr_pos[1]) ** 2)






