import pygame
import pygame.gfxdraw
import math
import util, area
import problem, search, hastar, group


class Agent:
    speed = 0.5
    count = 0

    def __init__(self, pos, env, goal, pather, color=(0, 0, 255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.pather = pather
        self.color = color
        self.index = Agent.count
        # NEW: Use index as priority (lower index is higher priority)
        self.priority = Agent.count
        Agent.count += 1
        self.path_cache = []
        self.steps = 0

    def get_next_move(self):

        # Check if our path is ready from the pathfinder
        if self.index in self.pather.complete.keys():
            self.path_cache = self.pather.complete[self.index]
            # Remove from complete dict and waiting set
            del self.pather.complete[self.index]
            if self.index in self.pather.waiting:
                self.pather.waiting.remove(self.index)

        # If we have no path and aren't waiting for one, request a new path
        if len(self.path_cache) == 0 and self.index not in self.pather.waiting:
            # Create problem from CURRENT position
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal, 1000, 1000)

            # Agent's index is its priority. This is the key change for group pathfinding.
            self.pather.queue_path(self.index, prob)
            return (0, 0, 0)  # Wait for path

        if len(self.path_cache) == 0:
            return (0, 0, 0)  # Still waiting

        return self.path_cache.pop(0)

    def update(self):
        if self.goal.check_collision(self.pos):
            return True

        self.steps += 1

        dir = self.get_next_move()
        if len(dir) == 3:
            return False
        

        dir_length = util.length(dir)
        self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed, self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False

    def render(self, screen, offset, scale, render_path=False):
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale),
                           0.5 * scale)

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200, 200, 255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale),
                               0.2 * scale)
            

class HAAgent(Agent):
    """
    Agent that uses Hierarchical A* (H-A*) to find its path.
    """
    speed = 0.5
    count = 0

    def __init__(self, pos, env, goal, color=(0, 0, 255)):
        # Initialize as a standard Agent but maintain a separate count if needed
        self.pos = pos
        self.env = env
        self.goal = goal
        self.color = color
        self.index = HAAgent.count
        HAAgent.count += 1
        self.path_cache = []
        self.steps = 0
        # Agent.count will still track all agents if used elsewhere

    def get_next_move(self):
        # path_cache is a list of (dx, dy) moves
        if len(self.path_cache) == 0:
            # Use the Hierarchical A* search
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal, 1000, 1000)
            self.path_cache = hastar.hierarchical_astar_search(prob) # Call H-A* here

        if len(self.path_cache) == 0:
            return (0, 0, 0) # Path failed

        return self.path_cache.pop(0)

    # The update and render methods are inherited or identical to the base Agent class

    def update(self):
        if self.goal.check_collision(self.pos):
            return True

        self.steps += 1

        dir = self.get_next_move()
        if len(dir) == 3: # Check for the 'path failed' sentinel (0, 0, 0)
            return False

        # dir is a move vector (dx, dy)
        dir_length = util.length(dir)
        # Normalize the move direction but use the fixed agent speed
        if dir_length > 0:
            self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed,
                        self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False

    def render(self, screen, offset, scale, render_path=False):
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale),
                           0.5 * scale)

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200, 200, 255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale),
                               0.2 * scale)
 

class CoordinatedAgent(Agent):
    speed = 0.5
    count = 0

    def __init__(self, pos, env, goal, color=(0, 0, 255)):
        self.pos = pos
        self.env = env
        self.goal = goal
        self.color = color
        self.index = CoordinatedAgent.count
        CoordinatedAgent.count += 1
        self.path_cache = []
        self.steps = 0
        self.time_step = 0  # Tracks time for reservation system

    def get_next_move(self):
        if len(self.path_cache) == 0:
            prob = problem.ContinuousNavigation(self.pos, self.env, self.goal, 1000, 1000)

            # Run standard A*
            calculated_path = search.astar_search(prob)

            if calculated_path:
                # Reserve the entire path in the global system
                group.ReservationSystem.reserve_path(self.index, calculated_path, self.pos)
                self.path_cache = calculated_path

            if len(self.path_cache) == 0:
                return (0, 0, 0)

        # Look at the next move in the cache
        next_move = self.path_cache[0]

        # Calculate the potential next position
        dir_length = util.length(next_move)
        if dir_length == 0:
            next_pos = self.pos
        else:
            next_pos = (self.pos[0] + (next_move[0] / dir_length) * self.speed,
                        self.pos[1] + (next_move[1] / dir_length) * self.speed)

        # Check for immediate collision with other agents' future reservations
        # Check time_step + 1 because the move happens between self.time_step and self.time_step + 1
        if group.ReservationSystem.is_reserved(next_pos, self.time_step + 1, self.index):
            # Collision detected: stall for one time step
            return (0, 0, 0)  # Use the "no move" sentinel

        # Move is clear, pop it and return
        return self.path_cache.pop(0)

    def update(self):
        if self.goal.check_collision(self.pos):
            return True

        self.steps += 1
        self.time_step += 1  # Advance time

        dir = self.get_next_move()
        if len(dir) == 3:
            # If (0, 0, 0) is returned, the agent is waiting/stalling.
            return False

        dir_length = util.length(dir)
        if dir_length > 0:
            self.pos = (self.pos[0] + (dir[0] / dir_length) * self.speed,
                        self.pos[1] + (dir[1] / dir_length) * self.speed)
        return False

    def render(self, screen, offset, scale, render_path=False):
        # Implementation is identical to base Agent
        pygame.draw.circle(screen, self.color, ((self.pos[0] - offset[0]) * scale, (self.pos[1] - offset[1]) * scale),
                            0.5 * scale)

        if not render_path:
            return
        hold_x = self.pos[0]
        hold_y = self.pos[1]

        for node in self.path_cache:
            hold_x += node[0]
            hold_y += node[1]
            pygame.draw.circle(screen, (200, 200, 255), ((hold_x - offset[0]) * scale, (hold_y - offset[1]) * scale),
                                0.2 * scale)

